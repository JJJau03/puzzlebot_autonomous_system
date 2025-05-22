#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import math

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        # ------------- Declaración de parámetros -------------
        self.declare_parameter('vision_height', 10,
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                                   description='Altura (px) del ROI inferior-central'))
        self.declare_parameter('roi_center_width', 140,
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                                   description='Ancho (px) del ROI'))
        self.declare_parameter('threshold_value', 80,  ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('min_area',       15,  ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('max_area',      1000,  ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('blur_kernel',     9,  ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('morph_kernel',    7,  ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('erode_iterations', 3, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('dilate_iterations',3, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('use_edge_detection', True,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('memory_factor', 0.7,  ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('kp', 0.3,  ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('ki', 0.0,  ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('kd', 0.0,  ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('kp_angle', 0.08, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('ki_angle', 0.0,  ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('kd_angle', 0.0,  ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('angle_weight', 0.4, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('target_angle', 0.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('linear_speed', 0.07, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('max_angular_speed', 0.15, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('recovery_mode', True, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('debug_mode',    True, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))

        # ------------- Inicialización -------------
        self.update_parameters()
        self.create_timer(1.0, self.update_parameters)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            CompressedImage, '/video_source/compressed',
            self.image_callback, qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Ventanas de depuración
        if self.debug_mode:
            cv2.namedWindow('Imagen Original', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Procesamiento',   cv2.WINDOW_NORMAL)
            cv2.namedWindow('Control',         cv2.WINDOW_NORMAL)
            if self.use_edge_detection:
                cv2.namedWindow('Bordes',      cv2.WINDOW_NORMAL)

        # Variables de estado
        self.accumulated_error = 0.0
        self.previous_error    = 0.0
        self.max_accumulated_error = 1000.0
        self.accumulated_angle_error = 0.0
        self.previous_angle_error   = 0.0
        self.max_accumulated_angle_error = 1000.0

        self.last_valid_centroid  = None
        self.last_valid_angle     = 0.0
        self.last_valid_endpoints = None
        self.last_valid_error     = 0.0
        self.recovery_direction   = 1
        self.confidence           = 0.0
        self.consecutive_detections = 0
        self.consecutive_misses     = 0

        self.scale_factor = 2.0            # factor para rellenar la pantalla
        self.get_logger().info('line_follower iniciado ✅')

    # ---------- Parámetros dinámicos ----------
    def update_parameters(self):
        gp = self.get_parameter
        self.vision_height     = gp('vision_height').value
        self.roi_center_width  = gp('roi_center_width').value
        self.threshold_value   = gp('threshold_value').value
        self.min_area          = gp('min_area').value
        self.max_area          = gp('max_area').value
        self.blur_kernel       = gp('blur_kernel').value
        self.morph_kernel      = gp('morph_kernel').value
        self.erode_iterations  = gp('erode_iterations').value
        self.dilate_iterations = gp('dilate_iterations').value
        self.use_edge_detection= gp('use_edge_detection').value
        self.memory_factor     = max(0.0, min(1.0, gp('memory_factor').value))
        self.kp, self.ki, self.kd = gp('kp').value, gp('ki').value, gp('kd').value
        self.kp_angle, self.ki_angle, self.kd_angle = gp('kp_angle').value, gp('ki_angle').value, gp('kd_angle').value
        self.angle_weight      = max(0.0, min(1.0, gp('angle_weight').value))
        self.target_angle      = gp('target_angle').value
        self.linear_speed      = gp('linear_speed').value
        self.max_angular_speed = gp('max_angular_speed').value
        self.recovery_mode     = gp('recovery_mode').value
        self.debug_mode        = gp('debug_mode').value

    # ---------- Helpers ----------
    def show(self, name, frame, scale=None):
        """Muestra la imagen reescalada para llenar la ventana."""
        if scale is None:
            scale = self.scale_factor
        if scale != 1.0:
            frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        cv2.imshow(name, frame)

    # ---------- Callback principal ----------
    def image_callback(self, msg: CompressedImage):
        try:
            # 1) Decodificar y rotar 180°
            img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = cv2.rotate(img, cv2.ROTATE_180)

            # 2) ROI inferior-centro (limitado)
            h, w = img.shape[:2]
            roi_h = min(self.vision_height, h)          # <-- límite
            cx = w // 2
            half = self.roi_center_width // 2
            roi = img[h - roi_h: h,
                      max(0, cx - half): min(w, cx + half)]
            roi_h, roi_w = roi.shape[:2]
            local_target_x = roi_w // 2

            original_view = roi.copy()

            # 3) Preprocesamiento
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            k = self.blur_kernel + (self.blur_kernel % 2 == 0)
            blurred = cv2.GaussianBlur(gray, (k, k), 2.0)

            binary = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV, 51, 10)
            _, binary_g = cv2.threshold(blurred, self.threshold_value, 255,
                                        cv2.THRESH_BINARY_INV)
            binary = cv2.bitwise_or(binary, binary_g)

            if self.use_edge_detection:
                edges = cv2.Canny(blurred, 50, 150)
                edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), 1)
                binary = cv2.bitwise_or(binary, edges)
                edge_view = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

            kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
            morph = cv2.erode(binary, kernel, self.erode_iterations)
            morph = cv2.dilate(morph, kernel, self.dilate_iterations)

            # 4) Componentes conectados
            n, labels, stats, cents = cv2.connectedComponentsWithStats(morph, 8)

            best_idx, best_score = -1, -1
            for i in range(1, n):
                x, y, ww, hh, area = stats[i]
                cx_i, cy_i = cents[i]
                if self.min_area <= area <= self.max_area:
                    score = area / (abs(cx_i - local_target_x) + 1)
                    if score > best_score:
                        best_idx, best_score = i, score
                        best_cx, best_cy, best_area = cx_i, cy_i, area

            current_c, current_ang, current_eps, conf = None, 0.0, None, 0.0
            if best_idx > 0:
                current_c = np.array([best_cx, best_cy])
                mask = np.zeros_like(morph)
                mask[labels == best_idx] = 255
                cnt, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if cnt:
                    vx, vy, x0, y0 = cv2.fitLine(cnt[0], cv2.DIST_L2, 0, 0.01, 0.01)
                    current_ang = (math.degrees(math.atan2(vy, vx)) - 90 + 180) % 180 - 90
                    length = max(stats[best_idx][3], 30)
                    p1 = (int(x0 - vx * length), int(y0 - vy * length))
                    p2 = (int(x0 + vx * length), int(y0 + vy * length))
                    current_eps = (p1, p2)
                conf = min(1.0, best_area / (self.min_area * 3))
                self.consecutive_detections += 1
                self.consecutive_misses = 0
            else:
                self.consecutive_detections = 0
                self.consecutive_misses += 1

            # 5) Suavizado
            final_c, final_ang, final_eps = None, None, None
            if current_c is not None:
                if self.last_valid_centroid is not None:
                    final_c = (1 - self.memory_factor) * current_c + self.memory_factor * self.last_valid_centroid
                    final_ang = (1 - self.memory_factor) * current_ang + self.memory_factor * self.last_valid_angle
                else:
                    final_c, final_ang = current_c, current_ang
                final_eps = current_eps
                self.last_valid_centroid, self.last_valid_angle, self.last_valid_endpoints = final_c, final_ang, final_eps
                self.confidence = conf
            elif self.last_valid_centroid is not None and self.consecutive_misses < 10:
                final_c, final_ang, final_eps = self.last_valid_centroid, self.last_valid_angle, self.last_valid_endpoints
                self.confidence = max(0.0, self.confidence - 0.1)
            else:
                self.last_valid_centroid = None
                self.confidence = 0.0

            # 6) Control
            if final_c is not None:
                pos_err = local_target_x - int(final_c[0])
                self.last_valid_error = pos_err
                ang_err = self.target_angle - final_ang
                self.apply_pid(pos_err, ang_err, self.confidence, roi_w)
            else:
                if self.recovery_mode and self.consecutive_misses > 5:
                    self.recovery_behavior()
                else:
                    self.stop_robot()

            # 7) Debug visual
            if self.debug_mode:
                self.show('Imagen Original', original_view)
                self.show('Procesamiento', cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR))
                if final_c is not None:
                    cview = original_view.copy()
                    cv2.circle(cview, (int(final_c[0]), int(final_c[1])), 6, (0, 0, 255), -1)
                    self.show('Control', cview)
                else:
                    self.show('Control', original_view)
                if self.use_edge_detection:
                    self.show('Bordes', edge_view)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.stop_robot()

    # ---------- PID combinado ----------
    def apply_pid(self, pos_err, ang_err, conf, roi_w):
        # PID posición
        self.accumulated_error += pos_err * conf
        self.accumulated_error = max(-self.max_accumulated_error,
                                     min(self.accumulated_error, self.max_accumulated_error))
        d_pos = pos_err - self.previous_error
        self.previous_error = pos_err
        pos_w = self.kp * pos_err * conf + self.ki * self.accumulated_error + self.kd * d_pos * conf

        # PID ángulo
        self.accumulated_angle_error += ang_err * conf
        self.accumulated_angle_error = max(-self.max_accumulated_angle_error,
                                           min(self.accumulated_angle_error, self.max_accumulated_angle_error))
        d_ang = ang_err - self.previous_angle_error
        self.previous_angle_error = ang_err
        ang_w = (self.kp_angle * ang_err * conf +
                 self.ki_angle * self.accumulated_angle_error +
                 self.kd_angle * d_ang * conf)

        omega = ((1 - self.angle_weight) * pos_w +
                 self.angle_weight * ang_w)
        omega = max(-self.max_angular_speed, min(self.max_angular_speed, omega))

        err_mag = abs(pos_err) / (roi_w / 2)
        v = self.linear_speed * (1 - min(0.7, err_mag)) * min(1.0, conf * 1.2)

        t = Twist()
        t.linear.x, t.angular.z = v, omega
        self.cmd_vel_pub.publish(t)

    # ---------- Recuperación ----------
    def recovery_behavior(self):
        if self.last_valid_error != 0:
            self.recovery_direction = -1 if self.last_valid_error > 0 else 1
        t = Twist()
        t.linear.x  = self.linear_speed * 0.3
        t.angular.z = self.recovery_direction * self.max_angular_speed * 0.7
        self.cmd_vel_pub.publish(t)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

# ----------------------- main -----------------------
def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()