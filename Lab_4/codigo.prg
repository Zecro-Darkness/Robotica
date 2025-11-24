#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose   # Para leer la orientación

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Suscripción a la pose para leer el ángulo real
        self.current_theta = None
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Paso de tiempo del timer
        self.dt = 0.05

        # Velocidades generales (O, J, L, M, B, A, T)
        self.v = 1.0
        self.omega = 1.2  # rad/s

                # Parámetros específicos para la letra O (círculo)
        self.v_o = 1.25      # para que el radio sea 1.25 y el diámetro 2.5
        self.omega_o = 1.2  # mantenemos la misma omega

                # Parámetros específicos para el gancho de la J
        self.v_j_arc = 0.67       # velocidad más pequeña → radio más chico
        self.omega_j_arc = self.omega  # usamos la misma omega (1.2)



        # Velocidades específicas para la S
        self.v_s = 1.0
        self.omega_s = 2.025

        # Duración de tramos S
        self.T_s_straight = 0.7

        # Duración de tramos J
        self.j_straight_time = 1.8

        # Duración de tramos L
        self.l_vertical_time = 2.5      # tramo vertical (más largo)
        self.l_horizontal_time = 1.4    # tramo horizontal (más corto)

        # Duración de tramos M
        self.m_vertical_time = 2.5
        ratio_m = 0.75  # altura vertical de diagonales = 3/4 de la vertical
        self.m_diag_time = self.m_vertical_time * ratio_m / math.sin(math.pi / 4.0)

        # ---- Parámetros de la B ----
        self.b_vertical_time = 2.5
        self.b_mid_down_time = 0.05

        v_vert = self.v * 0.8
        H_b = v_vert * self.b_vertical_time          # longitud del tramo vertical de la B
        radius_b = H_b / 4.0                         # diámetro = H/2

        self.b_arc_omega = self.omega
        self.b_arc_v = radius_b * self.b_arc_omega   # v = r * ω
        self.b_arc_time = math.pi / abs(self.b_arc_omega)  # semicirculo

        # ---- Parámetros de la A ----
        self.a_diag_time = 2.89
        self.a_back_half_time = self.a_diag_time / 2.0
        # Barra horizontal: 3/4 del largo de la diagonal (tu cambio)
        self.a_bar_time = self.a_diag_time * 0.65

        # ---- Parámetros de la T ----
        self.t_vertical_time = 2.5
        # barra izquierda: 1/4 del vertical
        self.t_bar_quarter_time = self.t_vertical_time * 0.40
        # barra derecha: 1/2 del vertical
        self.t_bar_half_time = self.t_vertical_time * 0.9

        # Estado del movimiento
        # circle,
        # j_orient, j_step1, j_step2,
        # l_orient, l_step1, l_orient2, l_step2,
        # m_orient1, m_step1, m_orient2, m_step2,
        # m_orient3, m_step3, m_orient4, m_step4,
        # b_orient1, b_step1, b_orient2, b_arc1,
        # b_orient3, b_step2, b_orient4, b_arc2,
        # a_orient1, a_step1, a_orient2, a_step2,
        # a_orient3, a_step3, a_orient4, a_step4,
        # t_orient1, t_step1, t_orient2, t_step2, t_orient3, t_step3,
        # s_step0, s_step1, s_step2, s_step3, s_step4
        self.state = None
        self.end_time = 0.0

        # Objetivos de orientación para M
        self.m_target_up = 0.0
        self.m_target_diag1 = 0.0
        self.m_target_diag2 = 0.0
        self.m_target_down = 0.0

        # Objetivos de orientación para B
        self.b_target_up = 0.0
        self.b_target_right = 0.0
        self.b_target_down = 0.0

        # Objetivos de orientación para A
        self.a_target_diag1 = 0.0
        self.a_target_diag2 = 0.0
        self.a_target_back = 0.0
        self.a_target_bar = 0.0

        # Objetivos de orientación para T
        self.t_target_up = 0.0
        self.t_target_left = 0.0
        self.t_target_right = 0.0

        self.timer = self.create_timer(self.dt, self.move_turtle)
        self._publish_stop()

        # Hilo teclado
        self.key_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.key_thread.start()

        self.get_logger().info(
            "Listo. 'o'→O, 'j'→J, 's'→S, 'l'→L, 'm'→M, 'b'→B, 'a'→A, 't'→T."
        )

    # ----------------------------- CALLBACK POSE -----------------------------
    def pose_callback(self, msg: Pose):
        self.current_theta = msg.theta

    # Normaliza diferencia de ángulo a [-pi, pi]
    def angle_diff(self, target, current):
        diff = target - current
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff

    # ----------------------------- TECLADO -----------------------------
    def _keyboard_loop(self):
        while rclpy.ok():
            try:
                s = input().strip().lower()
            except EOFError:
                break

            if s == 'o':
                self.start_circle()
            elif s == 'j':
                self.start_letter_j()
            elif s == 's':
                self.start_letter_s()
            elif s == 'l':
                self.start_letter_l()
            elif s == 'm':
                self.start_letter_m()
            elif s == 'b':
                self.start_letter_b()
            elif s == 'a':
                self.start_letter_a()
            elif s == 't':
                self.start_letter_t()

    # ----------------------------- O (círculo) ------------------------
    def start_circle(self):
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return

        T = 2 * math.pi / abs(self.omega_o)
        self.state = "circle"
        self.end_time = time.monotonic() + T
        self.get_logger().info("→ Iniciando letra O (círculo).")

    # ----------------------------- LETRA J ----------------------------
    def start_letter_j(self):
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return

        self.get_logger().info("→ Iniciando letra J.")
        self.state = "j_orient"
        self.target_down = -math.pi / 2.0

    # ----------------------------- LETRA L ----------------------------
    def start_letter_l(self):
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return

        self.get_logger().info("→ Iniciando letra L.")
        self.state = "l_orient"
        self.target_down = -math.pi / 2.0
        self.target_right = 0.0

    # ----------------------------- LETRA M ----------------------------
    def start_letter_m(self):
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return
        if self.current_theta is None:
            self.get_logger().warn("Aún no tengo pose de la tortuga; intenta de nuevo en un momento.")
            return

        self.get_logger().info("→ Iniciando letra M.")

        base = self.current_theta
        self.m_target_up = base + math.pi / 2.0
        self.m_target_diag1 = base - math.pi / 4.0
        self.m_target_diag2 = base + math.pi / 4.0
        self.m_target_down = base - math.pi / 2.0

        self.state = "m_orient1"

    # ----------------------------- LETRA B ----------------------------
    def start_letter_b(self):
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return

        self.get_logger().info("→ Iniciando letra B.")
        self.b_target_up = math.pi / 2.0
        self.b_target_right = 0.0
        self.b_target_down = -math.pi / 2.0

        self.state = "b_orient1"

    # ----------------------------- LETRA A ----------------------------
    def start_letter_a(self):
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return
        if self.current_theta is None:
            self.get_logger().warn("Aún no tengo pose de la tortuga; intenta de nuevo en un momento.")
            return

        self.get_logger().info("→ Iniciando letra A.")

        base = self.current_theta
        # 1ª diagonal: +60°
        self.a_target_diag1 = base + math.pi / 3.0
        # 2ª diagonal: -60°
        self.a_target_diag2 = base - math.pi / 3.0
        # Volver por 2ª diagonal (opuesta): +180°
        self.a_target_back = self.a_target_diag2 + math.pi
        # Barra horizontal: base + 180° (izquierda)
        self.a_target_bar = base + math.pi

        self.state = "a_orient1"

    # ----------------------------- LETRA T ----------------------------
    def start_letter_t(self):
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return
        if self.current_theta is None:
            self.get_logger().warn("Aún no tengo pose de la tortuga; intenta de nuevo en un momento.")
            return

        self.get_logger().info("→ Iniciando letra T.")

        base = self.current_theta
        # Giro 90° CCW → arriba
        self.t_target_up = base + math.pi / 2.0
        # 90° CCW desde arriba → izquierda
        self.t_target_left = self.t_target_up + math.pi / 2.0
        # 180° desde izquierda → derecha
        self.t_target_right = self.t_target_left + math.pi

        self.state = "t_orient1"

    # ----------------------------- LETRA S ----------------------------
    def start_letter_s(self):
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return

        self.get_logger().info("→ Iniciando letra S.")
        self.state = "s_step0"
        self.end_time = time.monotonic() + self.T_s_straight

    # ----------------------------- TIMER ------------------------------
    def move_turtle(self):
        now = time.monotonic()

        if self.state is None:
            return

        # -------- O (círculo) --------
        if self.state == "circle":
            if now >= self.end_time:
                self.stop_movement("Letra O completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v_o
                msg.angular.z = self.omega_o
                self.publisher_.publish(msg)
            return

        # ===================== LETRA J =====================
        if self.state == "j_orient":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.target_down, self.current_theta)
            if abs(error) < 0.02:
                self.state = "j_step1"
                self.end_time = time.monotonic() + self.j_straight_time
                self.get_logger().info("→ Trazo vertical de la J.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "j_step1":
            if now >= self.end_time:
                self.state = "j_step2"
                self.end_time = time.monotonic() + (math.pi / abs(self.omega))
                self.get_logger().info("→ Curva inferior de la J.")
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "j_step2":
            if now >= self.end_time:
                self.stop_movement("Letra J completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v_j_arc
                msg.angular.z = -self.omega_j_arc
                self.publisher_.publish(msg)
            return

        # ===================== LETRA L =====================
        if self.state == "l_orient":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.target_down, self.current_theta)
            if abs(error) < 0.02:
                self.state = "l_step1"
                self.end_time = time.monotonic() + self.l_vertical_time
                self.get_logger().info("→ Trazo vertical de la L.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "l_step1":
            if now >= self.end_time:
                self.state = "l_orient2"
                self.get_logger().info("→ Giro para tramo horizontal de la L.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "l_orient2":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.target_right, self.current_theta)
            if abs(error) < 0.02:
                self.state = "l_step2"
                self.end_time = time.monotonic() + self.l_horizontal_time
                self.get_logger().info("→ Trazo horizontal de la L.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "l_step2":
            if now >= self.end_time:
                self.stop_movement("Letra L completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        # ===================== LETRA M =====================
        if self.state == "m_orient1":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.m_target_up, self.current_theta)
            if abs(error) < 0.02:
                self.state = "m_step1"
                self.end_time = time.monotonic() + self.m_vertical_time
                self.get_logger().info("→ Primer tramo vertical de la M (hacia arriba).")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "m_step1":
            if now >= self.end_time:
                self.state = "m_orient2"
                self.get_logger().info("→ Giro para primer tramo diagonal de la M.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "m_orient2":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.m_target_diag1, self.current_theta)
            if abs(error) < 0.02:
                self.state = "m_step2"
                self.end_time = time.monotonic() + self.m_diag_time
                self.get_logger().info("→ Primer tramo diagonal de la M.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "m_step2":
            if now >= self.end_time:
                self.state = "m_orient3"
                self.get_logger().info("→ Giro para segundo tramo diagonal de la M.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "m_orient3":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.m_target_diag2, self.current_theta)
            if abs(error) < 0.02:
                self.state = "m_step3"
                self.end_time = time.monotonic() + self.m_diag_time
                self.get_logger().info("→ Segundo tramo diagonal de la M.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "m_step3":
            if now >= self.end_time:
                self.state = "m_orient4"
                self.get_logger().info("→ Giro para último tramo vertical de la M.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "m_orient4":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.m_target_down, self.current_theta)
            if abs(error) < 0.02:
                self.state = "m_step4"
                self.end_time = time.monotonic() + self.m_vertical_time
                self.get_logger().info("→ Último tramo vertical de la M (hacia abajo).")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "m_step4":
            if now >= self.end_time:
                self.stop_movement("Letra M completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        # ===================== LETRA B =====================
        if self.state == "b_orient1":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.b_target_up, self.current_theta)
            if abs(error) < 0.02:
                self.state = "b_step1"
                self.end_time = time.monotonic() + self.b_vertical_time
                self.get_logger().info("→ Tramo vertical de la B (espina).")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "b_step1":
            if now >= self.end_time:
                self.state = "b_orient2"
                self.get_logger().info("→ Giro para primer lóbulo de la B.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8   # hacia arriba
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "b_orient2":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.b_target_right, self.current_theta)
            if abs(error) < 0.02:
                self.state = "b_arc1"
                self.end_time = time.monotonic() + self.b_arc_time
                self.get_logger().info("→ Primer semicírculo de la B (lóbulo superior).")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "b_arc1":
            if now >= self.end_time:
                self.state = "b_orient3"
                self.get_logger().info("→ Giro para tramo entre lóbulos.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.b_arc_v
                msg.angular.z = -self.b_arc_omega  # horario
                self.publisher_.publish(msg)
            return

        if self.state == "b_orient3":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.b_target_down, self.current_theta)
            if abs(error) < 0.02:
                self.state = "b_step2"
                self.end_time = time.monotonic() + self.b_mid_down_time
                self.get_logger().info("→ Pequeño tramo entre lóbulos de la B.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "b_step2":
            if now >= self.end_time:
                self.state = "b_orient4"
                self.get_logger().info("→ Giro para segundo lóbulo de la B.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8   # hacia abajo (muy poco tiempo)
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "b_orient4":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.b_target_right, self.current_theta)
            if abs(error) < 0.02:
                self.state = "b_arc2"
                self.end_time = time.monotonic() + self.b_arc_time
                self.get_logger().info("→ Segundo semicírculo de la B (lóbulo inferior).")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "b_arc2":
            if now >= self.end_time:
                self.stop_movement("Letra B completada.")
            else:
                msg = Twist()
                msg.linear.x = self.b_arc_v
                msg.angular.z = -self.b_arc_omega  # horario
                self.publisher_.publish(msg)
            return

        # ===================== LETRA A =====================
        if self.state == "a_orient1":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.a_target_diag1, self.current_theta)
            if abs(error) < 0.02:
                self.state = "a_step1"
                self.end_time = time.monotonic() + self.a_diag_time
                self.get_logger().info("→ Primer tramo diagonal de la A.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "a_step1":
            if now >= self.end_time:
                self.state = "a_orient2"
                self.get_logger().info("→ Giro para segunda diagonal de la A.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "a_orient2":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.a_target_diag2, self.current_theta)
            if abs(error) < 0.02:
                self.state = "a_step2"
                self.end_time = time.monotonic() + self.a_diag_time
                self.get_logger().info("→ Segundo tramo diagonal de la A.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "a_step2":
            if now >= self.end_time:
                self.state = "a_orient3"
                self.get_logger().info("→ Giro para devolver por la segunda diagonal.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "a_orient3":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.a_target_back, self.current_theta)
            if abs(error) < 0.02:
                self.state = "a_step3"
                self.end_time = time.monotonic() + self.a_back_half_time
                self.get_logger().info("→ Volviendo hasta la mitad de la segunda diagonal.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "a_step3":
            if now >= self.end_time:
                self.state = "a_orient4"
                self.get_logger().info("→ Giro para la barra horizontal de la A.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "a_orient4":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.a_target_bar, self.current_theta)
            if abs(error) < 0.02:
                self.state = "a_step4"
                self.end_time = time.monotonic() + self.a_bar_time
                self.get_logger().info("→ Barra horizontal de la A.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        if self.state == "a_step4":
            if now >= self.end_time:
                self.stop_movement("Letra A completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        # ===================== LETRA T =====================
        # 1) Orientar hacia arriba
        if self.state == "t_orient1":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.t_target_up, self.current_theta)
            if abs(error) < 0.02:
                self.state = "t_step1"
                self.end_time = time.monotonic() + self.t_vertical_time
                self.get_logger().info("→ Tramo vertical de la T.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        # 2) Tramo vertical
        if self.state == "t_step1":
            if now >= self.end_time:
                self.state = "t_orient2"
                self.get_logger().info("→ Giro para barra hacia la izquierda de la T.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8   # hacia arriba
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        # 3) Orientar hacia la izquierda
        if self.state == "t_orient2":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.t_target_left, self.current_theta)
            if abs(error) < 0.02:
                self.state = "t_step2"
                self.end_time = time.monotonic() + self.t_bar_quarter_time
                self.get_logger().info("→ Barra horizontal hacia la izquierda (1/4).")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        # 4) Tramo horizontal hacia la izquierda (1/4)
        if self.state == "t_step2":
            if now >= self.end_time:
                self.state = "t_orient3"
                self.get_logger().info("→ Giro para barra hacia la derecha (1/2).")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8   # hacia la izquierda
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        # 5) Orientar hacia la derecha (180° desde izquierda)
        if self.state == "t_orient3":
            if self.current_theta is None:
                return
            error = self.angle_diff(self.t_target_right, self.current_theta)
            if abs(error) < 0.02:
                self.state = "t_step3"
                self.end_time = time.monotonic() + self.t_bar_half_time
                self.get_logger().info("→ Barra horizontal hacia la derecha (1/2).")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = self.omega if error > 0 else -self.omega
                self.publisher_.publish(msg)
            return

        # 6) Tramo horizontal hacia la derecha (1/2)
        if self.state == "t_step3":
            if now >= self.end_time:
                self.stop_movement("Letra T completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8   # hacia la derecha
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        # ===================== LETRA S (igual que antes) =====================

        if self.state == "s_step0":
            if now >= self.end_time:
                self.state = "s_step1"
                T_arc = math.pi / abs(self.omega_s)
                self.end_time = time.monotonic() + T_arc
                self.get_logger().info("→ Primer semicírculo de la S.")
            else:
                msg = Twist()
                msg.linear.x = self.v_s
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "s_step1":
            if now >= self.end_time:
                self.state = "s_step2"
                T_straight_mid = (math.pi / abs(self.omega_s)) / 6.0
                self.end_time = time.monotonic() + T_straight_mid
                self.get_logger().info("→ Tramo recto central de la S.")
            else:
                msg = Twist()
                msg.linear.x = self.v_s
                msg.angular.z = self.omega_s
                self.publisher_.publish(msg)
            return

        if self.state == "s_step2":
            if now >= self.end_time:
                self.state = "s_step3"
                T_arc = math.pi / abs(self.omega_s)
                self.end_time = time.monotonic() + T_arc
                self.get_logger().info("→ Segundo semicírculo de la S.")
            else:
                msg = Twist()
                msg.linear.x = self.v_s
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

        if self.state == "s_step3":
            if now >= self.end_time:
                self.state = "s_step4"
                self.end_time = time.monotonic() + self.T_s_straight
                self.get_logger().info("→ Tramo recto final de la S.")
            else:
                msg = Twist()
                msg.linear.x = self.v_s
                msg.angular.z = -self.omega_s
                self.publisher_.publish(msg)
            return

        if self.state == "s_step4":
            if now >= self.end_time:
                self.stop_movement("Letra S completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v_s
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            return

    # ----------------------------- STOP -------------------------------
    def stop_movement(self, message):
        self._publish_stop()
        self.state = None
        self.get_logger().info(message)

    def _publish_stop(self):
        self.publisher_.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

