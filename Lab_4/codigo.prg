#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose   # Para leer la orientación de la tortuga


class TurtleController(Node):
    """
    Nodo que controla a la tortuga de turtlesim para dibujar letras
    (O, J, S, L, M, B, A, T) usando comandos de velocidad (Twist).
    """

    def __init__(self):
        super().__init__('turtle_controller')

        # Publicador de velocidades: /turtle1/cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Suscripción a la pose para leer el ángulo actual (theta)
        self.current_theta = None
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Periodo del timer principal (control "discreto")
        self.dt = 0.05

        # Tolerancia angular para considerar que ya se alcanzó una orientación objetivo
        self.angle_tolerance = 0.02

        # ----------------- PARÁMETROS GLOBALES DE VELOCIDAD -----------------
        # Velocidades base para la mayoría de letras
        self.v = 1.0          # [m/s] velocidad lineal
        self.omega = 1.2      # [rad/s] velocidad angular

        # -------- LETRA O (CÍRCULO COMPLETO) --------
        # Estos parámetros definen el radio de la O: r = v_o / omega_o
        self.v_o = 1.25       # velocidad lineal para la O
        self.omega_o = 1.2    # velocidad angular para la O

        # -------- GANCHO DE LA J --------
        # Se hace con un semicírculo de radio más pequeño para que la J
        # tenga una altura similar a la L y se vea proporcionada.
        self.v_j_arc = 0.67          # velocidad lineal en el gancho
        self.omega_j_arc = self.omega  # misma omega de siempre

        # -------- LETRA S --------
        # Estos parámetros controlan la "altura" de la S (dos semicírculos).
        self.v_s = 1.0
        self.omega_s = 2.025

        # Duración del tramo recto inicial y final de la S
        self.T_s_straight = 0.7

        # -------- LETRA J --------
        # Tiempo durante el cual se dibuja el tramo vertical de la J
        self.j_straight_time = 1.8

        # -------- LETRA L --------
        # Tiempos de los tramos rectos de la L
        self.l_vertical_time = 2.5      # tramo vertical (más largo)
        self.l_horizontal_time = 1.4    # tramo horizontal (más corto)

        # -------- LETRA M --------
        # Altura de los tramos verticales:
        self.m_vertical_time = 2.5
        # Proporción de la altura que se desea para las diagonales
        ratio_m = 0.75  # diagonales ≈ 3/4 de la altura vertical
        # Tiempo de cada diagonal (ajustado para que la altura sea la deseada)
        self.m_diag_time = self.m_vertical_time * ratio_m / math.sin(math.pi / 4.0)

        # -------- LETRA B --------
        self.b_vertical_time = 2.5
        self.b_mid_down_time = 0.05  # pequeño tramo entre lóbulos

        # Cálculo del radio de los lóbulos a partir de la altura de la espina
        v_vert = self.v * 0.8
        H_b = v_vert * self.b_vertical_time          # longitud del tramo vertical
        radius_b = H_b / 4.0                         # radio tal que diámetro total = H/2

        self.b_arc_omega = self.omega
        self.b_arc_v = radius_b * self.b_arc_omega   # v = r * ω
        self.b_arc_time = math.pi / abs(self.b_arc_omega)  # tiempo de un semicírculo

        # -------- LETRA A --------
        # Tiempo de cada diagonal
        self.a_diag_time = 2.89
        # Tiempo para volver solo hasta la mitad de la segunda diagonal
        self.a_back_half_time = self.a_diag_time / 2.0
        # Tiempo de la barra horizontal (ajustado para que se vea proporcionada)
        self.a_bar_time = self.a_diag_time * 0.65

        # -------- LETRA T --------
        self.t_vertical_time = 2.5
        # Barra hacia la izquierda ~ 40% de la altura del tramo vertical
        self.t_bar_quarter_time = self.t_vertical_time * 0.40
        # Barra hacia la derecha ~ 90% de la altura del tramo vertical
        self.t_bar_half_time = self.t_vertical_time * 0.9

        # ----------------- MÁQUINA DE ESTADOS -----------------
        # state: indica en qué parte del "dibujo" vamos (orientar, tramo1, tramo2, etc.)
        self.state = None
        # end_time: instante de tiempo en el que debe terminar el tramo actual
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

        # Timer principal: llama periódicamente a move_turtle()
        self.timer = self.create_timer(self.dt, self.move_turtle)

        # Aseguramos que la tortuga arranca quieta
        self._publish_stop()

        # Hilo independiente para leer el teclado (input bloquea)
        self.key_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.key_thread.start()

        self.get_logger().info(
            "Listo. 'o'→O, 'j'→J, 's'→S, 'l'→L, 'm'→M, 'b'→B, 'a'→A, 't'→T."
        )

    # ----------------------------- CALLBACK POSE -----------------------------
    def pose_callback(self, msg: Pose):
        """Callback que actualiza la orientación actual de la tortuga."""
        self.current_theta = msg.theta

    # Normaliza diferencia de ángulo a [-pi, pi]
    def angle_diff(self, target, current):
        """Devuelve la diferencia angular entre target y current en [-π, π]."""
        diff = target - current
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff

    # Helper genérico para todas las orientaciones
    def _rotate_and_schedule(self, target, next_state, next_duration, log_msg):
        """
        Gira la tortuga hasta apuntar a 'target'.
        Cuando se logra (error < angle_tolerance), cambia a 'next_state'
        y programa que ese estado dure 'next_duration' segundos.
        """
        if self.current_theta is None:
            # Aún no se ha recibido ninguna pose
            return

        error = self.angle_diff(target, self.current_theta)
        if abs(error) < self.angle_tolerance:
            # Ya estamos suficientemente cerca del ángulo objetivo
            self.state = next_state
            self.end_time = time.monotonic() + next_duration
            self.get_logger().info(log_msg)
            self._publish_stop()
        else:
            # Seguimos girando hacia el objetivo
            msg = Twist()
            msg.angular.z = self.omega if error > 0 else -self.omega
            self.publisher_.publish(msg)

    # ----------------------------- TECLADO -----------------------------
    def _keyboard_loop(self):
        """
        Hilo que espera teclas por consola.
        Según la letra, se inicia el dibujo correspondiente.
        """
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
        """Inicia el dibujo de la letra O (un círculo completo)."""
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return

        # Tiempo para un círculo completo: 2π / ω
        T = 2 * math.pi / abs(self.omega_o)
        self.state = "circle"
        self.end_time = time.monotonic() + T
        self.get_logger().info("→ Iniciando letra O (círculo).")

    # ----------------------------- LETRA J ----------------------------
    def start_letter_j(self):
        """Inicia el dibujo de la letra J."""
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return

        self.get_logger().info("→ Iniciando letra J.")
        self.state = "j_orient"
        # Queremos que la tortuga apunte hacia abajo (-π/2)
        self.target_down = -math.pi / 2.0

    # ----------------------------- LETRA L ----------------------------
    def start_letter_l(self):
        """Inicia el dibujo de la letra L."""
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return

        self.get_logger().info("→ Iniciando letra L.")
        self.state = "l_orient"
        self.target_down = -math.pi / 2.0  # primero vertical hacia abajo
        self.target_right = 0.0            # luego horizontal hacia la derecha

    # ----------------------------- LETRA M ----------------------------
    def start_letter_m(self):
        """Inicia el dibujo de la letra M."""
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return
        if self.current_theta is None:
            self.get_logger().warn("Aún no tengo pose de la tortuga; intenta de nuevo en un momento.")
            return

        self.get_logger().info("→ Iniciando letra M.")

        # Se definen las orientaciones relativas a la orientación actual
        base = self.current_theta
        self.m_target_up = base + math.pi / 2.0      # vertical hacia arriba
        self.m_target_diag1 = base - math.pi / 4.0   # diagonal hacia el centro
        self.m_target_diag2 = base + math.pi / 4.0   # diagonal hacia el otro lado
        self.m_target_down = base - math.pi / 2.0    # vertical hacia abajo

        self.state = "m_orient1"

    # ----------------------------- LETRA B ----------------------------
    def start_letter_b(self):
        """Inicia el dibujo de la letra B."""
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return

        self.get_logger().info("→ Iniciando letra B.")
        # Espina vertical (hacia arriba), luego lóbulos derechos
        self.b_target_up = math.pi / 2.0
        self.b_target_right = 0.0
        self.b_target_down = -math.pi / 2.0

        self.state = "b_orient1"

    # ----------------------------- LETRA A ----------------------------
    def start_letter_a(self):
        """Inicia el dibujo de la letra A."""
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
        # Volver por la 2ª diagonal (dirección opuesta): +180°
        self.a_target_back = self.a_target_diag2 + math.pi
        # Barra horizontal central: apuntando a la izquierda (base+π)
        self.a_target_bar = base + math.pi

        self.state = "a_orient1"

    # ----------------------------- LETRA T ----------------------------
    def start_letter_t(self):
        """Inicia el dibujo de la letra T."""
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return
        if self.current_theta is None:
            self.get_logger().warn("Aún no tengo pose de la tortuga; intenta de nuevo en un momento.")
            return

        self.get_logger().info("→ Iniciando letra T.")

        base = self.current_theta
        # Vertical hacia arriba
        self.t_target_up = base + math.pi / 2.0
        # Barra hacia la izquierda (90° CCW desde arriba)
        self.t_target_left = self.t_target_up + math.pi / 2.0
        # Barra hacia la derecha (180° desde la izquierda)
        self.t_target_right = self.t_target_left + math.pi

        self.state = "t_orient1"

    # ----------------------------- LETRA S ----------------------------
    def start_letter_s(self):
        """Inicia el dibujo de la letra S."""
        if self.state is not None:
            self.get_logger().warn("Ya estoy en un movimiento. Espera.")
            return

        self.get_logger().info("→ Iniciando letra S.")
        # Empezamos por un tramo recto inicial
        self.state = "s_step0"
        self.end_time = time.monotonic() + self.T_s_straight

    # ----------------------------- TIMER ------------------------------
    def move_turtle(self):
        """
        Esta función se ejecuta periódicamente (cada dt) y
        actualiza la velocidad de la tortuga según el estado actual.
        """
        now = time.monotonic()

        if self.state is None:
            # No hay movimiento activo
            return

        # ==================== CÍRCULO (O) ====================
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
            # Orientar hacia abajo
            self._rotate_and_schedule(
                target=self.target_down,
                next_state="j_step1",
                next_duration=self.j_straight_time,
                log_msg="→ Trazo vertical de la J."
            )
            return

        if self.state == "j_step1":
            # Tramo vertical de la J
            if now >= self.end_time:
                # Pasamos al gancho (semicírculo)
                self.state = "j_step2"
                self.end_time = time.monotonic() + (math.pi / abs(self.omega))
                self.get_logger().info("→ Curva inferior de la J.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                self.publisher_.publish(msg)
            return

        if self.state == "j_step2":
            # Gancho inferior (semicírculo)
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
            # Orientar hacia abajo
            self._rotate_and_schedule(
                target=self.target_down,
                next_state="l_step1",
                next_duration=self.l_vertical_time,
                log_msg="→ Trazo vertical de la L."
            )
            return

        if self.state == "l_step1":
            # Tramo vertical de la L
            if now >= self.end_time:
                self.state = "l_orient2"
                self.get_logger().info("→ Giro para tramo horizontal de la L.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                self.publisher_.publish(msg)
            return

        if self.state == "l_orient2":
            # Orientar hacia la derecha
            self._rotate_and_schedule(
                target=self.target_right,
                next_state="l_step2",
                next_duration=self.l_horizontal_time,
                log_msg="→ Trazo horizontal de la L."
            )
            return

        if self.state == "l_step2":
            # Tramo horizontal de la L
            if now >= self.end_time:
                self.stop_movement("Letra L completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                self.publisher_.publish(msg)
            return

        # ===================== LETRA M =====================
        if self.state == "m_orient1":
            # Orientar primer tramo vertical hacia arriba
            self._rotate_and_schedule(
                target=self.m_target_up,
                next_state="m_step1",
                next_duration=self.m_vertical_time,
                log_msg="→ Primer tramo vertical de la M (hacia arriba)."
            )
            return

        if self.state == "m_step1":
            # Primer tramo vertical
            if now >= self.end_time:
                self.state = "m_orient2"
                self.get_logger().info("→ Giro para primer tramo diagonal de la M.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                self.publisher_.publish(msg)
            return

        if self.state == "m_orient2":
            # Orientar primer tramo diagonal
            self._rotate_and_schedule(
                target=self.m_target_diag1,
                next_state="m_step2",
                next_duration=self.m_diag_time,
                log_msg="→ Primer tramo diagonal de la M."
            )
            return

        if self.state == "m_step2":
            # Primer tramo diagonal
            if now >= self.end_time:
                self.state = "m_orient3"
                self.get_logger().info("→ Giro para segundo tramo diagonal de la M.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                self.publisher_.publish(msg)
            return

        if self.state == "m_orient3":
            # Orientar segundo tramo diagonal
            self._rotate_and_schedule(
                target=self.m_target_diag2,
                next_state="m_step3",
                next_duration=self.m_diag_time,
                log_msg="→ Segundo tramo diagonal de la M."
            )
            return

        if self.state == "m_step3":
            # Segundo tramo diagonal
            if now >= self.end_time:
                self.state = "m_orient4"
                self.get_logger().info("→ Giro para último tramo vertical de la M.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                self.publisher_.publish(msg)
            return

        if self.state == "m_orient4":
            # Orientar último tramo vertical hacia abajo
            self._rotate_and_schedule(
                target=self.m_target_down,
                next_state="m_step4",
                next_duration=self.m_vertical_time,
                log_msg="→ Último tramo vertical de la M (hacia abajo)."
            )
            return

        if self.state == "m_step4":
            # Último tramo vertical
            if now >= self.end_time:
                self.stop_movement("Letra M completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                self.publisher_.publish(msg)
            return

        # ===================== LETRA B =====================
        if self.state == "b_orient1":
            # Orientar espina vertical hacia arriba
            self._rotate_and_schedule(
                target=self.b_target_up,
                next_state="b_step1",
                next_duration=self.b_vertical_time,
                log_msg="→ Tramo vertical de la B (espina)."
            )
            return

        if self.state == "b_step1":
            # Espina vertical de la B
            if now >= self.end_time:
                self.state = "b_orient2"
                self.get_logger().info("→ Giro para primer lóbulo de la B.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8   # hacia arriba
                self.publisher_.publish(msg)
            return

        if self.state == "b_orient2":
            # Orientar hacia la derecha para el lóbulo superior
            self._rotate_and_schedule(
                target=self.b_target_right,
                next_state="b_arc1",
                next_duration=self.b_arc_time,
                log_msg="→ Primer semicírculo de la B (lóbulo superior)."
            )
            return

        if self.state == "b_arc1":
            # Primer semicírculo (lóbulo superior)
            if now >= self.end_time:
                self.state = "b_orient3"
                self.get_logger().info("→ Giro para tramo entre lóbulos.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.b_arc_v
                msg.angular.z = -self.b_arc_omega  # giro horario
                self.publisher_.publish(msg)
            return

        if self.state == "b_orient3":
            # Orientar hacia abajo (pequeño tramo entre lóbulos)
            self._rotate_and_schedule(
                target=self.b_target_down,
                next_state="b_step2",
                next_duration=self.b_mid_down_time,
                log_msg="→ Pequeño tramo entre lóbulos de la B."
            )
            return

        if self.state == "b_step2":
            # Pequeño tramo vertical entre lóbulos
            if now >= self.end_time:
                self.state = "b_orient4"
                self.get_logger().info("→ Giro para segundo lóbulo de la B.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8   # hacia abajo (muy poco tiempo)
                self.publisher_.publish(msg)
            return

        if self.state == "b_orient4":
            # Orientar de nuevo hacia la derecha para el lóbulo inferior
            self._rotate_and_schedule(
                target=self.b_target_right,
                next_state="b_arc2",
                next_duration=self.b_arc_time,
                log_msg="→ Segundo semicírculo de la B (lóbulo inferior)."
            )
            return

        if self.state == "b_arc2":
            # Segundo semicírculo (lóbulo inferior)
            if now >= self.end_time:
                self.stop_movement("Letra B completada.")
            else:
                msg = Twist()
                msg.linear.x = self.b_arc_v
                msg.angular.z = -self.b_arc_omega  # giro horario
                self.publisher_.publish(msg)
            return

        # ===================== LETRA A =====================
        if self.state == "a_orient1":
            # Orientar para la primera diagonal
            self._rotate_and_schedule(
                target=self.a_target_diag1,
                next_state="a_step1",
                next_duration=self.a_diag_time,
                log_msg="→ Primer tramo diagonal de la A."
            )
            return

        if self.state == "a_step1":
            # Primera diagonal
            if now >= self.end_time:
                self.state = "a_orient2"
                self.get_logger().info("→ Giro para segunda diagonal de la A.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                self.publisher_.publish(msg)
            return

        if self.state == "a_orient2":
            # Orientar para la segunda diagonal
            self._rotate_and_schedule(
                target=self.a_target_diag2,
                next_state="a_step2",
                next_duration=self.a_diag_time,
                log_msg="→ Segundo tramo diagonal de la A."
            )
            return

        if self.state == "a_step2":
            # Segunda diagonal
            if now >= self.end_time:
                self.state = "a_orient3"
                self.get_logger().info("→ Giro para devolver por la segunda diagonal.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                self.publisher_.publish(msg)
            return

        if self.state == "a_orient3":
            # Orientar en sentido contrario para volver hasta la mitad
            self._rotate_and_schedule(
                target=self.a_target_back,
                next_state="a_step3",
                next_duration=self.a_back_half_time,
                log_msg="→ Volviendo hasta la mitad de la segunda diagonal."
            )
            return

        if self.state == "a_step3":
            # Volver hasta la mitad de la diagonal
            if now >= self.end_time:
                self.state = "a_orient4"
                self.get_logger().info("→ Giro para la barra horizontal de la A.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                self.publisher_.publish(msg)
            return

        if self.state == "a_orient4":
            # Orientar para la barra horizontal central
            self._rotate_and_schedule(
                target=self.a_target_bar,
                next_state="a_step4",
                next_duration=self.a_bar_time,
                log_msg="→ Barra horizontal de la A."
            )
            return

        if self.state == "a_step4":
            # Barra horizontal
            if now >= self.end_time:
                self.stop_movement("Letra A completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8
                self.publisher_.publish(msg)
            return

        # ===================== LETRA T =====================
        if self.state == "t_orient1":
            # Orientar tramo vertical hacia arriba
            self._rotate_and_schedule(
                target=self.t_target_up,
                next_state="t_step1",
                next_duration=self.t_vertical_time,
                log_msg="→ Tramo vertical de la T."
            )
            return

        if self.state == "t_step1":
            # Tramo vertical
            if now >= self.end_time:
                self.state = "t_orient2"
                self.get_logger().info("→ Giro para barra hacia la izquierda de la T.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8   # hacia arriba
                self.publisher_.publish(msg)
            return

        if self.state == "t_orient2":
            # Orientar hacia la izquierda (barra)
            self._rotate_and_schedule(
                target=self.t_target_left,
                next_state="t_step2",
                next_duration=self.t_bar_quarter_time,
                log_msg="→ Barra horizontal hacia la izquierda (1/4)."
            )
            return

        if self.state == "t_step2":
            # Barra hacia la izquierda
            if now >= self.end_time:
                self.state = "t_orient3"
                self.get_logger().info("→ Giro para barra hacia la derecha (1/2).")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8   # hacia la izquierda
                self.publisher_.publish(msg)
            return

        if self.state == "t_orient3":
            # Orientar hacia la derecha (segunda parte de la barra)
            self._rotate_and_schedule(
                target=self.t_target_right,
                next_state="t_step3",
                next_duration=self.t_bar_half_time,
                log_msg="→ Barra horizontal hacia la derecha (1/2)."
            )
            return

        if self.state == "t_step3":
            # Barra hacia la derecha
            if now >= self.end_time:
                self.stop_movement("Letra T completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v * 0.8   # hacia la derecha
                self.publisher_.publish(msg)
            return

        # ===================== LETRA S =====================
        if self.state == "s_step0":
            # Tramo recto inicial de la S
            if now >= self.end_time:
                self.state = "s_step1"
                T_arc = math.pi / abs(self.omega_s)  # tiempo de un semicírculo
                self.end_time = time.monotonic() + T_arc
                self.get_logger().info("→ Primer semicírculo de la S.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v_s
                self.publisher_.publish(msg)
            return

        if self.state == "s_step1":
            # Primer semicírculo (parte superior de la S)
            if now >= self.end_time:
                self.state = "s_step2"
                T_straight_mid = (math.pi / abs(self.omega_s)) / 6.0  # tramo recto pequeño
                self.end_time = time.monotonic() + T_straight_mid
                self.get_logger().info("→ Tramo recto central de la S.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v_s
                msg.angular.z = self.omega_s
                self.publisher_.publish(msg)
            return

        if self.state == "s_step2":
            # Tramo recto central
            if now >= self.end_time:
                self.state = "s_step3"
                T_arc = math.pi / abs(self.omega_s)
                self.end_time = time.monotonic() + T_arc
                self.get_logger().info("→ Segundo semicírculo de la S.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v_s
                self.publisher_.publish(msg)
            return

        if self.state == "s_step3":
            # Segundo semicírculo (parte inferior de la S)
            if now >= self.end_time:
                self.state = "s_step4"
                self.end_time = time.monotonic() + self.T_s_straight
                self.get_logger().info("→ Tramo recto final de la S.")
                self._publish_stop()
            else:
                msg = Twist()
                msg.linear.x = self.v_s
                msg.angular.z = -self.omega_s
                self.publisher_.publish(msg)
            return

        if self.state == "s_step4":
            # Tramo recto final
            if now >= self.end_time:
                self.stop_movement("Letra S completada.")
            else:
                msg = Twist()
                msg.linear.x = self.v_s
                self.publisher_.publish(msg)
            return

    # ----------------------------- STOP -------------------------------
    def stop_movement(self, message):
        """Detiene la tortuga y resetea el estado."""
        self._publish_stop()
        self.state = None
        self.get_logger().info(message)

    def _publish_stop(self):
        """Publica un Twist nulo (velocidad cero)."""
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
