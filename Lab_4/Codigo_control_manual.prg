#!/usr/bin/env python3
"""
Nodo ROS2 para controlar la tortuga de turtlesim usando el teclado.

Controles:
  Flecha ↑ : avanzar
  Flecha ↓ : retroceder
  Flecha ← : girar a la izquierda
  Flecha → : girar a la derecha
  SPACE    : detener
"""

import sys
import threading
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleKeyboard(Node):
    """Nodo que publica comandos de velocidad a /turtle1/cmd_vel según teclas presionadas."""

    def __init__(self):
        super().__init__('turtle_keyboard')

        # Publicador de velocidades
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Velocidades base
        self.linear_speed = 1.5   # [m/s]
        self.angular_speed = 2.0  # [rad/s]

        # Comando de velocidad actual (se envía periódicamente)
        self.current_cmd = Twist()

        # Timer para publicar el comando actual cada dt segundos
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.publish_cmd)

        # Guardar configuración original del terminal
        self.settings = termios.tcgetattr(sys.stdin)

        # Hilo dedicado a leer el teclado (input es bloqueante)
        self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.key_thread.start()

        self.get_logger().info(
            "Teleoperación lista.\n"
            "Flechas: mover la tortuga | SPACE: detener | Ctrl+C: salir."
        )

    # ------------------------------------------------------------------
    # PUBLICACIÓN DEL COMANDO
    # ------------------------------------------------------------------
    def publish_cmd(self):
        """Publica en /turtle1/cmd_vel el comando de velocidad actual."""
        self.cmd_pub.publish(self.current_cmd)

    # ------------------------------------------------------------------
    # LECTURA DEL TECLADO
    # ------------------------------------------------------------------
    def keyboard_loop(self):
        """
        Lee teclas directamente desde stdin en modo 'cbreak'.

        Las flechas llegan como secuencias:
          ESC [ A → ↑
          ESC [ B → ↓
          ESC [ C → →
          ESC [ D → ←
        """
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                ch = sys.stdin.read(1)

                # Flechas (secuencia ESC [ X)
                if ch == '\x1b':
                    ch2 = sys.stdin.read(1)
                    ch3 = sys.stdin.read(1)
                    if ch2 == '[':
                        if ch3 == 'A':      # Flecha arriba
                            self.move_forward()
                        elif ch3 == 'B':    # Flecha abajo
                            self.move_backward()
                        elif ch3 == 'C':    # Flecha derecha
                            self.turn_right()
                        elif ch3 == 'D':    # Flecha izquierda
                            self.turn_left()

                # Barra espaciadora → detener
                elif ch == ' ':
                    self.stop()

                time.sleep(0.01)

        finally:
            # Restaurar configuración del terminal al salir
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    # ------------------------------------------------------------------
    # ACCIONES DE MOVIMIENTO
    # ------------------------------------------------------------------
    def move_forward(self):
        """Avanzar hacia adelante."""
        self.current_cmd.linear.x = self.linear_speed
        self.current_cmd.angular.z = 0.0

    def move_backward(self):
        """Retroceder."""
        self.current_cmd.linear.x = -self.linear_speed
        self.current_cmd.angular.z = 0.0

    def turn_left(self):
        """Girar a la izquierda sobre su eje."""
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = self.angular_speed

    def turn_right(self):
        """Girar a la derecha sobre su eje."""
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = -self.angular_speed

    def stop(self):
        """Detener la tortuga (velocidades a cero)."""
        self.current_cmd = Twist()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleKeyboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.publish_cmd()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
