# Informe lab 3 grupo 1C

## Integrantes

- Juan Manuel Beltran Botello 
- Alejandro Mendivelso Torres
- Oscar Jhondairo Siabato Leon

Este repositorio contiene los entregables para la práctica de laboratorio con el robot PhantomX Pincher.

## 1. Descripción de la Solución
La solución desarrollada propone un sistema de control robusto y modular para el brazo robótico **PhantomX Pincher**, integrando tecnologías de robótica moderna sobre el middleware **ROS 2 (Robot Operating System, distribución Humble)**. El proyecto se centra en la implementación de un nodo controlador inteligente capaz de gestionar la cinemática del manipulador y ofrecer una experiencia de usuario fluida a través de una Interfaz Gráfica (GUI) avanzada.

### 1.1 Arquitectura de Software y Hardware
El sistema opera bajo una arquitectura de **Nodos Concurrentes**, donde el script principal `control_servo.py` instancia una clase heredada de `rclpy.node.Node`.

- **Capa de Abstracción de Hardware (HAL)**:
  - Se utiliza la librería `dynamixel_sdk` para la comunicación serial directa con los servos Dynamixel (Protocolos 1.0/2.0 según el modelo).
  - El sistema implementa **lectura asíncrona** de la posición de los motores mediante un `timer` dedicado a 20Hz, garantizando que el estado del software siempre refleje la realidad física del robot.
  - Se gestiona la conversión de unidades entre *raw values* (0-1023/4095) y *radianes* del sistema internacional.

- **Integración con Ecosistema ROS 2**:
  - El nodo publica constantemente en el tópico `/joint_states` (mensaje `sensor_msgs/JointState`). Esto permite la total interoperabilidad con herramientas estándar de ROS como **RViz** (visualización 3D), **Robot State Publisher** (transformaciones TF) y paquetes de planificación como **MoveIt 2**.
  - La arquitectura *multithreading* permite que el bucle de eventos de ROS (`rclpy.spin`) y el bucle principal de la interfaz gráfica (`tkinter.mainloop`) se ejecuten en paralelo sin bloquearse mutuamente.

### 1.2 Modelo Cinemático y Matemático
El control del robot se fundamenta en un modelo matemático riguroso:

- **Cinemática Directa (FK)**:
  - Implementación manual de matrices de transformación homogénea basada en los parámetros **Denavit-Hartenberg (DH)** del PhantomX Pincher.
  - Permite calcular la posición `(x, y, z)` y orientación `(roll, pitch, yaw)` del Efector Final (TCP) en tiempo real para propósitos de visualización y odometría.

- **Cinemática Inversa Numérica (IK)**:
  - Se desarrolló un **solver iterativo basado en el Jacobiano**. A diferencia de las soluciones analíticas cerradas, este método numérico es más flexible y permite incorporar restricciones.
  - **Algoritmo**: Utiliza el método de Newton-Raphson con la pseudo-inversa del Jacobiano (`J^+`) para calcular las velocidades articulares necesarias que minimicen el error cartesiano `e = x_deseado - x_actual`.
  - **Manejo de Límites**: El algoritmo integra una verificación activa de los límites articulares ("Joint Limits") en cada iteración, asegurando que las soluciones generadas sean físicamente alcanzables y seguras para el robot.

### 1.3 Interfaz Hombre-Máquina (HMI) Avanzada
La interfaz gráfica no es solo un panel de botones, sino una herramienta de ingeniería completa:

- **Visualización Geométrica 3D**: Utilizando `matplotlib` integrado en `tkinter`, se generan tres proyecciones ortogonales (Planta, Perfil, Alzado) que se actualizan dinámicamente. Esto permite al operador verificar la pose del robot y prevenir colisiones antes de ejecutar movimientos.
- **Odometría Visual**: Muestra numéricamente la pose Cartesiana del TCP y los valores articulares con alta precisión decimal.
- **Control Híbrido**:
    - **Modo Articular**: Control directo de cada grado de libertad (DoF) mediante sliders de precisión.
    - **Modo Cartesiano**: Control intuitivo en el espacio de tarea. El usuario define *dónde* quiere el robot, y el algoritmo IK resuelve *cómo* llegar ahí.

### 1.4 Protocolos de Seguridad
- **Parada de Emergencia**: Interruptor de software prioritario bloquear el envío de torque a los motores (`Torque Enable = 0`), permitiendo detener el robot instantáneamente ante situaciones de riesgo.
- **Validación de Entradas**: Todos los campos de texto y sliders poseen validación de rangos para evitar comandos fuera de los límites operativos del hardware.

## Diagrama de flujo de acciones del robot utilizando la herramienta Mermaid.

### Diagrama 1
``` mermaid
flowchart TD

A[Start program] --> B[main and rclpy.init]
B --> C[Create PincherController]

C --> D{Port and baudrate OK}
D -->|Yes| E[Hardware connected]
D -->|No| G[Simulation mode]

E --> F[Initialize motors read positions]
G --> H[Skip hardware init]

E --> I[Create publisher and timers]
G --> I

I --> J[Create ROS spin thread]
J --> K[Create GUI and start mainloop]

L1[Timer read_joint_positions] --> L2{Hardware and no emergency}
L2 -->|Yes| L3[Read motor positions]
L3 --> L4[Convert to radians and apply sign]
L4 --> L5[Update current_joint_positions]
L2 -->|No| L6[Do not read hardware]

M1[Timer publish_joint_states] --> M2[Build JointState message]
M2 --> M3[Publish on joint_states]

U1[User moves joint slider] --> S1[on_slider_change deg to rad to ticks]
S1 --> CALL_MOVE

U2[User enters joint angles] --> S2[move_from_entry or move_to_joint_angles]
S2 --> CALL_MOVE

U3[User changes Cartesian sliders] --> C1[on_cartesian_change read XYZ]
C1 --> C2[Get current q]
C2 --> C3[Inverse kinematics]
C3 --> C4{IK converged and limits OK}
C4 -->|Yes| C5[Convert q solution to ticks]
C5 --> CALL_MOVE
C4 -->|No| C6[Update IK status only]

U4[User presses HOME] --> H1[home_all_motors default goal]
H1 --> CALL_MOVE

U5[User presses EMERGENCY STOP] --> E1[Set emergency flag true]
E1 --> E2[Disable motor torque]

CALL_MOVE[move_motor] --> N1{Emergency active}
N1 -->|Yes| N2[Ignore command]
N1 -->|No| N3[Compute target angle from ticks]

N3 --> N4{Change > threshold}
N4 -->|Yes| N5[Update previous_joint_positions]
N4 -->|No| N6[Keep previous]

N5 --> N7[Update target_joint_positions]
N6 --> N7

N7 --> N8{Hardware connected}
N8 -->|Yes| N9[Write GOAL_POSITION to motor]
N8 -->|No| N10[Update current_joint_positions in simulation]

K --> Z[User closes window]
Z --> Z1[on_close destroy GUI and shutdown rclpy]

```
## Plano de planta de la ubicación de cada uno de los elementos.
## Descripción de las funciones utilizadas.
A continuación se describen las funciones y métodos más relevantes del sistema de control para el robot PhantomX Pincher, implementado en el script control_servo.py.

## 1. Controlador del Robot (PincherController)

Este componente gestiona la lógica de bajo nivel, la comunicación con los motores y la seguridad.

| Función | Descripción |
| :--- | :--- |
| **move_motor(motor_id, position)** | **Actuación**. Envía el comando de posición a una articulación específica. Es la función base para cualquier movimiento del robot. |
| **read_joint_positions()** | **Retroalimentación**. Lee la posición real de los motores desde el hardware (Dynamixel) para mantener el estado del sistema sincronizado. |
| **initialize_motors()** | **Configuración**. Establece los parámetros iniciales de torque y velocidad, y sincroniza la posición lógica con la física al arrancar. |
| **emergency_stop()** | **Seguridad**. Deshabilita inmediatamente el torque de todos los motores para detener el robot ante cualquier contingencia. |

## 2. Interfaz y Matemáticas (PincherGUI)

Este componente maneja la interacción con el usuario y los cálculos cinemáticos.

| Función | Descripción |
| :--- | :--- |
| **on_cartesian_change()** | **Control TCP**. Se ejecuta al modificar los valores X, Y, Z. Orquestra el cálculo de la cinemática inversa para mover el robot a la coordenada deseada. |
| **inverse_kinematics(target, current)** | **Cálculo Matemático**. Algoritmo iterativo (Jacobiano) que determina qué ángulos articulares son necesarios para alcanzar una posición cartesiana específica. |
| **on_slider_change(motor_id)** | **Control Manual**. Callback para el movimiento individual de cada articulación mediante sliders, permitiendo un ajuste fino pose a pose. |
| **update_visualization()** | **Feedback Visual**. Renderiza en tiempo real las vistas Superior, Lateral y Frontal del robot, basándose en el modelo cinemático directo. |

## Código del script utilizado para el desarrollo de la práctica.
## Videos

### Video del brazo
### Video interfaz de usuario

## Gráfica digital de las poses comparádola con la fotografía del brazo real en la misma configuración.







