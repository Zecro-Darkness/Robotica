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
    A[Inicio del programa] --> B[Inicializar rclpy]
    B --> C[Crear nodo PincherController]
    C --> D{Hardware conectado?}

    D -- Si --> E[Inicializar motores]
    D -- No --> F[Modo simulacion]

    C --> G[Crear GUI PincherGUI]
    G --> H[Configurar pestañas y sliders]

    H --> I[Timers ROS2]
    I --> J[Publicar joint_states 10Hz]
    I --> K[Leer motores 20Hz]

    G --> L[Eventos GUI]
    L --> M[Usuario mueve sliders]
    M --> N[Ejecutar move_motor]

    G --> O[Control cartesiano]
    O --> P[Cinematica inversa -> nuevas q]

    C --> Q{Parada de emergencia?}
    Q -- Si --> R[Deshabilitar torque]

```
### Diagrama 2
``` mermaid
flowchart TD

    A[Inicio] --> B[Inicializar rclpy]
    B --> C[Crear PincherController]
    C --> D[Verificar conexión con motores]

    D -->|Conectado| E[Inicializar motores]
    D -->|Sin hardware| F[Modo simulación]

    A --> G[Crear GUI PincherGUI]
    G --> H[Cargar pestañas, sliders y controles]

    subgraph ROS2
        I[Timer 1: publicar joint_states]
        J[Timer 2: leer posiciones de motores]
    end

    subgraph GUI
        K[Eventos de teclado y sliders]
        L[Control cartesiano]
    end

    K --> M[Enviar comando al controlador]
    L --> M

    M --> N[Controlador recibe comandos]

    N --> O[Controlador calcula velocidades y posiciones]
    O --> P[Mover motores]
    P --> J

    J --> I
    I --> G

    N --> Q{Parada de emergencia?}
    Q -->|Si| R[Deshabilitar torque]

```
### Diagrama 3
``` mermaid
sequenceDiagram
    participant U as Usuario
    participant GUI as PincherGUI (Tkinter)
    participant C as PincherController (ROS2)
    participant DXL as Motores Dynamixel
    participant ROS as Publicación JointState

    U ->> GUI: Mover slider / Teclas
    GUI ->> C: Enviar comando de movimiento
    C ->> C: Calcular nueva posición objetivo
    C ->> DXL: Enviar posición / velocidad

    Note over DXL: Motor se mueve físicamente
    DXL -->> C: Reporte de posición actual

    C ->> ROS: Publicar joint_states
    ROS -->> GUI: Actualizar visualización

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






