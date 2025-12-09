# Informe lab 3 grupo 1C

## Integrantes

- Juan Manuel Beltran Botello 
- Alejandro Mendivelso Torres
- Oscar Jhondairo Siabato Leon

Este repositorio contiene los entregables para la práctica de laboratorio con el robot PhantomX Pincher.

## 1. Descripción de la Solución
La solución implementada consiste en un script de Python que integra ROS 2 (Humble) con una interfaz gráfica desarrollada en `tkinter`. El sistema permite el control del brazo robótico PhantomX Pincher tanto en el espacio articular (moviendo cada motor individualmente) como en el espacio cartesiano (controlando la posición y orientación del efector final).

### Características Principales:
- **Control Articular**: Sliders y campos de entrada para controlar los 5 servomotores Dynamixel.
- **Control Cartesiano**: Cinematica inversa numérica para mover el efector final a coordenadas (X, Y, Z) y orientación (Roll, Pitch, Yaw).
- **Visualización en Tiempo Real**: Gráficos con `matplotlib` mostrando la configuración del robot en 3 vistas (Superior, Lateral, Frontal) y visualización numérica de la pose.
- **Integración con RViz**: Botón para lanzar la visualización del modelo URDF en RViz sincronizado con el robot real.
- **Seguridad**: Botón de Parada de Emergencia que deshabilita el torque de los motores.

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





