# Laboratorio 04 – Control de Turtlesim con ROS2 Humble
## Integrantes
Juan Manuel Beltran Botello
Alejandro Mendivelso Torres
Oscar Jhondairo Siabato Leon
## Descripción breve / Objetivos

Este proyecto consiste en controlar una tortuga dentro de ROS2 utilizando el teclado (sin hacer uso del paquete `turtle_teleop_key`). Además, se implementa la funcionalidad para dibujar las iniciales del nombre mediante teclas específicas. El objetivo principal es practicar el uso de ROS2, Linux y Python en el desarrollo de nodos, publishers y subscribers.

## Requisitos y entorno

- **ROS 2 Humble** instalado.
- **Turtlesim** instalado.
- **Workspace `my_turtle_controller`** creado.

## Procedimiento

### Paso 1

El primer paso para la realización de este laboratorio fue instalar Linux. En nuestro caso, optamos por configurar una máquina virtual utilizando **VMware Workstation Player**x, siguiendo el tutorial disponible en el siguiente repositorio: https://github.com/labsir-un/ROB_Intro_Linux

### Paso 2 

El siguiente paso consistió en la instalación de ROS2. Para ello, seguimos el tutorial proporcionado en el siguiente repositorio: https://github.com/labsir-un/ROB_Intro_ROS2_Humble. Este proceso incluyó la instalación de **Visual Studio Code**, **Terminator**, y la configuración del locale. Además, configuramos las fuentes de ROS2 Humble, realizamos la instalación de ROS2 Humble, y verificamos su funcionamiento ejecutando un ejemplo básico de talker-listener.


## Diagrama de flujo

### Codigo control manual
El codigo en mermaid es
```mermaid
flowchart TD
    A[Inicio] --> B[main]
    B --> C[Inicializar rclpy]
    C --> D[Crear nodo]
    D --> E[Configurar publicador Twist]
    E --> F[Iniciar hilo lector]

    F --> G{Tecla?}
    G -->|No| G
    G -->|Sí| H[Leer tecla]

    H --> I{Arriba?}
    I -->|Sí| J[Velocidad adelante]
    I -->|No| K{Abajo?}

    K -->|Sí| L[Velocidad atrás]
    K -->|No| M{Izquierda?}

    M -->|Sí| N[Giro izquierda]
    M -->|No| O{Derecha?}

    O -->|Sí| P[Giro derecha]
    O -->|No| Q{Space?}

    Q -->|Sí| R[Detener]
    Q -->|No| G

    J --> S[Publicar comando]
    L --> S
    N --> S
    P --> S
    R --> S
    S --> G

    G -->|Ctrl+C| T[Detener nodo]
    T --> U[Publicar stop final]
    U --> V[Destruir nodo]
    V --> W[Shutdown]
    W --> X[Fin]

```

### Codigo letras

El codigo en mermaid es
```mermaid
flowchart TD

    A[Inicio] --> B[main]
    B --> C[Inicializar rclpy]
    C --> D[Crear TurtleController]
    D --> E[Suscribirse a pose de la tortuga]
    E --> F[Configurar publicador a cmd_vel]
    F --> G[Esperar comandos de letras]

    G --> H{¿Letra T?}
    H -->|Sí| T1[Verificar si tortuga está lista]
    T1 --> T2[Calcular ángulos objetivo: arriba, izquierda, derecha]
    T2 --> T3[Ejecutar rotaciones]
    T3 --> T4[Ejecutar movimientos lineales]
    T4 --> R[Publicar stop]
    H -->|No| I1{¿Letra I?}

    I1 -->|Sí| I2[Verificar pose]
    I2 --> I3[Calcular trayecto recto]
    I3 --> I4[Ejecutar movimiento vertical]
    I4 --> R

    I1 -->|No| L1{¿Letra L?}

    L1 -->|Sí| L2[Verificar pose]
    L2 --> L3[Bajar verticalmente]
    L3 --> L4[Avanzar horizontalmente]
    L4 --> R

    L1 -->|No| G

    R --> G

```

