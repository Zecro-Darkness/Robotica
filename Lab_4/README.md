# Laboratorio 04 – Control de Turtlesim con ROS2 Humble
## Integrantes

- Juan Manuel Beltran Botello 
- Alejandro Mendivelso Torres
- Oscar Jhondairo Siabato Leon

  
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

%% ================================================
%% BLOQUE PRINCIPAL
%% ================================================

A[Inicio] --> B[main]
B --> C[Inicializar rclpy]
C --> D[Crear TurtleController]
D --> E[Suscribirse a pose]
E --> F[Configurar publicador cmd_vel]
F --> G[Esperar letra recibida]

G --> H{¿Qué letra llegó?}

%% ================================================
%% LETRA O (CÍRCULO)
%% ================================================

H -->|O| O1[¿Pose disponible?]
O1 -->|No| G
O1 -->|Sí| O2[Calcular trayectoria circular]
O2 --> O3[Rotar incrementalmente]
O3 --> O4[Avanzar en pequeñas líneas formando círculo]
O4 --> O5[Repetir hasta 360°]
O5 --> O6[STOP]
O6 --> G

%% ================================================
%% LETRA J
%% ================================================

H -->|J| J1[¿Pose disponible?]
J1 -->|No| G
J1 -->|Sí| J2[Rotar hacia arriba]
J2 --> J3[Avanzar trazo vertical]
J3 --> J4[Rotar derecha]
J4 --> J5[Avanzar base curva]
J5 --> J6[Rotar abajo]
J6 --> J7[Completar curva inferior]
J7 --> J8[STOP]
J8 --> G

%% ================================================
%% LETRA S
%% ================================================

H -->|S| S1[¿Pose disponible?]
S1 -->|No| G
S1 -->|Sí| S2[Rotar izquierda]
S2 --> S3[Avanzar trazo superior]
S3 --> S4[Rotar abajo]
S4 --> S5[Avanzar curva central]
S5 --> S6[Rotar derecha]
S6 --> S7[Avanzar trazo inferior]
S7 --> S8[STOP]
S8 --> G

%% ================================================
%% LETRA L
%% ================================================

H -->|L| L1[¿Pose disponible?]
L1 -->|No| G
L1 -->|Sí| L2[Rotar abajo]
L2 --> L3[Avanzar trazo vertical]
L3 --> L4[Rotar derecha]
L4 --> L5[Avanzar trazo base]
L5 --> L6[STOP]
L6 --> G

%% ================================================
%% LETRA M
%% ================================================

H -->|M| M1[¿Pose disponible?]
M1 -->|No| G
M1 -->|Sí| M2[Rotar arriba]
M2 --> M3[Avanzar pata izquierda]
M3 --> M4[Rotar diagonal derecha]
M4 --> M5[Avanzar pico central]
M5 --> M6[Rotar diagonal izquierda]
M6 --> M7[Avanzar segundo pico]
M7 --> M8[Rotar abajo]
M8 --> M9[Avanzar pata derecha]
M9 --> M10[STOP]
M10 --> G

%% ================================================
%% LETRA B
%% ================================================

H -->|B| B1[¿Pose disponible?]
B1 -->|No| G
B1 -->|Sí| B2[Rotar arriba]
B2 --> B3[Avanzar línea principal]
B3 --> B4[Rotar derecha]
B4 --> B5[Formar curva superior]
B5 --> B6[Rotar derecha]
B6 --> B7[Formar curva inferior]
B7 --> B8[STOP]
B8 --> G

%% ================================================
%% LETRA A
%% ================================================

H -->|A| A1[¿Pose disponible?]
A1 -->|No| G
A1 -->|Sí| A2[Rotar diagonal izquierda]
A2 --> A3[Avanzar pata izquierda]
A3 --> A4[Rotar diagonal derecha]
A4 --> A5[Avanzar pata derecha]
A5 --> A6[Rotar horizontal]
A6 --> A7[Avanzar línea del medio]
A7 --> A8[STOP]
A8 --> G

%% ================================================
%% LETRA T
%% ================================================

H -->|T| T1[¿Pose disponible?]
T1 -->|No| G
T1 -->|Sí| T2[Rotar arriba]
T2 --> T3[Avanzar trazo vertical]
T3 --> T4[Retroceder al centro]
T4 --> T5[Rotar izquierda]
T5 --> T6[Avanzar barra izquierda]
T6 --> T7[Rotar derecha]
T7 --> T8[Avanzar barra derecha]
T8 --> T9[STOP]



```

