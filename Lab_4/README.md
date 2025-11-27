# Laboratorio 04 – Control de Turtlesim con ROS2 Humble
## Integrantes

- Juan Manuel Beltran Botello 
- Alejandro Mendivelso Torres
- Oscar Jhondairo Siabato Leon

  
## Descripción breve / Objetivos

Este proyecto consiste en controlar una tortuga dentro de ROS2 utilizando el teclado (sin hacer uso del paquete `turtle_teleop_key`). Además, se implementa la funcionalidad para dibujar las iniciales del nombre mediante teclas específicas. El objetivo principal es practicar el uso de ROS2, Linux y Python en el desarrollo de nodos, publishers y subscribers.

## Objetivo del Laboratorio

Este laboratorio tiene como objetivo proporcionar un aprendizaje práctico de tres elementos fundamentales:

1. **Tutoriales de Linux**: Comandos esenciales de consola para navegación, permisos, procesos y manejo de archivos.
2. **Tutoriales de ROS 2 Humble**: Estructura de nodos, tópicos, mensajes, y paquetes dentro de un `workspace`.
3. **Tutoriales de Turtlesim**: Uso del simulador y del tópico `/turtle1/cmd_vel` para controlar la velocidad lineal y angular.

## Resultados de Aprendizaje

- Comprender los conceptos básicos de ROS.
- Usar comandos fundamentales de Linux.
- Conectar nodos en Python dentro de ROS 2.
- Manipular la tortuga mediante código propio.

## Procedimiento

### Paso 1

El primer paso para la realización de este laboratorio fue instalar Linux. En nuestro caso, optamos por configurar una máquina virtual utilizando **VMware Workstation Player**x, siguiendo el tutorial disponible en el siguiente repositorio: https://github.com/labsir-un/ROB_Intro_Linux

### Paso 2 

1. **Configuración del entorno de desarrollo**:
   - Se configuró el sistema operativo Linux, incluyendo la instalación de los paquetes necesarios para el trabajo con ROS 2 Humble.
   - Se estableció un `workspace` en ROS 2 para gestionar los proyectos y nodos correspondientes.
   - Se instaló y configuró el simulador Turtlesim, asegurándose de que se pudieran controlar los movimientos de la tortuga mediante comandos en Python.

2. **Control de la tortuga con teclado**:
   - Se creó un script en Python (llamado `move_turtle.py`) para controlar el movimiento de la tortuga a través del teclado.
   - Se implementaron los controles para avanzar, retroceder, y girar a la izquierda y derecha utilizando las teclas de flecha.
   - Se optó por no usar el nodo `turtle_teleop_key` para garantizar que el control se hiciera de forma personalizada desde nuestro script.

3. **Dibujo automático de letras**:
   - Se creó un archivo `codigo_letras.prg` con las trayectorias específicas para cada letra (O, J, S, L, M, B, A, T).
   - Cada letra fue convertida a código ROS2, implementando una función independiente para cada una de ellas que movía la tortuga a lo largo de las trayectorias definidas.

4. **Integración y prueba de nodos**:
   - Se verificó la correcta comunicación entre los diferentes nodos utilizando los comandos `colcon build` y `ros2 run`.
   - Se ejecutaron pruebas de control de movimiento y dibujo para asegurar que todos los componentes funcionaran de manera integrada.

## Decisiones de Diseño

1. **Uso de ROS 2 Humble**:
   - Se eligió ROS 2 debido a su capacidad para manejar sistemas distribuidos y sus mejoras sobre ROS 1, incluyendo un mejor manejo de la latencia y mayor escalabilidad.
   - La decisión de trabajar con Python fue tomada por su facilidad de integración con ROS 2 y su sintaxis clara, lo cual facilita el desarrollo rápido de scripts.

2. **Control personalizado de la tortuga**:
   - Se optó por desarrollar el control de la tortuga desde cero sin usar el nodo `turtle_teleop_key` para asegurar un control más flexible y adaptado a nuestras necesidades. Esto permitió un mayor control sobre los movimientos y la incorporación de lógica personalizada para los movimientos de la tortuga.

3. **Estructura de las funciones de las letras**:
   - Cada letra fue implementada como una función independiente, lo que permitió modularizar el código y hacer que cada una fuera fácilmente reutilizable.
   - Esto facilita la extensión del proyecto para dibujar más letras o figuras en el futuro, simplemente añadiendo más funciones de dibujo.

4. **Uso de ROS 2 para integración**:
   - La elección de usar ROS 2 para la integración de nodos fue clave para garantizar la escalabilidad del proyecto, permitiendo que se pueda expandir a otros robots o sistemas en el futuro.
   - La comunicación entre nodos se facilitó utilizando mensajes ROS para controlar la tortuga y ejecutar los movimientos definidos.

## Funcionamiento General del Proyecto

El proyecto se basa en la interacción entre diferentes elementos: el sistema operativo Linux, el entorno ROS 2 Humble, y el simulador Turtlesim. El flujo general de funcionamiento es el siguiente:

1. **Inicialización**: Se construye el workspace y se configura el entorno ROS.
2. **Control de la tortuga**: Se ejecuta el script `move_turtle.py`, que escucha las teclas del teclado y envía comandos al nodo ROS correspondiente para controlar el movimiento de la tortuga.
3. **Dibujo de letras**: A medida que el script se ejecuta, se llaman las funciones correspondientes a las letras definidas en el archivo `codigo_letras.prg`, haciendo que la tortuga trace las letras sobre el espacio de trabajo.
4. **Verificación**: El sistema verifica que la comunicación entre nodos esté funcionando correctamente y asegura que los movimientos y dibujos de la tortuga se realicen con precisión.
   
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

