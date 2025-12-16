# Exposición del Proyecto: Ejecución y Desarrollo (PhantomX Pincher)

Este documento detalla el desarrollo y ejecución de las dos partes principales del proyecto: Control Automático (Clasificación de Figuras) y Control por Teclado (Teleoperación Articular con Gripper Neumático).

---

## Parte 1: Control Automático (Clasificación)

Esta etapa implementa una solución autónoma donde el robot recibe un tipo de figura, planifica una trayectoria para recogerla de una zona de recolección y la deposita en la caneca correspondiente según su color/forma.

### 1. Ejecución del Desarrollo
La lógica se basa en una **Máquina de Estados Finitos (FSM)** robusta implementada en Python.
- **Inicio**: El sistema espera en reposo (`IDLE`) hasta recibir un mensaje en el tópico `/figure_type`.
- **Proceso**: Al detectar una figura (e.g., "cubo"), el nodo busca la caneca destino asociada (e.g., "caneca_roja") y desencadena una secuencia de pasos predefinidos.
- **Secuencia**:
  1.  **Home**: Movimiento a posición inicial segura.
  2.  **Pick**: Aproximación a la zona de recolección (`recoleccion`).
  3.  **Grasp**: Activación del gripper (cierre).
  4.  **Lift & Transport**: Elevación y movimiento a través de puntos de paso seguros (`safe_carry_1` a `safe_carry_4`) para evitar colisiones.
  5.  **Place**: Llegada a la coordenada de la caneca destino y apertura del gripper.
  6.  **Return**: Retorno inverso por los puntos seguros hasta Home.

La ejecución utiliza **trayectorias no bloqueantes** con timers para permitir un monitoreo constante y evitar congelar el nodo durante movimientos largos.

### 2. Archivos Usados
*   **`clasificador_node.py`**: Nodo principal. Contiene la lógica de la máquina de estados, el mapeo de `figura -> caneca` y la gestión de tiempos.
*   **`poses.yaml`**: "Base de datos" de coordenadas. Almacena posiciones cartesianas precisas (x, y, z, roll, pitch, yaw) para Home, Recolección y cada una de las Canecas.
*   **`phantomx_pincher.launch.py`**: Orquestador. Levanta el `clasificador_node`, pero fundamentalmente inicia el nodo `commander` (C++) y **MoveIt**, habilitando la planificación de trayectorias.
*   **`phantomx_pincher_arm.xacro`**: Define la geometría del robot necesaria para calcular colisiones y cinemática.

### 3. Conceptos de ROS 2 Usados
*   **Nodos**: `clasificador_node` (Python) actúa como el cerebro de alto nivel.
*   **Topics**:
    *   `/figure_type` (Suscripción): Entrada del sistema (String).
    *   `/pose_command` (Publicación): Mensaje personalizado `PoseCommand` que envía la pose deseada al nodo de control de bajo nivel.
*   **Actions**: `gripper_trajectory_controller/follow_joint_trajectory`. Se usa un **Action Client** para controlar el gripper, permitiendo confirmar cuándo el gripper ha terminado de abrirse o cerrarse completamente antes de mover el brazo.
*   **Parameters**: Uso de parámetros para cargar configuraciones o definir tiempos de espera (`TIME_MOVEMENT`, `TIME_GRIPPER`).

### 4. Cinemática y Geometría
Para esta parte, el sistema hace uso intensivo de la **Cinemática Inversa (IK)**.
*   **Cinemática Inversa**: El `clasificador_node` trabaja en el **Espacio Cartesiano** (coordenadas X, Y, Z + Orientación RPY). El robot, sin embargo, se mueve rotando articulaciones.
    *   El nodo envía una posición deseada (ej: `x=0.1, y=0.0, z=0.046` para recolección).
    *   Un solver de cinemática (provisto por **MoveIt/KDL**) calcula qué ángulos deben tener las 4 articulaciones del brazo para situar el efector final exactamente en esa posición y con esa orientación (gripper mirando hacia abajo).
*   **Jacobiano**: Aunque implícito para el usuario en este nivel, el solver utiliza la matriz Jacobiana para mapear las velocidades espaciales deseadas a velocidades articulares, asegurando movimientos suaves y evitando singularidades (puntos donde el robot pierde grados de libertad).

### 5. Valores Importantes para Toma de Decisiones
Los valores críticos que determinan el éxito de la clasificación están definidos en `poses.yaml`:
*   **Zona de Recolección**: `x: 0.100, z: 0.046`. Esta altura es crítica; muy alta y no agarra el objeto, muy baja y choca con la mesa.
*   **Home**: `z: 0.157`. Altura segura para moverse sin chocar obstáculos bajos.
*   **Orientación (Roll: 3.142)**: Corresponde a ~180 grados (Pi), forzando al gripper a mirar siempre verticalmente hacia abajo, esencial para agarrar objetos desde arriba.
*   **Apertura del Gripper**:
    *   **Abierto**: `1.4` rad (aprox 80°). Suficiente para rodear los objetos.
    *   **Cerrado**: `0.5` rad (aprox 28°). Presión suficiente para sostener sin aplastar o soltar.

---

## Parte 2: Control por Teclado (Teleoperación)

Esta etapa permite el control manual directo de cada articulación del robot y la integración de un efector final neumático (ventosa) controlado por hardware externo.

### 1. Ejecución del Desarrollo
Se desarrolló un nodo de teleoperación (`teleop_joint_node.py`) que captura las pulsaciones del teclado en tiempo real sin necesidad de presionar Enter (usando `tty` y `termios`).
*   **Control Articular**: A diferencia del automático, aquí el usuario controla directamente el espacio articular (**Joint Space**). Teclas específicas (W/S, A/D, etc.) incrementan o decrementan el ángulo de una articulación específica.
*   **Integración Hardware (Relé)**: Se implementó una comunicación Serial con un **Arduino** para controlar un relé. Esto permite encender/apagar una bomba de vacío (teclas O/P), simulando un "gripper neumático" real que no es parte estándar del paquete ROS.

### 2. Archivos Usados
*   **`teleop_joint_node.py`**: Nodo principal. Maneja la captura de teclado, el mapeo de teclas a joints, y la comunicación serial con Arduino.
*   **`phantomx_pincher_arm.xacro`**: Define los límites físicos de cada articulación (`lower_limit`, `upper_limit`) que el movimiento debe respetar.
*   **`gripper_neumatico.stl`**: Archivo de malla 3D integrado visualmente para representar la ventosa en la simulación/visualización.

### 3. Conceptos de ROS 2 Usados
*   **Action Clients**: `joint_trajectory_controller/follow_joint_trajectory`. En lugar de publicar velocidades, el nodo construye una trayectoria dinámica de un solo punto con una duración muy corta, enviando la nueva posición articular deseada continuamente. Esto garantiza movimientos fluidos.
*   **Joint States**: Suscripción a `/joint_states` al inicio para sincronizar la posición interna del nodo con la posición real del robot, evitando "saltos" bruscos al conectar.
*   **Interacción Serial**: Uso de la librería `pyserial` dentro de un nodo ROS para interactuar con hardware no-ROS (Arduino), puenteando el mundo ROS con actuadores externos simples.

### 4. Cinemática
*   **Cinemática Directa (Forward Kinematics)**: En este modo, el usuario actúa como el "solver" inverso. El usuario decide los ángulos (entradas) y el robot (a través de `robot_state_publisher` y el URDF) calcula dónde termina el efector final en el espacio mediante Cinemática Directa.
    *   Cada vez que cambiamos un ángulo $\theta_i$, las matrices de transformación homogénea se multiplican para actualizar la posición visual del robot en RViz.
*   **Espacio de Trabajo**: Al controlar joints individualmente, es responsabilidad del operador mantener el robot dentro de su espacio de trabajo válido. Sin embargo, los límites definidos en el URDF (ficheros `.xacro`) actúan como barrera de seguridad.

### 5. Valores Importantes para Toma de Decisiones
*   **`JOINT_STEP = 0.008` rad**: Este valor define la resolución de movimiento ("step") por cada pulsación de tecla.
    *   Es un compromiso entre precisión (pasos finos para posicionamiento exacto) y velocidad (pasos grandes para moverse rápido). 0.008 rad (~0.45 grados) ofrece un control fino.
*   **Límites de Joints (del Xacro)**:
    *   Joint 1 (Base): `±150°` (2.61 rad).
    *   Joint 2 (Hombro): `±120°` (2.09 rad).
    *   Joint 3 (Codo): `±139°` (2.42 rad).
    *   Joint 4 (Muñeca): `-98°` a `+103°`.
*   **Posiciones de Home**:
    *   `HOME`: `[0, 0, 0, 0]` (Robot extendido horizontalmente).
    *   `HOME2`: `[0, 0, 1.57, 1.57]` (Robot en configuración L vertical, segura para reposo).
