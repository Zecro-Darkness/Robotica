# Informe lab 3 grupo 1C

## Integrantes

- Juan Manuel Beltran Botello 
- Alejandro Mendivelso Torres
- Oscar Jhondairo Siabato Leon

## Descripción detallada de la solución planteada.
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
## Código del script utilizado para el desarrollo de la práctica.
## Videos

### Video del brazo
### Video interfaz de usuario

## Gráfica digital de las poses comparádola con la fotografáa del brazo real en la misma configuración.

