# Guía de Uso: Clasificación de Objetos

Esta guía explica paso a paso cómo ejecutar el sistema de clasificación de objetos con el robot PhantomX Pincher.

## Requisitos Previos
Asegúrate de haber compilado el workspace:
```bash
cd ~/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws
colcon build --packages-select clasificacion_objetos
. install/setup.bash
```

## Paso 1: Lanzar Todo el Sistema (Robot + Clasificador)
En una **Terminal 1**, ejecuta el siguiente comando que iniciará tanto el robot (RViz/MoveIt) como el nodo clasificador automáticamente.

```bash
cd ~/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws
. install/setup.bash
ros2 launch clasificacion_objetos clasificador.launch.py
```
*Espera hasta ver que RViz se abra y en la terminal aparezca: `Clasificador Node Started`.*

## Paso 2: Ejecutar una Clasificación (Manual)
En una **Terminal 2**, tú actuarás como el "detector" enviando manualmente el comando.

**Comandos para mover el robot:**

🔴 **Clasificar Cubo (Rojo):**
```bash
ros2 topic pub --once /tipo_figura std_msgs/msg/String "data: 'cubo'"
```

🟢 **Clasificar Cilindro (Verde):**
```bash
ros2 topic pub --once /tipo_figura std_msgs/msg/String "data: 'cilindro'"
```

🔵 **Clasificar Pentágono (Azul):**
```bash
ros2 topic pub --once /tipo_figura std_msgs/msg/String "data: 'pentagono'"
```

🟡 **Clasificar Rectángulo (Amarillo):**
```bash
ros2 topic pub --once /tipo_figura std_msgs/msg/String "data: 'rectangulo'"
```

## Configuración de Posiciones
Si necesitas ajustar las coordenadas del robot, edita el archivo:
`src/clasificacion_objetos/config/positions.yaml`

Después de editarlo, **no necesitas recompilar**, solo reinicia el nodo clasificador (Paso 2).
