from robodk import *
from robolink import *
from matplotlib.textpath import TextPath
import math
import numpy as np

# ----------------------------------------------------------
# 1) Conexión e inicialización
# ----------------------------------------------------------
RDK = Robolink()
robot = RDK.ItemUserPick("Selecciona un robot", ITEM_TYPE_ROBOT)

frame_name = "Frame_from_Target1"
frame = RDK.Item(frame_name, ITEM_TYPE_FRAME)
if not frame.Valid():
    raise Exception(f'No se encontró el Frame "{frame_name}".')

robot.setPoseFrame(frame)
robot.setPoseTool(robot.PoseTool())
robot.setSpeed(300)
robot.setRounding(5)

# ----------------------------------------------------------
# 2) Parámetros de la espiral
# ----------------------------------------------------------
z_surface = 0
z_safe = 50
a = 0
b = 15           # reducido para asegurar alcance
num_turns = 3  # menos vueltas para no alejarse mucho
num_points = 800

# ----------------------------------------------------------
# 3) Función para movimiento seguro
# ----------------------------------------------------------
def moveL_safe(x, y, z):
    """Mueve el robot a (x, y, z) en línea recta.
       Si el punto está fuera de alcance, sube a z_safe y continúa."""
    pose = transl(x, y, z)
    if not robot.Valid():
        return
    try:
        # Comprobar si el punto está dentro del espacio de trabajo
        if not robot.PoseIsWithinReach(pose):
            raise Exception("Fuera de alcance")
        robot.MoveL(pose)
    except:
        print(f"⚠️ Punto fuera de alcance: ({x:.1f}, {y:.1f}, {z:.1f})")
        robot.MoveJ(transl(x, y, z_safe))

# ----------------------------------------------------------
# 4) Dibujar espiral de Arquímedes
# ----------------------------------------------------------
robot.MoveJ(transl(0, 0, z_surface + z_safe))
robot.MoveL(transl(0, 0, z_surface))

theta_max = num_turns * 2 * math.pi
for i in range(num_points + 1):
    t = i / num_points
    theta = t * theta_max
    r = a + b * theta
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    moveL_safe(x, y, z_surface)

robot.MoveL(transl(x, y, z_surface + z_safe))

# ----------------------------------------------------------
# 5) Ir a Target 1
# ----------------------------------------------------------
target_name = "Target 1"
target = RDK.Item(target_name, ITEM_TYPE_TARGET)
if not target.Valid():
    raise Exception(f'No se encontró el Target "{target_name}".')

robot.MoveJ(target)
pose_target = target.Pose()
x_target, y_target, z_target = pose_target.Pos()

# ----------------------------------------------------------
# 6) Generar texto orientado en +Y sin reflejo
# ----------------------------------------------------------
texto = "ALEJO JUAN OSCAR"
tp = TextPath((0, 0), texto, size=60)

X, Y = tp.vertices[:, 0], tp.vertices[:, 1]

# Rotar 90° hacia +Y
angle = math.radians(90)
cos_a, sin_a = math.cos(angle), math.sin(angle)
X_rot = -(X * cos_a - Y * sin_a)  # corrección reflejo
Y_rot = X * sin_a + Y * cos_a

# Offset desde Target 1
x_offset = x_target
y_offset = y_target + 50  # desplaza texto hacia +Y
z_offset = z_target

# ----------------------------------------------------------
# 7) Dibujar texto con protección
# ----------------------------------------------------------
robot.MoveL(transl(x_offset, y_offset, z_offset + z_safe))
robot.MoveL(transl(x_offset, y_offset, z_offset))

for i in range(len(X_rot)):
    x = X_rot[i] + x_offset
    y = Y_rot[i] + y_offset
    moveL_safe(x, y, z_offset)

robot.MoveL(transl(x, y, z_offset + z_safe))

# ----------------------------------------------------------
# 8) Regresar a Home
# ----------------------------------------------------------
home = RDK.Item("Home", ITEM_TYPE_TARGET)
if not home.Valid():
    raise Exception("⚠️ No se encontró el Target 'Home'. Verifica que exista en RoboDK.")

robot.MoveJ(home)

print("✅ Espiral completada, texto 'ALEJO JUAN OSCAR' escrito en +Y y robot regresó a Home.")
