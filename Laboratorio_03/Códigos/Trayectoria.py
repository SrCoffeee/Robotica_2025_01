from robodk.robolink import *    # API para comunicarte con RoboDK
from robodk.robomath import *    # Funciones matemáticas y transformación de poses
import math                      # Librería matemática de Python

#------------------------------------------------
# 1) Conexión a RoboDK e inicialización
#------------------------------------------------
RDK = Robolink()

# Seleccionar un robot de la estación
robot = RDK.ItemUserPick("Selecciona un robot", ITEM_TYPE_ROBOT)

#------------------------------------------------
# 2) Selección del Frame (sistema de coordenadas)
#------------------------------------------------
frame_name = "Frame_from_Target1"
frame = RDK.Item(frame_name, ITEM_TYPE_FRAME)
if not frame.Valid():
    raise Exception(f'No se encontró el Frame "{frame_name}" en la estación.')

# Asignamos el frame seleccionado al robot
robot.setPoseFrame(frame)
# Usamos la herramienta activa actual
robot.setPoseTool(robot.PoseTool())

# Ajuste de velocidad y suavizado de trayectorias
robot.setSpeed(300)   # mm/s
robot.setRounding(5)  # radio de blending en mm

#------------------------------------------------
# 3) Parámetros de la espiral
#------------------------------------------------
num_points = 1000      # Más puntos = espiral más suave
a = 0                  # radio inicial (comienza en el centro)
b = 3                 # controla el espaciado entre vueltas
num_vueltas = 4
max_theta = 2 * math.pi * num_vueltas  # número total de vueltas (3 vueltas completas)
z_surface = 0          # plano de dibujo
z_safe = 50            # altura segura para entrada/salida 

#------------------------------------------------
# 4) Movimiento inicial al centro (altura segura)
#------------------------------------------------

#valorz=500

for e in range(100, 500,200):
    z_surface = 0          # plano de dibujo
    z_safe = 50   
    robot.MoveJ(transl(400, e, z_surface + z_safe))     # Centro pero arriba
    robot.MoveL(transl(400, e, z_surface))              # Baja al plano

    #------------------------------------------------
    # 5) Dibujar espiral: r = a + b*theta
    #------------------------------------------------
    for i in range(num_points + 1):
        # Ángulo actual
        theta = max_theta * (i / num_points)

        # Radio actual
        r = a + b * theta

        # Coordenadas cartesianas
        x = r * math.cos(theta)
        y = r * math.sin(theta)

        # Movimiento lineal al punto (x, y) sobre el plano
        robot.MoveL(transl(x, y, z_surface))

    #------------------------------------------------
    # 6) Al finalizar, subimos para evitar colisión
    #------------------------------------------------
    robot.MoveL(transl(x, y, z_surface + z_safe))

# Mensaje final
print(f"¡Espiral completada exitosamente en el frame '{frame_name}'!")