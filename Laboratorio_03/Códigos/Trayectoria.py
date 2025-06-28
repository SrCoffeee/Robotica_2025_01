from robodk.robolink import *    # API para comunicarte con RoboDK
from robodk.robomath import *    # Funciones matemáticas
import math

#------------------------------------------------
# 1) Conexión a RoboDK e inicialización
#------------------------------------------------
RDK = Robolink()

# Elegir un robot (si hay varios, aparece un popup)
robot = RDK.ItemUserPick("Selecciona un robot", ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception("No se ha seleccionado un robot válido.")

# Conectar al robot físico
#if not robot.Connect():
    #raise Exception("No se pudo conectar al robot. Verifica que esté en modo remoto y que la configuración sea correcta.")

# Confirmar conexión
#if not robot.ConnectedState():
   # raise Exception("El robot no está conectado correctamente. Revisa la conexión.")

#print("Robot conectado correctamente.")

# Configuración inicial del robot
frame_name = "Frame_from_Target1"
frame = RDK.Item(frame_name, ITEM_TYPE_FRAME)
if not frame.Valid():
    raise Exception(f'No se encontró el Frame "{frame_name}".')

robot.setPoseFrame(frame)
robot.setPoseTool(robot.PoseTool())
robot.setSpeed(300)
robot.setRounding(5)

# Parámetros globales
z_surface = 0
z_safe = 50
offset_z = 0
num_points = 100

def draw_circle(radio, offset_x, offset_y, repetitions=1):
    """Dibuja un círculo con los parámetros especificados"""
    for _ in range(repetitions):
        robot.MoveJ(transl(0, 0, z_surface - z_safe))
        
        for i in range(num_points + 1):
            angulo = 2 * math.pi * i / num_points
            x = radio * math.cos(angulo) + offset_x
            y = radio * math.sin(angulo) + offset_y
            target = transl(x, y, z_surface + offset_z)
            
            if i == 0:
                robot.MoveL(target)
            else:
                robot.MoveJ(target)

def draw_spiral(offset_x, offset_y, invert=False):
    """Dibuja una espiral con parámetros específicos"""
    a, b = 0, 3
    max_theta = 6 * math.pi
    
    robot.MoveJ(transl(0, 0, z_surface - z_safe))
    
    for i in range(num_points + 1):
        theta = max_theta * (i / num_points)
        r = a + b * theta
        
        multiplier = -1 if invert else 1
        x = multiplier * r * math.cos(theta) + offset_x
        y = multiplier * r * math.sin(theta) + offset_y
        
        robot.MoveL(transl(x, y, z_surface))
    
    robot.MoveL(transl(x, y, z_surface - z_safe))
    robot.MoveJ(transl(0, 0, z_surface - z_safe))

def draw_cissoid():
    """Dibuja una curva cisoide"""
    a = 100
    t_min, t_max = -1.2, 1.2
    offset_x = 0
    
    for _ in range(3):
        robot.MoveJ(transl(0, 0, z_surface - z_safe))
        
        for i in range(num_points + 1):
            t = t_min + (t_max - t_min) * i / num_points
            denom = 1 + t**2
            
            x = 2 * a * (t**2 / denom) * 0.3 + offset_x
            y = 2 * a * (t**3 / denom)
            offset_x -= 0.1
            
            target = transl(x, y, z_surface + offset_z)
            
            if i == 0:
                robot.MoveL(target)
            
            robot.MoveL(target)

def draw_lemniscate(a_inf, offset_x, offset_y):
    """Dibuja un símbolo de infinito (lemniscata)"""
    robot.MoveJ(transl(0, 0, z_surface - z_safe))
    
    for i in range(num_points + 1):
        t = 2 * math.pi * i / num_points
        denom = 1 + (math.sin(t))**2
        
        x = a_inf * math.sin(t) * math.cos(t) / denom + offset_x
        y = a_inf * math.cos(t) / denom + offset_y
        
        target = transl(x, y, z_surface + offset_z)
        
        if i == 0:
            robot.MoveL(target)
        else:
            robot.MoveL(target)

# Secuencia de dibujo
print("Iniciando secuencia de dibujo...")

# 1. Círculo principal
draw_circle(radio=100, offset_x=0, offset_y=0, repetitions=1)

# 2. Espiral ojo derecho
draw_spiral(offset_x=100, offset_y=100, invert=False)

# 3. Espiral ojo izquierdo
draw_spiral(offset_x=100, offset_y=-100, invert=True)

# 4. Curva cisoide
draw_cissoid()

# 5. Símbolo de infinito
draw_lemniscate(a_inf=50, offset_x=-100, offset_y=0)

# 6. Círculos adicionales
draw_circle(radio=35, offset_x=-70, offset_y=-110, repetitions=1)
draw_circle(radio=35, offset_x=-70, offset_y=110, repetitions=1)

# Movimiento final
robot.MoveJ(transl(0, 0, z_surface - z_safe))

# Cambio de frame final
frame_name = "Frame 3"
frame = RDK.Item(frame_name, ITEM_TYPE_FRAME)
if not frame.Valid():
    raise Exception(f'No se encontró el Frame "{frame_name}".')

robot.setPoseFrame(frame)
robot.setPoseTool(robot.PoseTool())
robot.MoveJ(transl(0, 0, 0))

print("¡Secuencia de dibujo completada!")













