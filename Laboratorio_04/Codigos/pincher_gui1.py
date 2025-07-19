import customtkinter as ctk
from tkinter import messagebox
import rclpy
import threading
from pincher_control.control_servo import PincherController
from PIL import Image
import os
import time

# Configurar apariencia de CustomTkinter
ctk.set_appearance_mode("dark")  # Puedes cambiar a "light" o "system"
ctk.set_default_color_theme("blue")  # Opciones: "blue", "green", "dark-blue"

# Posiciones predeterminadas
preset_positions = {
    "Posición 1": [512, 512, 512, 512, 512],
    "Posición 2": [582, 582, 568, 454, 512],
    "Posición 3": [412, 610, 426, 596, 512],
    "Posición 4": [753, 454, 667, 582, 512],
    "Posición 5": [738, 412, 667, 383, 512]
}

# Iniciar ROS y el controlador
rclpy.init()
pincher = PincherController()

# Crear ventana principal
ventana = ctk.CTk()
ventana.title("Interfaz Pincher")
ventana.geometry("700x850")

# Frame principal con padding
main_frame = ctk.CTkFrame(ventana, corner_radius=0)
main_frame.pack(fill="both", expand=True, padx=20, pady=20)

# Logo - Intenta cargar la imagen si existe
try:
    # Ajusta la ruta según tu estructura
    logo_path = "phantom_ws/src/pincher_control/pincher_control/unal_logo.png"
    if os.path.exists(logo_path):
        logo_img = ctk.CTkImage(
            light_image=Image.open(logo_path),
            dark_image=Image.open(logo_path),
            size=(100, 100)
        )
        logo_label = ctk.CTkLabel(main_frame, image=logo_img, text="")
        logo_label.pack(pady=(10, 15))
except:
    # Si no se puede cargar el logo, simplemente continuar
    pass

# Encabezado
titulo = ctk.CTkLabel(
    main_frame, 
    text="Robótica 2025-1", 
    font=ctk.CTkFont(size=26, weight="bold")
)
titulo.pack(pady=(0, 5))

subtitulo = ctk.CTkLabel(
    main_frame, 
    text="Universidad Nacional de Colombia", 
    font=ctk.CTkFont(size=16)
)
subtitulo.pack(pady=(0, 8))

# Frame para autores con borde
autores_frame = ctk.CTkFrame(main_frame, corner_radius=10)
autores_frame.pack(pady=(0, 20))

autores = ctk.CTkLabel(
    autores_frame, 
    text="Carlos Fernando Quintero\nDavid Camilo Valbuena", 
    font=ctk.CTkFont(size=13)
)
autores.pack(padx=20, pady=8)

# Frame para mostrar posición actual
posicion_actual_frame = ctk.CTkFrame(main_frame, corner_radius=10, fg_color=("gray85", "gray25"))
posicion_actual_frame.pack(fill="x", pady=(0, 20))

titulo_posicion = ctk.CTkLabel(
    posicion_actual_frame,
    text="Posición Actual del Robot",
    font=ctk.CTkFont(size=16, weight="bold")
)
titulo_posicion.pack(pady=(10, 5))

# Frame para los valores de posición
valores_frame = ctk.CTkFrame(posicion_actual_frame, corner_radius=8, fg_color=("gray90", "gray20"))
valores_frame.pack(padx=15, pady=(5, 15), fill="x")

# Labels para cada servo
servo_labels = []
servo_valores = []
servo_nombres = ["Servo 1", "Servo 2", "Servo 3", "Servo 4", "Gripper"]

for i in range(5):
    # Frame individual para cada servo
    servo_frame = ctk.CTkFrame(valores_frame, corner_radius=5, fg_color="transparent")
    servo_frame.pack(side="left", expand=True, fill="x", padx=5, pady=10)
    
    # Nombre del servo
    nombre_label = ctk.CTkLabel(
        servo_frame,
        text=servo_nombres[i],
        font=ctk.CTkFont(size=12, weight="bold"),
        text_color=("gray40", "gray60")
    )
    nombre_label.pack()
    
    # Valor del servo
    valor_label = ctk.CTkLabel(
        servo_frame,
        text="---",
        font=ctk.CTkFont(size=20, weight="bold"),
        text_color=("blue", "lightblue")
    )
    valor_label.pack()
    servo_valores.append(valor_label)

# Variable para controlar el thread de actualización
actualizando = True

def actualizar_posicion():
    """Actualiza la posición mostrada en la interfaz"""
    while actualizando:
        try:
            # Aquí deberías obtener la posición real del robot
            # Por ahora, simulamos con valores aleatorios o puedes implementar
            # un método en PincherController para leer las posiciones actuales
            
            # Opción 1: Si tu PincherController tiene un método para leer posiciones
            # posiciones = pincher.leer_posiciones_actuales()
            
            # Opción 2: Simulación (reemplaza esto con la lectura real)
            import random
            posiciones = [random.randint(400, 600) for _ in range(5)]
            
            # Actualizar los labels en el thread principal de la GUI
            for i, valor in enumerate(posiciones):
                ventana.after(0, lambda i=i, v=valor: servo_valores[i].configure(text=str(v)))
            
        except Exception as e:
            print(f"Error actualizando posición: {e}")
        
        time.sleep(0.5)  # Actualizar cada 500ms

# Iniciar thread de actualización
thread_actualizacion = threading.Thread(target=actualizar_posicion, daemon=True)
thread_actualizacion.start()

# Función para mover el robot
def mover_a(posicion):
    def _thread():
        # Deshabilitar botones durante el movimiento
        for widget in botones_preset:
            widget.configure(state="disabled")
        boton_personalizado.configure(state="disabled")
        
        # Mostrar estado de movimiento
        estado_label.configure(text="Estado: Moviendo... ⏳", text_color="orange")
        
        pincher.cambioPos(posicion)
        
        # Rehabilitar botones
        for widget in botones_preset:
            widget.configure(state="normal")
        boton_personalizado.configure(state="normal")
        
        # Actualizar estado
        estado_label.configure(text="Estado: Conectado ✓", text_color="green")
    
    threading.Thread(target=_thread).start()

# Frame para posiciones predeterminadas
preset_frame = ctk.CTkFrame(main_frame, corner_radius=10)
preset_frame.pack(fill="x", pady=(0, 15))

label_presets = ctk.CTkLabel(
    preset_frame, 
    text="Posiciones Predeterminadas", 
    font=ctk.CTkFont(size=15, weight="bold")
)
label_presets.pack(pady=(10, 8))

# Crear botones para posiciones predeterminadas
botones_preset = []
for i, (nombre, posicion) in enumerate(preset_positions.items()):
    # Alternar colores para mejor visualización
    color = "green" if i % 2 == 0 else "blue"
    
    boton = ctk.CTkButton(
        preset_frame, 
        text=f"{nombre}: {posicion}", 
        width=350,
        height=32,
        corner_radius=8,
        font=ctk.CTkFont(size=13),
        command=lambda p=posicion: mover_a(p),
        fg_color=color,
        hover_color=("gray30", "gray70")
    )
    boton.pack(pady=4)
    botones_preset.append(boton)

# Frame para posición personalizada
custom_frame = ctk.CTkFrame(main_frame, corner_radius=10)
custom_frame.pack(fill="x", pady=(0, 15))

label_custom = ctk.CTkLabel(
    custom_frame, 
    text="Posición Personalizada", 
    font=ctk.CTkFont(size=15, weight="bold")
)
label_custom.pack(pady=(10, 5))

instrucciones = ctk.CTkLabel(
    custom_frame, 
    text="Ingresa 5 valores separados por comas (0-1023)", 
    font=ctk.CTkFont(size=11),
    text_color=("gray60", "gray40")
)
instrucciones.pack(pady=(0, 8))

# Entrada de texto con estilo mejorado
entrada = ctk.CTkEntry(
    custom_frame, 
    width=350,
    height=35,
    corner_radius=8,
    placeholder_text="512, 512, 512, 512, 512",
    font=ctk.CTkFont(size=13)
)
entrada.pack(pady=(0, 8))
entrada.insert(0, "512, 512, 512, 512, 512")

def enviar_personalizado():
    texto = entrada.get()
    try:
        valores = [int(x.strip()) for x in texto.split(',')]
        if len(valores) != 5:
            raise ValueError("Debe ingresar exactamente 5 valores")
        
        # Validar rango
        for val in valores:
            if not (0 <= val <= 1023):
                raise ValueError("Los valores deben estar entre 0 y 1023")
        
        mover_a(valores)
        
        # Feedback visual
        entrada.configure(border_color="green")
        ventana.after(2000, lambda: entrada.configure(border_color=("gray70", "gray30")))
        
    except ValueError as e:
        messagebox.showerror("Error", str(e))
        entrada.configure(border_color="red")
        ventana.after(2000, lambda: entrada.configure(border_color=("gray70", "gray30")))
    except:
        messagebox.showerror("Error", "Formato inválido. Ingresa 5 números separados por comas")
        entrada.configure(border_color="red")
        ventana.after(2000, lambda: entrada.configure(border_color=("gray70", "gray30")))

boton_personalizado = ctk.CTkButton(
    custom_frame,
    text="Mover a Posición Personalizada",
    width=300,
    height=35,
    corner_radius=8,
    font=ctk.CTkFont(size=13, weight="bold"),
    command=enviar_personalizado,
    fg_color="orange",
    hover_color="darkorange"
)
boton_personalizado.pack(pady=(0, 12))

# Botón de estado/información
info_frame = ctk.CTkFrame(main_frame, corner_radius=10)
info_frame.pack(fill="x")

estado_label = ctk.CTkLabel(
    info_frame,
    text="Estado: Conectado ✓",
    font=ctk.CTkFont(size=12),
    text_color="green"
)
estado_label.pack(pady=8)

# Cierre seguro
def cerrar():
    global actualizando
    actualizando = False
    thread_actualizacion.join(timeout=1)
    pincher.terminar()
    ventana.destroy()

ventana.protocol("WM_DELETE_WINDOW", cerrar)

# Centrar ventana en la pantalla
ventana.update_idletasks()
width = ventana.winfo_width()
height = ventana.winfo_height()
x = (ventana.winfo_screenwidth() // 2) - (width // 2)
y = (ventana.winfo_screenheight() // 2) - (height // 2)
ventana.geometry(f'{width}x{height}+{x}+{y}')

ventana.mainloop()