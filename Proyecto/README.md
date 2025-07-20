# Proyecto Final - Robótica Industrial
# Automatización del Proceso de Preparación de Arepas

* David Camilo Valbuena Molano
* Juliana Gongora Rasmussen
* Gerhaldine Alejandra Suárez Bernal
* Carlos Fernando Quintero Castillo
---
## Introducción
El presente proyecto final de robótica industrial se enfoca en automatizar el proceso de preparación de arepas —tradicionalmente manual— mediante un sistema robotizado que garantice una cocción uniforme y precisa de cada unidad. El sistema integra además sistemas electro-neumáticos para el accionamiento del gripper, garantizando control preciso durante todo el proceso. Para la interacción con el operador se implementa una interfaz humano-máquina (HMI) gráfica en el FlexPendant mediante ScreenMaker de RobotStudio, que permite seleccionar la arepa deseada y reportar el estado del proceso. A su vez, se emplea el software de simulación RobotStudio para modelar y probar virtualmente el sistema completo, facilitando las etapas de diseño y validación sin riesgo físico.

## Descripción de la solución creada

### 1. Planeación y asignación de tareas
  • Reunión inicial para definir objetivos y alcance.  
  • Asignación de roles:  
    - Investigación de actuadores y componentes neumáticos  
    - Diseño CAD del gripper  
    - Selección de mangueras, acoples y accesorios hidráulicos  
    - Integración con RobotStudio
    - HMI FlexPendant 
### 2. Selección de componentes principales
- **Actuador neumático**  
  - Se eligió una pinza neumática (marca Mindman, modelo MCHA-20) por su precisión en el posicionamiento, facil acople y adaptabilidad al reto propuesto.
- **Sistema de agarre (gripper)**  
  - Bocetos iniciales realizados en Autodesk Inventor.  
  - Material: Plastico impreso en PLA por facilidad, alcance y adaptabilidad al actuador.  
- **Mangueras y acoples**  
  - Manguera poliuretano 1/4″ (ID) resistente a 8 bar.  
  - Conectores rápidos SMC en latón con rosca BSP 1/8″ para facilitar el montaje y mantenimiento.  
### 3. Diseño y pruebas del gripper
- **Modelado CAD y simulación**  
  - Conjunto 3D del gripper en Autodesk Inventor, validación de interferencias.  
  - Integración virtual en RobotStudio junto al robot ABB IRB 120.  


### 4. Montaje físico y puesta en marcha
- Fabricación del gripper en impresora 3D.  
- Instalación de mangueras, acoples y válvulas.  
- Conexionado neumático y cableado de sensores y válvulas a la estación de control.  
- Configuración de programa RAPID en RobotStudio y ScreenMaker en el FlexPendant.
### 5. Herramientas y recursos utilizados
- Software  
  - RobotStudio 2024 SP1  
  - Autodesk Inventor 2025   
- Equipos de taller  
  - Impresora 3D Ultimaker S3  
  - Banco de pruebas neumático con manómetro y depósito de aire comprimido  
- Piezas y consumibles  
  - Pinza neumática Mindman MCHA-20 
  - Manguera poliuretano 1/4″ y conectores rápidos SMC  
  - Válvulas solenoides SMC SY5120   
  - Tornillería M5 y perfiles de aluminio 20×20 mm

##  Diagrama de flujo de las acciones del robot
##  Descripción, planos y fotografías del gripper diseñado y sus piezas 
##  Modelo en Software de Simulación
## Códigos

Para este apartado se puede encontrar el código utilizado en la carpeta *"Códigos"*.

## Video explicativo
Para un mayor detalle de lo realizado, se puede ver el video de la implementación física [aquí](https://youtu.be/5WKLtGd51os)
