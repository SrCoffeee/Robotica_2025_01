# Laboratorio No. 04
# Robótica de Desarrollo, Cinemática Directa - Phantom X- ROS

* David Camilo Valbuena Molano
* Carlos Fernando Quintero Castillo
---

## Introducción 
El presente laboratorio tiene como objetivo implementar la cinemática directa del manipulador Phantom X Pincher utilizando herramientas de programación en Python y el framework ROS 2. La práctica se centra en la configuración y control de los servomotores Dynamixel AX-12 mediante joint controllers y tópicos de ROS, así como la visualización y verificación de movimientos articulares a través de una interfaz gráfica de usuario. Este proyecto combina la robótica física con la representación gráfica, permitiendo al estudiante consolidar conocimientos sobre modelado cinemático, control en tiempo real y desarrollo de interfaces HMI. 

## Descripción detallada de la solución planteada.

Control de articulaciones vía ROS 2
Se implementaron scripts en Python que utilizan publishers para enviar comandos de posición a los servomotores waist, shoulder, elbow, wrist_pitch y wrist_roll. Estos comandos corresponden a valores articulares definidos previamente y validados dentro de los límites de cada motor.

Interfaz de usuario (HMI)
Se diseñó una interfaz gráfica que permite al usuario:

Seleccionar una de las cinco configuraciones articulares dadas.

Mostrar en pantalla los valores reales de cada articulación.


## Diagrama de flujo de acciones del robot unsando la herramieenta Mermaid





## Plano de planta de la ubicación de cada uno de los elementos.






## Descripción de las funciones utilizadas.




## Código del script utilizado para el desarrollo de la práctica.

Para este apartado se puede encontrar el código utilizado en la carpeta *"Códigos"*.

## Video explicativo (alcanzando cada posición solicitada y del uso de la interfaz de usuario)
Para un mayor detalle de lo realizado, se puede ver el video de la implementación física [aquí](https://youtu.be/5WKLtGd51os)
