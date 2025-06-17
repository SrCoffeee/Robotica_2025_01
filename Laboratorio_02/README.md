# Laboratorio No. 02
# Robótica Industrial- Trayectorias, Entradas y Salidas Digitales.

* David Camilo Valbuena Molano
* Carlos Fernando Quintero Castillo
---

## Introducción 
Este laboratorio busca poner en práctica conceptos clave de la robótica industrial, como la programación de trayectorias, el diseño de herramientas y el uso de señales digitales. A través de la simulación de la decoración de una torta con un robot ABB IRB 140, se diseñaron movimientos personalizados, calibración de la herramienta y configuración entradas y salidas digitales, integrando así programación en RAPID con tareas reales de automatización.

## Herramienta de trabajo
Se modeló la herramienta según las especificaciones del flanche en la guia de laboratorio, posteriormente se cargó al entorno de robot studio y se asigno al robot correspondiente el cual es el ABB IRB 140 se define que es una herramienta de trabajo y se procede a crear en robot studio aparece en la ventana de diseño que es un mecanismo en este caso de nombre porta
![her1](/her1.png)
![her2](/her2.png) 

El marco de referencia de la herramienta se modifica de tal forma que el eje z coincida con el ataque del marcador esto será especialmente útil al momento de realizar las trayectorias que nos permitirán definir nuestra decoración
![porta](/porta.png)




## Plano de planta
Se adjunta una vista de planta de la estación en la cual se visualiza el robot, la banda transportadora y el objeto de trabajo en el que se va a hacer el decorado 
![planoplanta](/planoplanta.png)





## Descripción de la solución 
En primera instancia se modela un solido de dimensiones Ancho 160mm, Largo 235mm y Alto 47.5mm, en el cual se coloca el texto que va incluir para este caso es el nombre primer nombre de los integrantes del equipo (CARLOS y DAVID) y una figura a gusto,  posteriormente se extruye para así crear las referencias que van a tener los puntos de las trayectorias en robot studio.

![deco](/deco.png)


Una vez importado a robot studio se define como el work object (aunque en realidad el work object es el plano sobre el que se ejecutan las trayectorias) y se seleccionan los puntos para hacer cada trayectoria en RAPID hay 3 comandos principales para el sieño de trayectorias estos son MOVEL. MOVEJ, MOVEC el primero sirve para hacer trayectorias lineales, el segundo para ir a un punto en especifico y el tercero para hacer trayectorias curvas  
 


Una vez importado a robot studio se proceden a haecr las trayectorias 




## Diagrama de flujo de acciones del robot

Una secuencia simple de las acciones del robot es el siguiente: 

<img src="/Secuencia_simple.png" alt="secuencia simple" width="400">



## Código en RAPID

Para este apartado se puede encontrar el código utilizado en la carpeta *"Códigos"*.
