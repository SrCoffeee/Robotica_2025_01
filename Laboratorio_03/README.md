# Laboratorio No. 03
# Robótica Industrial- Análisis y Operación del Manipulador Motoman MH6.

* David Camilo Valbuena Molano
* Carlos Fernando Quintero Castillo
---

## Introducción 
Los manipuladores industriales son brazos robóticos articulados diseñados para automatizar tareas repetitivas, peligrosas o de alta precisión en entornos de manufactura, ensamblaje, soldadura, pintura y más. Em esta practica nos enfocaremos en el diseño de trayectorias polares apoyados en el software RobotDK con el objetivo de sentar las bases teóricas y prácticas para la programación offline, simulación y puesta en marcha de movimientos tanto virtuales como reales. Con ello podemos establecer criterios compartivos tanto en los entornos (RoboDK y RobotStudio) como en los manipuladores ­–el Yaskawa Motoman MH6 y el ABB IRB 140– con el fin de concretar las principales fortalezas y aplicaciones de cada herramienta.

## Comparación técnica entre el Motoman MH6 y el IRB140


| Característica               | Motoman MH6                          | ABB IRB 140                         |
|------------------------------|--------------------------------------|-------------------------------------|
| Carga máxima                 | 6 kg                                 | 6 kg                                |
| Alcance máximo               | 726 mm                               | 820 mm                              |
| Grados de libertad (DoF)     | 8 ejes                               | 6 ejes                              |
| Velocidad máxima lineal      | 1 500 mm/s                           | 2 000 mm/s                          |
| Repetibilidad                | ± 0,02 mm                            | ± 0,03 mm                           |
| Peso del manipulador         | 45 kg                                | 105 kg                              |
| Volumen del espacio de trabajo | Cilíndrico de Ø 1 450 mm y altura 1 100 mm | Cilíndrico de Ø 1 640 mm y altura 1 600 mm |
| Aplicaciones típicas         | Pick & place de componentes ligeros, embalaje, inspección | Montajes de ensamblaje pequeños, pick & place, handling, Drawing          |

**Observaciones**  
- Aunque ambos comparten 6 kg de carga, el IRB 140 ofrece mayor alcance y velocidad lineal, a costa de un peso mucho mayor.  
- El MH6 es más compacto y ligero, ideal en estaciones de mesas de trabajo con limitación de espacio.
  
##  Descripción de las configuraciones home1 y home2



## Control Manual del Manipulador: Movimiento Articular y Cartesiano

## Niveles de velocidad del Motoman

## Descripción de las principales funcionalidades de RoboDK,

## Análisis comparativo entre RoboDK y RobotStudio

## Código en Python

Para este apartado se puede encontrar el código utilizado en la carpeta *"Códigos"*.

## Video explicativo
Para un mayor detalle de lo realizado, se puede ver el video de la implementación física [aquí](enlace)
