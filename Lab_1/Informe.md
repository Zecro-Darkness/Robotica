
# Informe 1

## Solucion planteada

Descripcion detallada de la solucion planteada.

## Diagrama de flujo 
Diagrama de flujo de acciones del robot.
## Plano de planta
Plano de planta de la ubicacion de cada uno de los elementos.
## Funciones utilizadas
Descripcion de las funciones utilizadas.
## Diseño de la herramienta
### Objeivo de la herramienta
La herramienta diseñada tiene como objetivo principal permitir que el robot industrial IRB 140 pueda sostener y manipular un marcador o plumón para realizar la decoración de tortas virtuales o reales. Adicionalmente, el diseño incorpora un resorte que actúa como mecanismo de compensación ante posibles errores en la calibración de la herramienta o desviaciones propias del robot. Este resorte proporciona una tolerancia mecánica, asegurando que, incluso si existen pequeñas imprecisiones en la posición del robot o en el Tooldata, el marcador mantenga un contacto adecuado con la superficie de trabajo y permita un trazo continuo y preciso durante la ejecución de las trayectorias.
### Diseño conceptual herramienta
La herramienta diseñada es un soporte para marcador, destinada a sostener de manera segura un plumón o marcador en el flanche del robot industrial IRB 140. La herramienta tiene una forma cilíndrica y se compone de dos piezas principales:

**1. Cuerpo principal**:

- Incluye la base que se fija al flanche del robot mediante cuatro huecos de sujeción, garantizando un acople estable y seguro.

- Contiene un compartimento donde se inserta el marcador, formando un único conjunto con la base para facilitar la manipulación y la estabilidad del instrumento.

**2. Tapa**:

- Se ensambla al cuerpo principal mediante un sistema a presión, permitiendo un cierre seguro pero desmontable.

- Su función es mantener el marcador firme en su posición durante el movimiento del robot, evitando desplazamientos que puedan afectar la precisión de las trayectorias.

### Proceso de construcción
La herramienta fue diseñada en el software Fusion360 y se fabricó mediante impresión 3D con material PLA, sin presentar dificultades durante la fabricación. El único ajuste realizado fue lijar ligeramente la zona de inserción de la tapa, asegurando un ensamblaje a presión correcto y un acople firme del marcador durante el movimiento del robot.

 ![Herramienta](Imagenes/Montaje_1) ![Herramienta2](Imagenes/Montaje_2) |

### Calibración herramienta
La calibración de la herramienta se realizó posicionando el marcador montado en el flanche del robot sobre un punto de referencia común desde cuatro posiciones diferentes: frente, trasera, lado izquierdo y lado derecho. Este procedimiento permitió determinar con precisión el Tooldata, asegurando que el robot reconociera correctamente la posición y orientación de la punta del marcador durante la ejecución de las trayectorias.

## Codigo RAPID
Codigo en RAPID del m´odulo utilizado para el desarrollo de la practica.
## Video
Vıdeo que contenga la simulaci´on en RobotStudio ası como la implementaci´on de la pr´actica con los robots
reales.
Los videos debe comenzar con la introduccion oficial del laboratorio LabSIR Intro LabSIR.
