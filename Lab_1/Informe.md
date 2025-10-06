
# Informe 1 grupo 1c

## Integrantes

- Alejandro Mendivelso Torres
- Juan Manuel Beltran Botello 
- Oscar Jhondairo Siabato Leon

## Solución planteada

### Restricciones
Segun las restricciones:

 -El tamaño de la torta es para 20 personas
 
 -Las trayectorias a desarrollar deberán realizarse en un rango de velocidades entre 100 y 1000.
 
 -La zona tolerable de errores máxima debe ser de z10.
 
 -El movimiento debe partir de una posición home especificada (puede ser el home del robot) y realizar la trayectoria de cada palabra y decoración con un trazo continuo. El movimiento debe finalizar en la misma posición de home en la que se inició.
 
 -La decoración de la torta debe ser realizada sobre una torta virtual.
 
 -Los nombres deben estar separados.
 

### Solución 

Nuestra solución plantea una torta rectagunlar de dimensiones 32X22X7 cm que alcanza para 20 personas, los nombres que escribiremos son de los 3 integrantes (juan, oscar y alejo) junto a un dibujo del raton jerry animado, la velocidad escogida es de 100 para tener un menor velocidad pero mayor precision en la escritura por el marcador, se diseña la herramienta para el manipulador ABB, se crea el codigo para que la torta venga por la banda tranpostadora y el robot empiece en home, ademas de tener una pose para cambiar de herramienta, ademas se parametriza la banda transportadora para hacer los calculos de la velocidad y tiempo de duracion de cada pastel pase exacto para que el robot lo decore perfectamente. 

## Diagrama de flujo 
Diagrama de flujo de acciones del robot.
 ![Diagrama de flujo](Imagenes/Diagrama)
## Plano de planta
Plano de planta de la ubicacion de cada uno de los elementos.
 ![Plano de planta](Imagenes/Plano.png)
## Funciones utilizadas
Descripcion de las funciones utilizadas.

 -SetDO: Activa o desactiva una salida digital

 SetDO DO_01,0;   ! Apaga la salida digital DO_01
 
 SetDO DO_02,0;   ! Apaga la salida digital DO_02
 
 SetDO DO_01,1;   ! Enciende la salida DO_01


 -WaitTime: Pausa la ejecución durante t segundos

 WaitTime 4;   ! Espera 4 segundos antes de continuar
 
 WaitTime 5;   ! Espera 5 segundos al final de la secuencia


 -Conveyor_FWD: Controlan el transportador (conveyor), Conveyor_FWD lo enciende hacia adelante.

 Conveyor_FWD;   ! Enciende la banda transportadora
 
 WaitTime 4;     ! Espera 4 segundos mientras la banda mueve
 
 Conveyor_STOP;  ! Detiene la banda transportadora

 -Conveyor_STOP: Controlan el transportador (conveyor), Conveyor_STOP lo detiene.

  
 Conveyor_FWD;   ! Enciende la banda transportadora
 
 WaitTime 4;     ! Espera 4 segundos mientras la banda mueve
 
 Conveyor_STOP;  ! Detiene la banda transportadora

 -MoveJ: Movimiento articular (Joint) hacia objetivo.

 MoveJ Target_320, v100, z10, herramienta\WObj:=Workobject;


 -MoveL: Movimiento lineal hacia objetivo.

 MoveL Target_331, v100, z10, herramienta\WObj:=Workobject;


 -MoveC: Movimiento circular pasando por objetivo1 y terminando en objetivo2.

 MoveC Target_100, Target_101, v100, z10, herramienta\WObj:=Workobject;


 -PROC / ENDPROC: Define un procedimiento (subrutina).

  PROC Path_10()
  
     MoveL Target_310, v100, z10, herramienta\WObj:=Workobject;
     
     MoveL Target_311, v100, z10, herramienta\WObj:=Workobject;
     
     MoveL Target_312, v100, z10, herramienta\WObj:=Workobject;
     
 ENDPROC


 -WHILE: Bucle repetitivo.
 
  WHILE TRUE DO
  
     IF DI_02=1 THEN
     
         Path_770;
         
         SetDO DO_01,0;
         
     ENDIF
     
     ...
     
 ENDWHILE


 -IF / ENDIF: Condicional simple.

  IF DI_01=1 THEN
  
     SetDO DO_01,1;
     
     Conveyor_FWD;
     
     WaitTime 4;
     
     Conveyor_STOP;
     
     Path_10;
     
     Path_20;
     
     ...
     
 ENDIF


## Diseño de la herramienta

### Objetivo de la herramienta
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
El código RAPID desarrollado para este laboratorio tiene como objetivo ejecutar las trayectorias necesarias para la decoración de la torta virtual. Para ello, se diseñaron una serie de paths definidos manualmente y ajustados de manera iterativa hasta lograr el patrón de decoración deseado.

Cada trayectoria se encuentra organizada dentro del módulo llamado Module1, donde se establecen los puntos de inicio, fin y las posiciones intermedias del marcador sobre la superficie de trabajo. El código incluye:

**1. Rutinas de movimiento del robot:**

MoveL y MoveJ para movimientos lineales y conjuntos de articulaciones, respectivamente.

Las trayectorias se ejecutan de forma secuencial para asegurar trazos continuos y precisos.

**2. Control de condiciones con IF:**

Se implementa un condicional IF para iniciar la banda transportadora y la rutina de decoración, asegurando que el robot solo comience la operación cuando las condiciones sean correctas.

**3. Resultado en RobotStudio:**

La ejecución del código genera en la simulación de RobotStudio el diseño final de la decoración, reflejando fielmente la disposición de letras y patrones definidos en las trayectorias.

![Resultado](Imagenes/RobotStudio)

**4. Resultado Practica**

A continuación se muestra una imagen del resultado obtenido en la práctica real:

![Resultado](Imagenes/Practica)


## Video
Link del video 
https://youtu.be/Crd41DUmBFk


