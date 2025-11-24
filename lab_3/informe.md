# Informe lab 3 grupo 1C

## Integrantes

- Juan Manuel Beltran Botello 
- Alejandro Mendivelso Torres
- Oscar Jhondairo Siabato Leon
  
# Comparativo técnico: Motoman MH6 vs. ABB IRB 140 vs. EPSON T3-401S

 Nota: Los datos se tomaron de fichas técnicas y catálogos de fabricante/distribuidores.

| Característica                           | **Yaskawa Motoman MH6**                               | **ABB IRB 140**                                                                 | **EPSON T3-401S (SCARA)**                                                                 |
|------------------------------------------|--------------------------------------------------------|----------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------|
| **Tipo**                                 | Brazo articulado                                       | Brazo articulado (6 ejes)                                                        | SCARA (4 ejes)                                                                            |
| **Grados de libertad (ejes)**            | 6                                                      | 6                                                                                | 4                                                                                         |
| **Carga máxima**                         | 6 kg                                                   | 6 kg                                                                             | 3 kg (1 kg nominal)                                                                       |
| **Alcance (radio/horizontal)**           | 1422 mm                                                | 810 mm (hasta eje 5)                                                             | 400 mm                                                                                     |
| **Repetibilidad**                        | ±0.08 mm                                               | ±0.03 mm                                                                         | ±0.02 mm (J1–J3); ±0.02° (J4)                                                             |
| **Velocidad por eje (máx.)**             | J1: 220°/s, J2: 200°/s, J3: 220°/s, J4: 410°/s, J5: 410°/s, J6: 610°/s | J1: 200°/s (140T: 250°/s), J2: 200°/s (140T: 250°/s), J3: 260°/s, J4: 360°/s, J5: 360°/s, J6: 450°/s | Ciclo estándar: **0.52–0.54 s** (no se especifican velocidades angulares por eje en la hoja de ventas) |
| **Velocidad TCP (máx.)**                 | —                                                      | 2.5 m/s                                                                          | —                                                                                         |
| **Carrera Z / Eje 3**                    | —                                                      | —                                                                                | 150 mm                                                                                    |
| **Protección / variantes**               | —                                                      | IP67; variantes Foundry Plus 2, Clean Room (ISO 6), SteamWash                    | —                                                                                         |
| **Montaje**                              | — (típico: suelo/invertido/ángulo)*                    | Cualquier ángulo: suelo, pared, invertido                                        | Sobremesa (tabletop)                                                                      |
| **Peso del manipulador**                 | 130 kg                                                 | 98 kg                                                                            | 16 kg                                                                                     |
| **Aplicaciones típicas**                 | Manipulación de materiales, ensamblaje, dispensado, soldadura | Soldadura por arco, ensamblaje, limpieza/pintura, tending, manipulación, empaque, desbarbado | Pick & place, inspección, ensamblaje                                                      |
| **Controlador**                          | — (serie DX)                                           | IRC5 (Compact / Single / Panel)                                                  | Controlador **integrado** (EPSON RC+)                                                     |

# Configuraciones *home* del EPSON T3-401S

En el EPSON T3-401S la referencia básica de **home** es la posición de **0 pulsos** de cada articulación.

En nuestro caso, el eje J1 tiene asignado un valor de **204800 pulsos** en la posición de **home**.  

Esta elección se hizo siguiendo la recomendación del profesor.

Con esto también queremos mostrar que el *home* **no tiene que ser necesariamente** la posición de 0 pulsos en todos los ejes (aunque esa suele ser la referencia ideal o “mecánica”).  
El entorno de Epson nos permite **ajustar o redefinir la posición de home de forma sencilla**, aplicando un desplazamiento (offset) sobre los pulsos y usando ese valor como nueva referencia de trabajo.

**Caso ideal**

| Articulación | Tipo / Eje                      | Posición en *home* (0 pulsos)              | Descripción geométrica / funcional                                                                 |
|--------------|----------------------------------|--------------------------------------------|-----------------------------------------------------------------------------------------------------|
| **J1**       | Rotación base (primer brazo)     | **J1 = 0°**                                | El brazo 1 se orienta hacia el eje **+X** del sistema de coordenadas del robot.                     |
| **J2**       | Rotación segundo brazo           | **J2 = 0°**                                | El brazo 2 queda **alineado con el brazo 1**, es decir, el brazo está completamente extendido.      |
| **J3**       | Eje vertical **Z** (prismático)  | **J3 = 0 mm** (origen / posición superior) | El eje Z está en su **tope superior**. A partir de ahí, los desplazamientos típicos serán hacia abajo (Z−). |
| **J4**       | Rotación herramienta (flange)    | **J4 = 0°**                                | El eje de la herramienta queda alineado con el segundo brazo, con la orientación “neutra” del flange.       |

**como esta configurado**

<img width="276" height="427" alt="image" src="https://github.com/user-attachments/assets/7bec95d9-513f-4ea5-b648-696de1eea28f" />
<img width="426" height="320" alt="image" src="https://github.com/user-attachments/assets/6227742d-6c0a-41d1-ab6d-2965d9d9c789" />



# EPSON T3-401S — Procedimiento para movimientos manuales
> Paso a paso para **desplazar el robot manualmente** desde EPSON RC+ (PC) y desde Teach Pendant, cambiando entre **modos articulados** y **cartesianos** y ejecutando **traslaciones** y **rotaciones** en X, Y, Z.

1. Abrir administrador de robot
   
2. Abrir la pestaña panel de control en **administrador de robot** y habilita motores dando en el botón **Motor On** (botón/menú o comando).  
<img width="297" height="189" alt="image" src="https://github.com/user-attachments/assets/268af207-666c-483a-839d-2ff7b54fd0e7" />

3. Abrir la ventana de mover y enseñar en **administrador de robot**
<img width="805" height="543" alt="image" src="https://github.com/user-attachments/assets/956eb79b-c05d-4f57-988d-689c371bf3bf" />
En este apartado encontraremos los siguiente modos

-Mundo (cartesiano)

-Herramienta (cartesiano)

-Local (cartesiano)

-Articulación (articulaciones)

-ECP (cartesiano)

### 1.4 Ejecutar **traslaciones** y **rotaciones**

- En **Joint**
  
use los botones de **J1, J2, J3, J4** (+/–) para girar cada eje. **J3** es lineal (Z), **J4** es rotacional (herramienta).

<img width="334" height="400" alt="image" src="https://github.com/user-attachments/assets/b1f9d8fa-0d32-44a6-9b47-3f8fd1fb9d14" />

- En **World/Tool/Local/ECP**:

**Traslación**: botones **+X/–X, +Y/–Y, +Z/–Z**.
     
**Rotación**: +U/–U
    
<img width="337" height="407" alt="image" src="https://github.com/user-attachments/assets/0ac9f029-b436-4e9c-ba3b-ddf93426efa3" />


# EPSON T3-401S — Niveles de velocidad para movimientos manuales

Enla ventana de **administrador de robot** ingresamos al apartado de mover y enseñar y aqui tenemos dos formas de cambia los niveles de velocidad de robot, ademas en esta misma pestaña podemos identificar el nivel de velocidad actual y son las siguientes:

### 1. Parámetro **Velocidad** (Low / High)

<img width="334" height="107" alt="image" src="https://github.com/user-attachments/assets/2b85942d-0dc0-49c2-9252-bf9a29f764cf" />

En la sección de jog existe un cuadro de selección llamado **Velocidad**, con dos opciones:

- **Low** → Velocidad baja de jog
  
- **High** → Velocidad alta de jog

Este parámetro determina **qué tan rápido se mueve el robot** cuando se utilizan los botones de jog en la interfaz.

> En modo TEACH la velocidad máxima está limitada por seguridad, pero dentro de ese límite, el cambio Low/High hace que el movimiento sea más lento o más rápido.

### 2. Parámetro **Distancia de movimiento** (Continuous / Long / Medium / Short)

<img width="427" height="151" alt="image" src="https://github.com/user-attachments/assets/5d5dc5e4-2913-4e04-abae-0f284a287e0a" />

el grupo **Distancia de movimiento**, con las opciones:

- **Continuous** → el robot se mueve mientras el botón de jog esté presionado.
  
- **Long** → movimiento por pasos grandes.
  
- **Medium** → movimiento por pasos medianos.
  
- **Short** → movimiento por pasos muy pequeños.

y si simplemente cambiamos entre estos modos para establecer las velocidades ademas tambien se establece el valor de los pasos en los espacios que dicen **X (mm)**, **Y (mm)**, **Z (mm)** y **U (mm)**

Este parámetro **no cambia la velocidad**, pero sí influye en **cuánto se desplaza el robot por cada acción de jog**, lo que afecta la “sensación” de rapidez y precisión del movimiento.

# Funcionalidades principales de EPSON RC+ 7.0

### a) Entorno de desarrollo y control de movimiento

* Para el T3-401S el “entorno de desarrollo” oficial es **EPSON RC+ 7.0** y el lenguaje de programación es **SPEL+**, un lenguaje de robot multitarea. 
* El controlador interno del robot se encarga del **control simultáneo de las 4 articulaciones**, con servocontrol digital AC y modos de movimiento **PTP (punto a punto)** y **CP (trayectoria continua)**, con velocidad y aceleración programables. RC+ es la interfaz donde tú defines esos movimientos y parámetros. 

En corto: RC+ es donde programas; el controlador del robot es quien realmente interpola trayectorias y cierra los lazos de posición/velocidad.

---

### b) Configuración de conexión PC–controlador

RC+ tiene un diálogo específico: **“PC to Controller Communications”**:

* Paso típico para conectar:

  1. Tener instalado EPSON RC+ 7.0 en el PC. 
  2. Conectar el PC al manipulador con el **cable USB**. 
  3. Encender el manipulador.
  4. Abrir EPSON RC+ 7.0. 
  5. Menú **Setup → PC to Controller Communications**, seleccionar “No.1 USB” y pulsar **Connect**. 
* Cuando la conexión se completa, en el estado aparece **“Connected”** y ya puedes usar el sistema de robot desde RC+. 

---

### c) Simulador y robot virtual

RC+ incluye un **simulador**:

* Permite configurar un **robot virtual**, usando un árbol de objetos y propiedades (robot, mesas, piezas, etc.).
* Puedes seleccionar conexión **Offline**, cargar un modelo “T3-401S”, crear escenas y probar programas sin conectar el robot físico. 

Esto te sirve para depurar trayectorias, lógicas y tiempos de ciclo antes de arriesgar el equipo real.

---

### d) Robot Manager, JOG & TEACH y Home

Dentro de RC+ está el **Robot Manager**, que es el “panel de operación” del robot:

* Permite crear un proyecto nuevo, controlar el robot, ver un panel de control y usar herramientas de operación segura.
* Con **JOG & TEACH** puedes:

  * Activar motores, mover el robot en modo manual (jog) por ejes o en coordenadas.
  * Ver las **coordenadas actuales**, definir el **Home** y registrar puntos (P1, P2, …). 
  * Mandar al robot a un punto con comandos de alto nivel desde la interfaz: **Go, Move, Jump**. 

---

### e) Editor SPEL+, compilación y ventana de RUN

RC+ tiene un editor específico para **SPEL+**:

* Temario: tipos de variables, operadores, comandos básicos, estructura de programas y **primer programa SPEL+**.
* Hay un botón o tecla **F5** que **compila el programa** y abre la **ventana de RUN**:

  * Ahí puedes fijar la potencia en **Low** sin importar lo que diga el programa.
  * También puedes establecer un **factor de velocidad máxima** global. 
* El temario incluye comandos:

  * `Wait`, bucles `Do...Loop`, condicionales, `Print`, bucles `For/While/Until`, lectura y escritura de I/O, **simulación de I/O**, variables globales, funciones y `Pallet`.

---

### f) Supervisión de I/O y mantenimiento

* Desde EPSON RC+ se puede:

  * **Simular entradas digitales** y ver estados de salidas (útil sin PLC ni sensores conectados). 
  * Ver y editar la **información de mantenimiento** (batería, engrase, etc.) usando **Tools → Maintenance** y los diálogos correspondientes.

---

# ¿Cómo se comunica con el manipulador y qué hace para ejecutar un movimiento?

### a) Comunicación física y lógica

1. **Enlace físico:**

   * T3-401S se conecta al PC por **USB (No.1 USB)**. RC+ abre ese canal usando el diálogo de **PC to Controller Communications**.

2. **Capas lógicas (simplificado):**

   * **RC+ en el PC**:

     * Edita y compila los programas SPEL+.
     * Envía el programa al controlador interno del robot.
     * Envía órdenes de “Run/Stop/Home/Jog” y lee estados.
   * **Controlador del T3**:

     * Interpreta el programa SPEL+.
     * Convierte comandos como `Move`, `Jump`, `Speed`, `Accel` en trayectorias (PTP o CP) con planificación de velocidad y aceleración. 
     * Ejecuta los lazos de servocontrol de las 4 articulaciones en tiempo real.

---

### b) Flujo típico para que el robot se mueva

Te lo pongo como “pipeline” de lo que pasa cuando tú haces que el robot vaya de Home a P1:

1. **Conexión:**

   * PC ↔ manipulador por USB, estado “Connected” en RC+. 

2. **Configuración de robot y proyecto:**

   * En RC+ (offline u online) se selecciona el modelo de robot (T3-401S) y se crea un proyecto asociado.

3. **Teach de puntos:**

   * En Robot Manager / JOG & TEACH:

     * Se energizan motores, se mueve el robot a una posición física y se registra como P1, P2, etc.
     * Se define la postura de **Home** y se guarda. 

4. **Programación:**

   * En el editor SPEL+ escribes algo tipo:

     ```spel
     Motor On
     Speed 50
     Accel 50,50
     Home
     Move P1
     ```
   * Compilas (F5); RC+ pasa ese código al controlador, que lo almacena en su memoria.

5. **Ejecución:**

   * Desde la ventana de RUN o Robot Manager:

     * RC+ manda la orden de **Run** al controlador.
     * El controlador activa el servocontrol y calcula la trayectoria Home→P1 según PTP o CP, con los parámetros de `Speed` y `Accel`.
     * Los lazos de servocontrol en las articulaciones generan las referencias de corriente/velocidad a los motores.

6. **Retroalimentación y estados:**

   * El controlador devuelve a RC+ el estado (Ready, Running, Error, etc.). En sistemas con Remote I/O, también activa salidas lógicas como **Ready**, **Running**, **Error**, que se pueden leer desde un PLC u otro sistema. 

7. **Parada / emergencia (si aplica):**

   * Si se pulsa E-STOP o se pierde el USB sin desconectar, el manual indica que el manipulador se detiene. 


# Análisis comparativo entre EPSON RC+ 7.0, RoboDK y RobotStudio

| Aspecto / Criterio                  | **EPSON RC+ 7.0**                                                                                                      | **RoboDK**                                                                                                                  | **RobotStudio (ABB)**                                                                                              |
|------------------------------------|------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------|
| **Tipo de herramienta**            | Entorno oficial de programación, simulación y operación para robots **Epson**                                          | Entorno de **simulación y OLP multimarca**                                                                                 | Entorno oficial de **simulación + OLP + gemelo digital** para robots **ABB**                                       |
| **Marcas soportadas**              | Solo **Epson** (SCARA, T-series como T3-401S, 6 ejes, etc.)                                                            | Multimarca: **ABB, KUKA, Fanuc, UR, Epson, etc.**                                                                           | Solo **ABB**                                                                                                       |
| **Lenguaje principal**             | **SPEL+** (propietario Epson)                                                                                          | Genera **código nativo** de muchas marcas (RAPID, KRL, TP, etc.) vía postprocesadores                                      | **RAPID** (lenguaje oficial ABB)                                                                                   |
| **Simulación 3D**                  | Simulación integrada; enfocada en validar trayectorias y celdas Epson                                                  | Simulación flexible de celdas complejas, mecanismos extra y procesos tipo CAD/CAM                                          | Simulación con **Virtual Controller** (misma lógica que el robot real, gemelo digital casi 1:1)                   |
| **Integración con hardware real**  | Máxima para Epson: jog, ejecución, visión, I/O, HMI todo en el mismo entorno                                           | Depende del controlador y del postprocesador; genera código que luego se carga al robot                                    | Máxima para ABB: configuración de sistema, I/O, herramientas, estaciones, seguridad                                |
| **Visión / Fuerza / Conveyors**    | Módulos nativos: **Vision Guide**, force control, tracking, GUI Builder dentro de RC+                                   | Puede simular sensores, pero la integración “oficial” depende de cada marca/controlador                                   | Módulos oficiales ABB para visión, tracking, procesos (soldadura, pegado, etc.)                                    |
| **Ventajas clave**                 | - Integración total con robots Epson<br>- Misma herramienta para programar y operar<br>- Visión y HMI nativos          | - Multimarca (un solo entorno para muchas marcas)<br>- Fuerte en OLP y CAD/CAM<br>- Ideal para docencia e integración mixta | - Gemelo digital muy fiel<br>- Gran reducción de tiempo de puesta en marcha<br>- Potente para ingeniería de celdas |
| **Limitaciones clave**            | - Solo Epson<br>- Simulación 3D menos genérica que RoboDK                                                              | - No es “controlador oficial” de ninguna marca<br>- Depende mucho de postprocesadores<br>- Algunas funciones avanzadas requieren retoques | - Solo ABB<br>- Curva de aprendizaje moderada (RAPID + arquitectura ABB)<br>- Licencias completas son de pago      |
| **Aplicaciones típicas**           | - Celdas con robots Epson (pick & place, ensamblaje, empaques, visión)<br>- Laboratorios con T3/T6/SCARA Epson         | - Plantas con **varias marcas** de robots<br>- Mecanizado, corte, pulido, pick & place multimarca                          | - Plantas estandarizadas en ABB<br>- Celdas complejas (soldadura, pintura, manipulación avanzada)                  |

# Diseño técnico del gripper neumático por vacío

En esta sección se describe el diseño técnico del gripper neumático por vacío utilizado en el sistema, incluyendo el diagrama esquemático, los componentes principales y la configuración de las entradas/salidas digitales del robot.

### Diagrama esquemático

<img width="372" height="83" alt="robotica drawio" src="https://github.com/user-attachments/assets/f80f91d8-7988-484a-85ee-b114e9c29a5b" />

### Configuración de E/S digitales del robot

El accionamiento del gripper se realiza mediante una salida digital del controlador del robot, conectada a la bobina de la válvula solenoide:

- **Salida digital usada**: Out_9.
- **Función**:
  - **ON:** activa la válvula solenoide, genera vacío y la ventosa sujeta la pieza.
  - **OFF:** desactiva la válvula, se pierde el vacío y la pieza se libera.
    
### Componentes utilizados

Los componentes principales del gripper neumático por vacío son:

- Ventosa de la marca festo
- venturi
- Válvula solenoide 24 VDC: `<<modelo o referencia>>`
- Mangueras neumáticas y racores: `<<diámetro / tipo>>`
- Fuente de aire comprimido: `<<presión de trabajo (ej. 6 bar)>>`
- Elementos de montaje mecánico para fijar el gripper al efector final del robot.

Los **planos del soporte** lo pueden observar en el archvo que se llama **Planos_Gripper.pdf**.

# Diagrama de flujo

<img width="415" height="2880" alt="Untitled diagram-2025-11-21-064623" src="https://github.com/user-attachments/assets/9c305108-e75f-4805-9d2e-dc83fde7069e" />



<img width="385" height="3296" alt="Untitled diagram-2025-11-21-064633" src="https://github.com/user-attachments/assets/8873e951-8c98-450c-8ba3-9eda48807e5b" />



<img width="385" height="2854" alt="Untitled diagram-2025-11-21-064639" src="https://github.com/user-attachments/assets/e75fdaac-408f-41ec-8521-392d9783fef9" />



<img width="385" height="2884" alt="Untitled diagram-2025-11-21-064646" src="https://github.com/user-attachments/assets/dc8879d6-7968-444a-b52a-21f4aa44bd2e" />



<img width="412" height="2522" alt="Untitled diagram-2025-11-21-064912" src="https://github.com/user-attachments/assets/94e81645-6722-4543-8265-3317550f7319" />




# Plano de planta

<img width="903" height="904" alt="Ensamble v4" src="https://github.com/user-attachments/assets/220eedff-2729-4b8e-ad43-f70200c3a808" />

<img width="2482" height="1755" alt="Ensamble Dibujo v1_page-0001" src="https://github.com/user-attachments/assets/567cfaaf-eb48-481e-ae9d-67fc6a96f962" />

<img width="2482" height="1755" alt="Ensamble Dibujo v1_page-0002" src="https://github.com/user-attachments/assets/0a817eb0-25f9-465b-894d-0c709ec552bb" />

# Video
## Simulacion
https://youtu.be/NJ_B73AV66c
## laboratorio
https://youtu.be/Sp665tFNVjU
