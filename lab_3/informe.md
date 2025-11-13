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



