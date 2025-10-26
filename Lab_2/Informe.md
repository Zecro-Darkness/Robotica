# Informe lab 2 grupo 1C

## Integrantes

- Juan Manuel Beltran Botello 
- Alejandro Mendivelso Torres
- Oscar Jhondairo Siabato Leon

## Comparativa técnica — Motoman MH6 vs ABB IRB 140

| Ítem | Motoman MH6 | ABB IRB 140 |
|---|---|---|
| **Fabricante / Controlador** | Yaskawa Motoman / **DX100** | ABB / **IRC5** |
| **Grados de libertad** | 8 ejes | 6 ejes |
| **Carga útil máxima** | **6 kg** | **6 kg** |
| **Alcance (horizontal)** | **1 422 mm** | **800 mm** |
| **Repetibilidad (pose)** | **±0,08 mm** | **±0,03 mm** |
| **Repetibilidad de trayectoria (lineal)** | N/D | **0,08 mm**|
| **Velocidad máx. por eje** | S: 220°/s · L: 200°/s · U: 220°/s · R: 410°/s · B: 410°/s · T: **610°/s** | 1: 200°/s · 2: 200°/s · 3: 260°/s · 4: 360°/s · 5: 360°/s · 6: **450°/s** |
| **Masa del manipulador** | ≈ **130 kg** | ≈ **98 kg** |
| **Montaje** | Piso, pared o techo | Piso, invertido o pared |
| **Aplicaciones típicas** | Manipulación, machine tending, ensamble, empaque, soldadura, dispensado | Manipulación, machine tending, ensamble, soldadura, limpieza/pintura, empaque |

## Home1 y Home2
### Home1

### Home2

## Movimiento manual — Paso a paso

 **Antes de empezar**
 - Asegúrate de que el área esté despejada y sin personas dentro del alcance del robot.
 - **Libera el paro de emergencia (E-STOP)** si está presionado.
 - Verifica que no haya alarmas activas en el péndulo.

1) **Seleccionar modo**
   - Gira la **llave del Teach Pendant** a **TEACH (Manual)**.
   - Esto bloquea la reproducción automática y habilita el “jog” manual.

2) **Preparar servos**
   - Pulsa **SERVO ON READY** para preparar el encendido.
   - **Mantén el interruptor de habilitación (deadman)** en la **posición intermedia** mientras mueves el robot.

3) **Elegir sistema de coordenadas**
   - Usa el botón **COORD** para alternar:
     - **JOINT (Articular):** movimiento eje por eje.
     - **CARTESIANO (XYZ):** traslaciones/rotaciones del TCP en X, Y, Z.

4) **Mover en modo articular (JOINT)**
   - Con el **Botón de hombre muerto** en posición intermedia:
   - Usa las teclas **S,L,U,R,B,T,E,8** de cada eje para mover de forma independiente.

5) **Mover en modo cartesiano (XYZ)**
   - Con **COORD** en **XYZ** y **deadman** activo:
   - **Traslaciones:** usa **X+ / X-**, **Y+ / Y-**, **Z+ / Z-** para mover el **TCP** en línea recta.
   - **Rotaciones:** usa **Rx+ / Rx-**, **Ry+ / Ry-**, **Rz+ / Rz-** para girar la orientación alrededor de los ejes X, Y, Z.
   - Verifica en pantalla el **marco activo** para saber respecto a qué sistema estás moviendo.

## Velocidades de movimiento manual (DX100)

### Niveles disponibles
Maneja **cuatro niveles** de velocidad manual:
- **INCH** → ultralento
- **SLOW (SLW)** → lento y controlado
- **MED** → intermedio
- **FAST (FST)** → rápido (solo usar en zonas despejadas)

### Cómo cambiar entre niveles (en el Teach Pendant)
Usa las teclas **FAST** y **SLOW** para ciclar por los niveles:

- Pulsando **FAST**: `INCH → SLOW → MED → FAST` (sube velocidad).
- Pulsando **SLOW**: `FAST → MED → SLOW → INCH` (baja velocidad).
- <img width="111" height="145" alt="image" src="https://github.com/user-attachments/assets/0ce561af-d684-41af-99fc-d4e76c0f7445" />

### ¿Dónde se ve el nivel activo en la interfaz?
- En el **Teach Pendant**, fíjate en la **indicación de velocidad manual** (área de estado/encabezado). Ahí aparece el nivel actual (**INCH/SLW/MED/FST**) de esta manera:

- <img width="510" height="160" alt="image" src="https://github.com/user-attachments/assets/99e6afcc-cbe9-4cec-a590-8392123a5a58" />
- <img width="627" height="154" alt="image" src="https://github.com/user-attachments/assets/722ddeef-5bdf-4250-9941-952936ecef29" />
- <img width="742" height="157" alt="image" src="https://github.com/user-attachments/assets/d9f8e972-cd23-459e-bf1f-4ffcd4d820c4" />
- <img width="635" height="153" alt="image" src="https://github.com/user-attachments/assets/666f2685-d2a2-4b59-a509-1210bc735ed5" />


### Buenas prácticas rápidas
- Arranca siempre en **INCH o SLOW** y sube a **MED/FAST** solo con entorno despejado.
- Mantén el **Botón de hombre muerto** en posición intermedia y servos listos antes de mover.
- Si cambias de herramienta/marco, vuelve a un nivel **bajo** hasta confirmar trayectorias.

## funcionalidades de RoboDK


## Comparativo: RoboDK vs RobotStudio

| **Aspecto** | **RoboDK** | **RobotStudio (ABB)** |
|--------------|-------------|------------------------|
| **Fabricante / Enfoque** | Desarrollado por RoboDK Inc. (independiente). Multimarca, compatible con +500 modelos de robots industriales. | Desarrollado por ABB Robotics. Software oficial para la familia **ABB IRC5/OmniCore**. |
| **Compatibilidad** | **Universal**: soporta ABB, Yaskawa/Motoman, Fanuc, KUKA, UR, Staubli, etc. | **Exclusivo de ABB**: solo trabaja con robots ABB y sus controladores. |
| **Tipo de programación** | **Offline** y **Online (en tiempo real)** con drivers. Genera código nativo (.JBI, .LS, .SRC, .PRG, etc.). | **Offline** (programación virtual) y **Online** con Virtual Controller (simulación exacta del IRC5). |
| **Lenguaje generado** | Usa **postprocesadores** que convierten la trayectoria a código nativo de cada marca (ej.: INFORM para Motoman). | Usa directamente **RAPID**, el lenguaje nativo de ABB (sin postproceso intermedio). |
| **Comunicación con el robot real** | Por **Ethernet** (drivers o FTP). En Motoman usa **HSE (High Speed Ethernet)** o **MotoCom**. | Comunicación directa con el **Virtual Controller ABB** o el robot físico vía **RobotWare**. |
| **Simulación de celda** | Permite crear celdas 3D con múltiples robots, cintas, PLC virtuales y periféricos CAD importados. | Simulación altamente realista del entorno ABB con cinemática exacta, trayectorias y herramientas virtuales. |
| **Postprocesado (exportación)** | Muy flexible: cada robot tiene su propio **post** editable (Python). Se pueden personalizar rutinas, formatos, cabeceras, etc. | No necesita postprocesador (el Virtual Controller ya ejecuta RAPID directamente). |
| **Precisión del modelo cinemático** | A nivel de fabricante (usa parámetros DH oficiales). Requiere calibrar si se desea correspondencia 1:1 con el robot físico. | Exactitud total: usa el **controlador virtual** idéntico al físico (misma firmware, librerías y limitaciones). |
| **Integración con visión, CNC y CAD/CAM** | Muy buena: importación CAD (STEP, IGES, STL) y conversión directa de trayectorias CAM a movimientos de robot. | Limitada a módulos específicos de ABB (Machining PowerPac, ArcWeld, PickMaster, etc.). |
| **Licenciamiento y costo** | Más **económico y flexible** (licencias modulares por funciones, incluso versión gratuita educativa). | **Comercial completo de ABB**, licencias por módulos y entorno cerrado. |
| **Ventajas clave** | Multimarca, liviano, rápido de configurar, personalizable, ideal para enseñanza, integración y pruebas de células mixtas. | Simulación de más alta fidelidad para ABB, entorno idéntico al real, sincronización exacta con IRC5/OmniCore. |
| **Limitaciones** | No reproduce exactamente los ciclos de control del fabricante (interpolaciones o zonas difieren levemente). | Solo funciona con robots ABB; mayor curva de aprendizaje si vienes de otras marcas. |
| **Aplicaciones típicas** | Diseño y simulación de células multimarca, optimización de trayectorias, generación de código CNC/robot, educación e investigación. | Programación avanzada ABB (RAPID), validación de rutinas industriales, entrenamiento y mantenimiento virtual. |

---

### ✅ **Conclusión**
- **RoboDK** es ideal cuando trabajas con **robots de diferentes marcas** (como Yaskawa + ABB), para crear gemelos digitales, probar trayectorias y generar código rápidamente.  
  Es **versátil, liviano y accesible** para proyectos académicos o células mixtas.

- **RobotStudio** es el entorno **más fiel y completo para ABB**, ya que usa el mismo controlador virtual que el robot real.  
  Es **imprescindible** si se requiere validar con precisión el comportamiento de un IRB 140 en producción o entrenamiento avanzado.

---

### 💡 **Recomendación para tu proyecto (MH6 vs IRB 140)**
- Usa **RoboDK** para programar, comparar y simular ambos robots en la misma celda virtual.
- Usa **RobotStudio** solo para validar la precisión y tiempos del **IRB 140** y generar programas RAPID optimizados para el IRC5.

