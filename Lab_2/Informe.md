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

> **Antes de empezar**
> - Asegúrate de que el área esté despejada y sin personas dentro del alcance del robot.
> - **Libera el paro de emergencia (E-STOP)** si está presionado.
> - Verifica que no haya alarmas activas en el péndulo.

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
   - Con **boton de hombre muerto** en posición intermedia:
   - Usa las teclas **S,L,U,R,B,T,E,8** de cada eje para mover de forma independiente.

5) **Mover en modo cartesiano (XYZ)**
   - Con **COORD** en **XYZ** y **deadman** activo:
   - **Traslaciones:** usa **X+ / X-**, **Y+ / Y-**, **Z+ / Z-** para mover el **TCP** en línea recta.
   - **Rotaciones:** usa **Rx+ / Rx-**, **Ry+ / Ry-**, **Rz+ / Rz-** para girar la orientación alrededor de los ejes X, Y, Z.
   - Verifica en pantalla el **marco activo** para saber respecto a qué sistema estás moviendo.

