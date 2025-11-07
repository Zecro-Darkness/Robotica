# Comparativo técnico: Motoman MH6 vs. ABB IRB 140 vs. EPSON T3-401S

> Nota: Los datos se tomaron de fichas técnicas y catálogos de fabricante/distribuidores.

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



# EPSON T3-401S — Procedimiento para movimientos manuales (Jog)
> Paso a paso para **desplazar el robot manualmente** desde EPSON RC+ (PC) y desde Teach Pendant, cambiando entre **modos articulados** y **cartesianos** y ejecutando **traslaciones** y **rotaciones** en X, Y, Z.

---

## 0) Requisitos y seguridad (antes de mover)
1. **Zona despejada y protecciones activas.** Verifique resguardos, paro de emergencia accesible y límites/SLP si aplica. :contentReference[oaicite:0]{index=0}  
2. **Energizar y habilitar motores.** En RC+: encender controlador y robot; luego **Motor On** (botón/menú o comando). :contentReference[oaicite:1]{index=1}  
3. **Ir a referencia si es necesario.** Puede usar `Go Pulse (0,0,0,0)` para llevar al **pulso 0** (home mecánico) antes de enseñar. :contentReference[oaicite:2]{index=2}  

---

## 1) Jog desde **EPSON RC+** (PC)

### 1.1 Abrir la ventana de Jog
1. Inicie **EPSON RC+** y abra **Jog & Teach**. (Menú/ver pestaña “Desplazar y enseñar”). :contentReference[oaicite:3]{index=3}

### 1.2 Seleccionar **modo de operación**
En la caja **Mode** elija el sistema de coordenadas:  
- **Joint** (articulaciones): mueve **cada eje** por separado. :contentReference[oaicite:4]{index=4}  
- **World / Tool / Local / ECP** (si ECP está habilitado): mueven en el sistema **cartesiano** (X, Y, Z). :contentReference[oaicite:5]{index=5}  
> Para un SCARA T3, la rotación cartesiana disponible es **Rz** (alrededor de Z); **Rx/Ry** no aplican al manipulador. (Propio de la cinemática SCARA.)

### 1.3 Ajustar **Velocidad** y **Distancia de Jog**
- **Speed:** fija la velocidad del movimiento.  
- **Jog Distance:** define el **incremento** en **Step**; en **Continuous** el movimiento es sostenido mientras se mantiene el botón. :contentReference[oaicite:6]{index=6}

### 1.4 Ejecutar **traslaciones** y **rotaciones**
- En **Joint**: use los botones de **J1, J2, J3, J4** (+/–) para girar cada eje. **J3** es lineal (Z), **J4** es rotacional (herramienta). :contentReference[oaicite:7]{index=7}  
- En **World/Tool/Local/ECP**:  
  - **Traslación**: botones **+X/–X, +Y/–Y, +Z/–Z**.  
  - **Rotación**: **+Rz/–Rz** (para SCARA T3). **Rx/Ry** no afectan al T3. :contentReference[oaicite:8]{index=8}  
- Para **enseñar puntos**, cambie a la pestaña Teach y almacene la posición actual (P0, P1, …) según su flujo. :contentReference[oaicite:9]{index=9}

---

## 2) Jog con **Teach Pendant** (TP2/TP3) — opcional

> Si dispone de la caja de mando, puede hacer Jog sin PC.

1. Conexión y compatibilidad: TP2/TP3 es compatible con controladores RC+ 7.x para serie T. :contentReference[oaicite:10]{index=10}  
2. **Habilitar motores** y seleccionar **Jog & Teach** en el Pendant. Puede usar **Free Joints** para mover **a mano** un eje (desenergizado) si lo requiere. :contentReference[oaicite:11]{index=11}  
3. Ajuste **Jog Distance**/**Speed** desde el pendant (incremento y velocidad de jog). :contentReference[oaicite:12]{index=12}  
4. Seleccione **Joint** o **Cartesian** (World/Tool/Local) y use las teclas de **X/Y/Z** o **J1–J4** para traslaciones/rotaciones como en RC+. :contentReference[oaicite:13]{index=13}

---

## 3) Guía rápida de **qué usar y cuándo**
- **Posicionar seguro/rápido** (evitar singularidades): **Joint** → mueva J1/J2 para orientar, **J3** para altura, **J4** para orientación final. :contentReference[oaicite:14]{index=14}  
- **Alinear con la pieza/herramental**: **Tool** → traslade **X/Y/Z** y gire **Rz** respecto al TCP (herramienta). :contentReference[oaicite:15]{index=15}  
- **Mover en el mundo de la celda**: **World** → traslade **X/Y/Z** respecto a la base. :contentReference[oaicite:16]{index=16}  


## Referencias
- **EPSON RC+ — Jog modes, Speed, Jog Distance** (World/Tool/Local/Joint/ECP; Step/Continuous). :contentReference[oaicite:20]{index=20}  
- **Jog & Teach (interfaz en español) / Teach points.** :contentReference[oaicite:21]{index=21}  
- **Motor On / Go Pulse** (llevar a pulso 0). :contentReference[oaicite:22]{index=22}  
- **Teach Pendant TP2/TP3** (compatibilidad, Jog Distance en pendant, Free Joints). :contentReference[oaicite:23]{index=23}  
- **Seguridad y resguardos (T-Series).** :contentReference[oaicite:24]{index=24}


