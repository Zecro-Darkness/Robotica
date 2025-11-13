># Comparativo técnico: Motoman MH6 vs. ABB IRB 140 vs. EPSON T3-401S

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

  

En la caja **Mode** elija el sistema de coordenadas:  
- **Joint** (articulaciones): mueve **cada eje** por separado.
### 1.4 Ejecutar **traslaciones** y **rotaciones**
- En **Joint**: use los botones de **J1, J2, J3, J4** (+/–) para girar cada eje. **J3** es lineal (Z), **J4** es rotacional (herramienta).

- <img width="334" height="400" alt="image" src="https://github.com/user-attachments/assets/b1f9d8fa-0d32-44a6-9b47-3f8fd1fb9d14" />

- En **World/Tool/Local/ECP**:  
  - **Traslación**: botones **+X/–X, +Y/–Y, +Z/–Z**.
     
  - **Rotación**: +U/–U
    
    <img width="337" height="407" alt="image" src="https://github.com/user-attachments/assets/0ac9f029-b436-4e9c-ba3b-ddf93426efa3" />
