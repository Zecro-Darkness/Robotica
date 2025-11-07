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

\* El folleto del MH6 no siempre detalla el tipo de montaje, pero en la práctica el MH6 se ofrece con opciones flexibles similares a sus pares.

---

## Fuentes
- **Motoman MH6**: especificaciones (ejes, carga, alcance, repetibilidad, peso) en RoboDK. Velocidades por eje y rangos típicos documentados en hojas de datos de la serie MH6. :contentReference[oaicite:0]{index=0}  
- **ABB IRB 140**: hoja oficial (octubre 2021) con alcance, velocidades por eje, repetibilidad, variantes y montaje. :contentReference[oaicite:1]{index=1}  
- **EPSON T3-401S**: ficha de la serie T3 (400 mm, 3 kg, repetibilidad, ciclo estándar, carreras y límites de ejes) y hoja T3-B actualizada (0.52 s). :contentReference[oaicite:2]{index=2}

