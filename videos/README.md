# Vídeos

Vídeos generados con `flight_video.py` y `capture.py --fly` (ver
[RUNME.md, sección 2](../RUNME.md#2-generar-un-vídeo-aparte)). Son
artefactos renderizados, no código — se pueden borrar y regenerar en
cualquier momento con los comandos de RUNME.md; nada del proyecto depende
de que sigan aquí.

| Fichero | Notas |
|---|---|
| `inspection_flight.mp4` | primer vuelo de demo grabado con `flight_video.py`, spawn `13.0,-14,0.13`, RGB |
| `preview_flight_76s.mp4` | vista previa general, 76 s |

Los vídeos usados para la demo de inferencia de `yolo_sim_training`
(site `j`, seed 1101, recorrido de 120 s con `--route`, RGB y térmico) se
grabaron aquí y se copiaron a `yolo_sim_training/videos/` para esa demo —
no viven en esta carpeta; catálogo y detalles en
[inference_demo/README.md](../../yolo_sim_training/inference_demo/README.md)
del otro repo. Vídeos anteriores de sitios `g`/`h`/`i` y versiones previas
del vuelo de `j` (45-60 s, sin `--route`) se grabaron durante el
desarrollo y se han borrado tras quedar superados por el recorrido de
120 s.

El nombre de cada fichero sigue el patrón
`<contexto>_flight_<duración>[_thermal].mp4` — el sufijo `_thermal`
significa que se grabó con `--thermal` (canal térmico simulado, falso
color); su ausencia significa RGB normal.

Para la proporción de daño recomendada al generar el mundo de origen de
cada vídeo (realista vs. de prueba/demo), ver
[RUNME.md, sección 1.1](../RUNME.md#11-cuánta-proporción-de-daño-usar).
