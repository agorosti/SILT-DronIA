# yolo_dataset/

Dataset de detección de objetos en formato YOLO para inspección de
defectos en paneles fotovoltaicos — 460 imágenes, 5 sitios,
5 clases (`dirt`, `bird_dropping`, `crack`, `delamination`,
`thermal_problem`). Generado por
[`../tools/build_yolo_dataset.py`](../tools/build_yolo_dataset.py) a
partir del generador procedural de parques solares de este proyecto, más
una captura con cámara de dron y un etiquetado automático.

Esta carpeta contiene únicamente los datos entrenables (`data.yaml`,
`images/`, `labels/`, con split train/val/test) y este README breve, para
poder subirla tal cual a Colab/Roboflow/etc. sin nada que quitar. Para la
explicación completa — diseño de las clases, cómo se calcularon las
etiquetas, imágenes térmicas, limitaciones conocidas, comandos de
entrenamiento — ver [`../docs/YOLO_DATASET.md`](../docs/YOLO_DATASET.md).

Una copia completa de esta carpeta vive también en
`yolo_sim_training/yolo_dataset/` — es lo que ese repo usa para entrenar
y evaluar, como copia propia (no por referencia), para poder ejecutarse
en otra máquina sin depender de tener este proyecto presente. Si
regeneras este dataset, actualiza también esa copia (ver
[README.md](../../yolo_sim_training/README.md), sección 0).

Para regenerar esta carpeta desde cero:

```bash
python3 tools/build_yolo_dataset.py
```
