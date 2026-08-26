# Visuales

Salida renderizada de los mundos generados. Son artefactos de compilación,
mantenidos en el repositorio para que el resultado se pueda ver sin
instalar Gazebo.

| Fichero | Qué es |
|---|---|
| `flythrough_1000.mp4` | vídeo de vuelo de 8 s del parque de 1000 módulos, 1280x720 @ 30 fps |

Las imágenes fijas usadas en la documentación viven en
[`../docs/images/`](../docs/images).

## Reproducir el vídeo de vuelo

```bash
ros2 run solar_farm_gz generate_farm -- \
    --panels 1000 --tables-per-row 10 --variants 20 --seed 11 -o /tmp/f1000

ros2 run solar_farm_gz capture -- \
    --world /tmp/f1000/solar_farm.sdf --fly \
    --path "92,53,36,0,0.50,3.1416; 70,53,23,0,0.45,3.1416; \
            29,4,11,0,0.45,1.5708; 29,100,11,0,0.45,1.5708" \
    --frames 240 --fps 30 --fov 1.15 -o flythrough_1000.mp4
```

La ruta consta de cuatro waypoints: una aproximación de establecimiento
alta, un descenso, y luego un transecto de inspección a lo largo de una
fila a 11 m. La captura tarda unos 160 s en la máquina de referencia — el
mundo se carga una sola vez y la cámara se reposiciona entre fotogramas, en
lugar de relanzarse por cada fotograma.

`--seed 11` fija el parque de forma exacta: 100 mesas, 1000 módulos, 20%
dañado, 420 instancias de defecto.
