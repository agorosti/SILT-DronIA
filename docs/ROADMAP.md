# Hoja de ruta

La Fase 1 entrega el entorno de simulación: un parque fotovoltaico
fotorrealista y generado proceduralmente, con defectos aleatorizados,
anotaciones de referencia (ground truth) y un sistema de materiales
preparado para lo térmico.

La Fase 2 entrega el vuelo. Las fases restantes son extensiones opcionales,
y cada una se construye sobre lo existente sin rehacer ningún recurso — la
arquitectura se diseñó para que el trabajo posterior sea aditivo.

---

## Fase 1 — entorno *(entregada)*

Parque procedural, síntesis de defectos, referencia (ground truth),
materiales preparados para lo térmico.

---

## Fase 2 — vuelo y control *(entregada)*

- ArduPilot SITL integrado con Gazebo Harmonic; arma, despega y mantiene un
  vuelo estacionario a 8 m verificado contra la referencia de Gazebo
- Cuadricóptero clase Holybro X500 V2 ajustado al armazón físico real, con
  una Raspberry Pi Camera Module 3 en nadir con su campo de visión real
- Teleoperación con mando a través de canales RC de MAVLink
- Transmisión de cámara en directo por ROS 2 en `/x500_rgb/nadir`, lista
  para OpenCV o YOLO
- Ambas vistas de operador — órbita libre e inspección en nadir — se abren
  juntas al lanzar
- Infraestructura del emplazamiento: valla perimetral, camino de servicio en
  anillo, estaciones de inversores
- Cobertura de césped como opción intercambiable
- Grabación de transectos autónomos con superposición de telemetría

Esta es la pieza que conecta el entorno con el detector: en lugar de
imágenes pre-renderizadas, el modelo ve lo que vería la cámara de un dron,
en tiempo real, mientras vuela.

Puntos conocidos que quedan pendientes: la cámara en nadir entrega ~23 Hz en
directo en lugar de los 30 Hz configurados (limitado por la lectura de
imagen, no por la GPU — grabar por debajo del tiempo real recupera la tasa
completa), y la teleoperación está verificada en banco contra entrada
sintética, no probada en vuelo con un mando físico.

---

## Fase 3 — imagen térmica

Añadir la modalidad de detección térmica para la que los paneles ya están
preparados.

Cada textura de panel de la Fase 1 se renderizó con un canal de temperatura
co-registrado — las grietas y la delaminación ya llevan una firma de calor
en los mismos píxeles que su daño visible. La Fase 3 convierte esos datos
latentes en una cámara térmica funcional: un segundo sensor que ve el mapa
de temperatura, de modo que tu pipeline pueda fusionar la detección visible
y la térmica. **No hace falta reconstruir ningún recurso de panel** — ese
era precisamente el propósito de preparar el sistema de materiales en la
Fase 1.

---

## Mejoras opcionales de realismo visual

Independientemente de las fases anteriores, el entorno se puede llevar más
lejos hacia un realismo cinematográfico para demostraciones y figuras de
publicación:

- Infraestructura del emplazamiento — valla perimetral, estaciones de
  inversores y transformadores, caminos de acceso, una plataforma de
  subestación
- Relieve del terreno en los márgenes del campo y taludes perfilados
- Texturas PBR de mayor resolución y desgaste de los paneles (suciedad en
  los bordes, manchas de agua, envejecimiento)
- Cielo volumétrico, bruma atmosférica y presets de hora del día
- Arreglos más densos y grandes para planos generales amplios

Los parques solares industriales se asientan sobre terreno plano y
nivelado, así que el terreno plano actual es fiel al dominio real; este
pulido trata de valor de producción y efecto "wow" para presentaciones, no
de corrección.

---

## Notas pendientes

Pequeños puntos internos, registrados para completitud:

- El movimiento de cámara del vídeo de vuelo es lineal por tramos entre
  waypoints; añadir suavizado (easing) e inclinación en curvas haría que el
  metraje grabado se leyera como pilotado en lugar de topografiado. La
  grabación de vuelo de la Fase 2 no tiene esta característica — la pilota
  el controlador — pero la herramienta de captura de la Fase 1 sí.
- La tasa de fotogramas en directo de la cámara en nadir está limitada por
  la lectura de imagen y el transporte por fotograma, no por el
  renderizado. Una copia cero o una codificación en el lado de la GPU
  cerraría esa brecha; grabar por debajo del tiempo real la evita para el
  trabajo de datasets.
- El encuadre de la cámara de seguimiento (chase camera) en el grabador de
  vuelo es fijo respecto a la aeronave. Un seguimiento con gimbal o
  consciente de la trayectoria daría resultados más cinematográficos para
  metraje de presentación.
