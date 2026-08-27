# Hoja de ruta

Este proyecto entrega, como una sola pieza, el entorno de simulación
completo: un parque fotovoltaico fotorrealista generado proceduralmente,
con defectos aleatorizados, anotaciones de referencia (ground truth), vuelo
real bajo ArduPilot SITL, y una cámara térmica simulada — todo ello ya
entregado, sin partes pendientes de una entrega futura.

---

## Qué incluye

**Entorno.** Parque procedural, síntesis de defectos, referencia
(ground truth), materiales con canal térmico co-registrado.

**Vuelo y control.**

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
- Grabación de transectos autónomos con superposición de telemetría,
  incluida una cámara térmica simulada (`--thermal` en `flight_video.py`)
- `capture.py --fly` interpola trayectorias multi-waypoint con
  aceleración/deceleración suave (ease-in/ease-out), redondeo de la
  esquina en cada waypoint intermedio, e inclinación (banking)
  proporcional al giro — para que el vídeo se lea como pilotado en vez de
  topografiado. Activado por defecto; `--no-ease`/`--corner-radius
  0`/`--bank-deg 0` lo desactivan si hace falta el comportamiento
  anterior exacto

Esta es la pieza que conecta el entorno con el detector: en lugar de
imágenes pre-renderizadas, el modelo ve lo que vería la cámara de un dron,
en tiempo real, mientras vuela.

Puntos conocidos que quedan pendientes: en directo, la cámara en nadir
entrega ~23 Hz sobre el topic ROS 2 en lugar de los 30 Hz a los que está
configurado el sensor (`update_rate` en `models/x500_rgb/model.sdf`, fijado
a 30 porque son los 1920×1080 @ 30 fps reales de la Raspberry Pi Camera
Module 3 que modela — ese valor ya es el correcto, no es un límite
arbitrario del código y no hace falta tocarlo). No es un cuello de botella
de GPU ni de renderizado, que sí sostiene los 30 Hz sin problema: el coste
está en sacar cada fotograma del contexto de render y transportarlo hasta
el topic ROS 2 (`image_bridge`), un paso que no se acelera con más
potencia gráfica — por eso se observa el mismo ~23 Hz tanto en el hardware
modesto de desarrollo (sin GPU discreta, ver "Rendimiento" en README.md)
como en el hardware objetivo con GPU discreta potente (RTX 5070,
ver [INSTRUCTIONS.md, sección
10](../INSTRUCTIONS.md#10-si-algo-no-funciona)). No afecta a la grabación
de vídeos ni a la generación del dataset, que no dependen de ir a tiempo
real: `flight_video.py` y `capture_dataset.py` muestrean el topic de
gz-transport directamente y esperan a que cada fotograma esté listo, así
que capturan los 30 fps completos aunque tarden más que el tiempo real en
hacerlo. Y la
teleoperación está verificada en banco contra entrada sintética, no
probada en vuelo con un mando físico.

**Imagen térmica.** Cada textura de panel se renderiza con un canal de
temperatura co-registrado — las grietas y la delaminación llevan una firma
de calor en los mismos píxeles que su daño visible. `flight_video.py
--thermal` usa ese canal para hacer que la señal de nadir grabada muestre
una cámara térmica simulada (falso color) en lugar de luz visible, sin
tocar la vista de seguimiento exterior. Ver [MANUAL.md, sección
3.4](MANUAL.md#34-el-canal-térmico-cómo-la-cámara-térmica-reutiliza-los-mismos-recursos)
para el detalle técnico, y [RUNME.md](../RUNME.md) para los comandos.

---

## Mejoras opcionales de realismo visual

Más allá de lo ya entregado, el entorno se puede llevar más lejos hacia un
realismo cinematográfico para demostraciones y figuras de publicación:

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

- La cámara en nadir en directo entrega ~23 Hz en vez de los 30 Hz a los
  que está configurada (`update_rate` en `models/x500_rgb/model.sdf` — 30
  Hz es el valor correcto, coincide con la cámara real que modela; no es
  un límite del código). No es un cuello de botella de GPU: el coste real
  está en la copia de cada fotograma y su transporte hasta el topic ROS 2,
  no en el renderizado, que sí sostiene los 30 Hz sin esfuerzo — por eso
  el mismo ~23 Hz se observa igual con gráficos integrados que con una GPU
  discreta potente (confirmado también en el hardware objetivo, una RTX
  5070; ver INSTRUCTIONS.md §10). Una copia cero o una codificación en el
  lado de la GPU cerraría esa brecha para el uso en directo; no hace falta
  para grabación ni generación de dataset, porque
  ninguna de las dos depende de ir a tiempo real (ver "Vuelo y control"
  más arriba).
- El encuadre de la cámara de seguimiento (chase camera) en el grabador de
  vuelo es fijo respecto a la aeronave. Un seguimiento con gimbal o
  consciente de la trayectoria daría resultados más cinematográficos para
  metraje de presentación.
- **Hecho, con una salvedad conocida.** `tools/build_yolo_dataset.py` fija
  en `SITE_RECIPES` una receta explícita de pesos de defecto
  (`--w-dirt`/`--w-crack`/etc.) para `site_h`/`site_i`/`site_j` — ya no
  son "pesos reconstruidos sobre la marcha", son constantes registradas,
  así que regenerar estos tres sitios a partir de ahora es determinista y
  reproducible byte a byte dado el mismo seed. La salvedad: esos pesos se
  reconstruyeron proporcionalmente a partir de los recuentos de
  `defects.json` (nunca se registró la línea de comandos original), y esa
  reconstrucción es aproximada, no una inversión calibrada del generador
  — contrastada contra `site_g` (el único sitio con pesos originales
  confirmados), el mismo método se desvía hasta ~43% relativo en la clase
  de menor recuento, aunque un contraste real sobre `site_h` quedó dentro
  de un ~5%. Detalle completo en el docstring de `build_yolo_dataset.py`
  y en "Reproducción" de [YOLO_DATASET.md](YOLO_DATASET.md#reproducción).
  Esto no es recuperable — las imágenes originales de esos tres sitios no
  se pueden regenerar píxel a píxel — pero tampoco es ya un cabo suelto:
  la receta registrada es la que cuenta de aquí en adelante. Un ajuste
  más fino (buscar pesos por prueba-error contra el generador real en vez
  de estimarlos proporcionalmente) es posible pero caro — cada intento
  tarda varios minutos — y no se ha hecho.
- **Hecho y confirmado en vuelo real (27/08).** `flight_video.py`
  incorpora un modo `--route` que porta la lógica de ruta de
  `autonomous_flight.py` (lectura de mesas reales del `.sdf`, recorrido
  en zigzag mesa a mesa, navegación por posición GPS absoluta vía
  `set_position_target_global_int_send`/`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`)
  al propio pipeline de grabación, en vez de mantener las dos
  herramientas separadas. Al volar por posición absoluta en lugar de
  velocidad en el frame del cuerpo, el rumbo de spawn deja de importar —
  soluciona de raíz el problema del punto siguiente, no sólo lo
  compensa como hacía `--yaw-deg`. `HOME_LAT`/`HOME_LON` son las mismas
  constantes de `autonomous_flight.py`, contrastadas esta sesión contra
  un `sitl.log` real ("Home: -35.363262 149.165237").

  Validado en vuelo real contra Gazebo/SITL tras arreglar un bug
  encontrado en las primeras pruebas: el bucle de `fly_route()` (y el
  de `fly()`, que comparte el mismo patrón) hacía tres llamadas
  `recv_match(type=X, blocking=False)` separadas por vuelta, una por
  cada tipo de mensaje esperado (`HEARTBEAT`, `STATUSTEXT`,
  `GLOBAL_POSITION_INT`). Cada una de esas llamadas, en modo no
  bloqueante, solo mira un mensaje de la cola de MAVLink y lo descarta
  si no es del tipo que busca — así que con tres tipos compitiendo cada
  vuelta, un `GLOBAL_POSITION_INT` que llegaba justo cuando le tocaba
  el turno a la llamada de `HEARTBEAT` se perdía sin que la llamada que
  sí lo esperaba llegara a verlo nunca. El síntoma en las primeras
  pruebas era exactamente ese: el dron armaba, despegaba y alcanzaba la
  primera mesa bien, pero después se quedaba sin ninguna posición nueva
  durante el resto del vuelo (confirmado con un contador de "hace
  cuánto llegó el último dato de posición", que se quedaba en varias
  decenas de segundos sin actualizarse). Arreglado sustituyendo las
  tres llamadas por tipo por una única llamada sin filtro que vacía la
  cola entera cada vuelta y reparte cada mensaje según su tipo real,
  sin perder ninguno. Tras el arreglo, un vuelo de 60 s sobre `site_j`
  alcanzó 23 waypoints (mesa a mesa) con 0 timeouts y sin ninguna
  desviación de rumbo. Probar con:
  `--route --route-tolerance 1.0 --route-waypoint-timeout 25` sobre un
  mundo ya generado.
- **Encontrado hoy (27/08) — resuelto arriba con `--route`.** El rumbo de spawn fijo que usa
  `flight_video.py` (antes `math.pi/2` sin más, documentado como
  "alineado con las filas") no es fiable en todos los mundos: al grabar
  sobre un `site_j` recién regenerado, el dron voló en línea recta lejos
  de las filas en vez de a lo largo de ellas, con altitud y velocidad
  estables (no fue un fallo del control de vuelo) — confirma lo que ya
  avisaba el propio docstring del fichero ("ArduPilot's NED axes and
  Gazebo's world axes differ by a rotation that's easy to get wrong"). Se
  ha añadido `--yaw-deg` para poder probar otros rumbos a mano sin tocar
  el resto del script, pero es un apaño, no una solución: lo correcto es
  derivar el rumbo real (o volar por posición absoluta, como
  `autonomous_flight.py`) en vez de asumir un valor fijo por mundo — ata
  con el punto anterior.
- **Encontrado hoy (27/08), segundo caso.** En los vuelos de 60 s grabados
  para la demo de inferencia (`site_j_seed1101_flight_60s_tfm.mp4` y su
  contraparte `_thermal`), el dron sale del recinto vallado cerca del
  final de la grabación: en el vídeo RGB el contenido es bueno hasta
  ~40 s y ya está fuera de la valla en el 41 s; en el térmico se degrada
  antes, buen contenido hasta ~39 s. En ambos casos la lectura de `ALT`
  del HUD se mantiene en valores plausibles (5-8 m) mientras el recuadro
  de nadir muestra solo cielo o césped, lo que apunta a una excursión de
  actitud (cabeceo/alabeo) más que a un fallo de altitud -- ningún
  failsafe ni cambio de modo se registra en ese tramo. Es probablemente
  la misma inestabilidad de control no determinista que motivó
  `--yaw-deg` más arriba, esta vez manifestada al final de un vuelo por
  lo demás bueno en vez de al principio. Solución aplicada: recortar
  ambos vídeos al segundo seguro (40 s / 39 s) en vez de perseguir la
  causa raíz -- no es una corrección, es un descarte del tramo malo.

  **Actualización (27/08): probablemente resuelto, sin confirmación
  retrospectiva.** El síntoma —buen comportamiento sostenido seguido de
  una degradación sin causa aparente, sin failsafe ni cambio de modo, en
  un vuelo por lo demás correcto— encaja con el bug de inanición de
  mensajes MAVLink descrito en la entrada de más arriba
  (`recv_match(type=X, blocking=False)` perdiendo `GLOBAL_POSITION_INT`
  cuando le tocaba el turno a otro tipo): explicaría exactamente una
  deriva de actitud no detectada a partir de cierto punto del vuelo, sin
  que ningún failsafe la vea venir. Tras el arreglo, los vuelos de 120 s
  grabados para la demo de inferencia (RGB y térmico, ver "Imagen
  térmica" más arriba y [RUNME.md](../RUNME.md)/[MANUAL.md](MANUAL.md))
  completaron su recorrido sin salirse del recinto ni degradarse — el
  síntoma no se ha vuelto a reproducir. No se ha confirmado
  retrospectivamente re-lanzando el escenario exacto contra el código
  viejo, pero el mecanismo encaja y no ha vuelto a aparecer en ningún
  vuelo posterior al arreglo.
- **Encontrado y arreglado (27/08).** `flight_video.py --nadir-out --thermal`
  escribía la señal de nadir en crudo sin el paso de falso color:
  `composite()` aplicaba la transformación (escala de grises → contraste
  `THERMAL_LOW`/`THERMAL_HIGH` → `cv2.COLORMAP_INFERNO`) únicamente a su
  propia copia reducida para el recuadro incrustado, nunca al fotograma
  que escribía `--nadir-out`. El síntoma no era visible en la vista
  compuesta (el recuadro salía bien) — solo en el vídeo de nadir en
  crudo, que salía en escala de grises/RGB sin colorear. Se descubrió al
  revisar fotogramas de una inferencia sobre ese vídeo: las cifras de
  detección eran plausibles en aislamiento (51,6% de fotogramas con
  detección) pero las clases detectadas no tenían sentido para un canal
  térmico (`dirt`/`crack`/`bird_dropping`/`delamination` en vez de solo
  `thermal_problem`) — el modelo, entrenado sobre térmico ya coloreado,
  estaba viendo un fotograma que no se parecía a nada de su
  entrenamiento. Arreglado extrayendo la transformación a una función
  `_thermalize()` que ahora usan tanto `composite()` como el escritor de
  `--nadir-out`. Tras el arreglo, la inferencia sobre el mismo vuelo
  vuelve a detectar solo `thermal_problem` (77,5% de fotogramas, ver
  VALIDACION_SIMULACION.docx en yolo_sim_training).
