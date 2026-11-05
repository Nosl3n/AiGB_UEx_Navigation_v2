# Control de vuelo del dron: del simulador al dron real

**Documento de trabajo, 5 de noviembre de 2026.** Recoge el análisis de qué le falta al dron simulado para que lo desarrollado se pueda llevar a un dron real, y la elección del controlador de vuelo para el pipeline de inspección. Complementa a `drone.md` (opciones de plataforma real) y a `camera_dron.md` (cámara).

---

## 1. Pipeline de inspección

Premisa: **el Husky está parado** durante todo el vuelo. El dron despega de la plataforma del Husky y vuelve a ella.

| | Modo **planta** | Modo **localización** |
|---|---|---|
| Para qué | Obtener más información de una planta concreta | Que el robot se sitúe en el mapa semántico / layout del huerto |
| Antes del vuelo | El robot se acerca a su objetivo y lo marca en el mapa semántico | El robot decide que necesita una vista superior |
| Cámara | **Frontal** (0°), mirando a la planta | **Hacia abajo** (90°) |
| Movimiento | Sube **en vertical** hasta la altura aproximada de la planta, toma capturas, baja | Sube **en vertical** lo más alto que se pueda, toma capturas, baja |
| Altura | La de la planta (del nodo `plant` del mapa semántico) | Entrada de contexto: clima, límites fijados, normativa, batería |
| Final | Aterrizaje autónomo en la plataforma | Aterrizaje autónomo en la plataforma |

El único movimiento horizontal deseado es **ninguno**: el dron sube y baja sobre la plataforma. Además, en modo planta debe orientarse (guiñada) para que la cámara frontal apunte a la planta.

Cada captura debe ir acompañada de la **pose de la cámara**, su **calibración** y el **instante de captura**. Sin eso, las imágenes no se pueden proyectar en el mapa.

---

## 2. Qué es ideal hoy y qué no lo será en la realidad

Ordenado por impacto en el paso a la realidad.

### 2.1 El control de vuelo no es el de un dron real (crítico)

`flight_controller.h` es un estabilizador propio que manda directamente la velocidad de cada motor. **Un dron real nunca se controla así:** trae su propio autopiloto (DJI, PX4 o ArduPilot) y se le mandan consignas de alto nivel ("despega a 3 m", "sube a 0,5 m/s", "gimbal a −90°", "aterriza"). Todo lo ajustado en el estabilizador propio (ganancias, armado, desarmado al tocar) se perdería al pasar al dron real.

→ La simulación debe usar **la misma interfaz que el dron real**: el autopiloto real ejecutándose en simulación (SITL, *software in the loop*) o, en un DJI, una imitación de su SDK.

### 2.2 Se usa la verdad del simulador en lugar de sensores

| Qué | Hoy (simulación) | En la realidad |
|---|---|---|
| Altura y velocidad para volar | `self->getPosition()` / `getVelocity()`: exactas | Estimadas por el autopiloto (GNSS, barómetro, IMU, flujo óptico, telémetro) con ruido y deriva |
| Pose dron respecto al Husky (`body→dron`) | Supervisor de Webots, exacta | Estimada: ArUco de la plataforma, GNSS/RTK en ambos, visión |
| Pose de la cámara (`dron→camera`) | Supervisor, exacta | Ángulo reportado por el gimbal + calibración del soporte |
| IMU, giróscopo, brújula | Sin ruido ni sesgo | Ruido, sesgo, deriva, perturbaciones magnéticas (motores, estructura metálica del Husky) |

El GPS del proto tiene ruido modelado pero hoy no se usa para volar.

### 2.3 Enlace de radio instantáneo y perfecto

- **Vídeo:** hoy llega por DDS en ~1 ms. En la realidad viaja por radio, comprimido (H.264/H.265), con 20–250 ms de retraso según el sistema, cortes, bajadas de resolución y artefactos.
- **Mando y consignas:** latencia, pérdidas, alcance limitado.
- **Failsafe:** falta qué hace el dron si pierde el enlace (en un autopiloto real: aterrizar o volver al punto de despegue), y el sistema del robot debe contar con ello.

### 2.4 Física optimista

- **Sin viento ni ráfagas.** En campo abierto es el primer factor, y en modo planta (cerca de la copa) es el más peligroso.
- **Sin batería:** ni autonomía ni pérdida de empuje al bajar el voltaje. El campo `battery` del proto está vacío.
- **Sin efecto suelo** al aterrizar. Es justo lo que hace difícil posarse en la plataforma.
- **Plataforma quieta e indeformable.** El Husky real vibra y está inclinado en terreno irregular, aunque esté parado.

### 2.5 Cámara ideal

Sin distorsión, desenfoque de movimiento, *rolling shutter*, cambios de exposición, compresión ni latencia. El gimbal solo tiene cabeceo, no tiene vibraciones y se lee perfecto. Ver la sección 7.

### 2.6 Seguridad y normativa

No hay geovalla, altura máxima, regreso a casa ni evitación de obstáculos. En España (EASA/AESA, categoría abierta) el límite general es 120 m sobre el terreno, con contacto visual, registro de operador y restricciones cerca de personas. Un dron de menos de 250 g (C0) simplifica los requisitos.

---

## 3. Elección del controlador de vuelo

**Aclaración:** SITL no es una alternativa a PX4. *SITL* es una forma de ejecutar el firmware real del autopiloto en el PC, conectado a un simulador. Tanto **ArduPilot** como **PX4** tienen SITL. La elección real es **ArduPilot o PX4** (o un DJI con su SDK).

### 3.1 Qué exige el pipeline al controlador

1. Despegar, mantener posición, cambiar de altura y girar en el sitio con consignas desde un programa (sin piloto).
2. Mover el gimbal entre 0° y −90° por programa y leer su ángulo.
3. **Aterrizaje de precisión** sobre un marcador visual (ArUco) detectado por nuestro software.
4. Mantener la posición con precisión **cerca de la copa** (modo planta) aunque el GNSS derive.
5. Limitar la altura y reaccionar a la pérdida de enlace (geovalla, failsafe).
6. **Funcionar dentro de Webots**, que es donde vive todo el proyecto (mundo, Husky, sensores, agentes).

### 3.2 Comparación

| Criterio | **ArduPilot (ArduCopter)** | **PX4** | **DJI + Mobile SDK** |
|---|---|---|---|
| SITL con **Webots** | **Oficial**: controladores Python y C/C++ mantenidos por ArduPilot, con ejemplos de cuadricóptero (Iris, Crazyflie, con cámara) | No oficial: su simulador de referencia es Gazebo; con Webots habría que escribir y mantener el puente | No hay SITL; habría que imitar el SDK |
| Consignas por programa | Modo GUIDED vía MAVLink (`pymavlink`, MAVSDK) | Modo Offboard vía MAVLink o ROS 2 (uXRCE-DDS) | *Virtual stick* desde una app Android |
| Aterrizaje de precisión con marcador propio | Sí: `PLND_TYPE` = companion, mensajes `LANDING_TARGET`; además simula balizas en SITL (`SIM_PLD_*`) | Sí: `LANDING_TARGET` + `landing_target_estimator`; la simulación documentada es en Gazebo (IR-LOCK) | No como función abierta; habría que cerrar el lazo desde la app con la latencia del enlace |
| Mantener posición sin depender del GNSS | EKF3 con flujo óptico + telémetro, o posición externa (visión) | EKF2 con flujo óptico, o visión externa | El del producto, sin control fino |
| Gimbal por programa | MAVLink (Gimbal v2 / `DO_MOUNT_CONTROL`) | MAVLink (Gimbal v2) | Sí, por SDK |
| Hardware pequeño (< 250 g) | Muchas controladoras pequeñas (F4/F7/H7) compatibles | Soporte más centrado en placas Pixhawk | Sí (Mini 4 Pro), pero cerrado |
| Encaje con el proyecto | MAVLink desde un componente RoboComp | Si se usara ROS 2 su bus DDS sería una ventaja, pero el proyecto usa FastDDS propio, no ROS 2 | Cadena dron → mando → Android → red → RoboComp |

### 3.3 Decisión: **ArduPilot (ArduCopter) en SITL con Webots**

Razones, en orden de peso:

1. **Es el único con integración oficial con Webots.** Con PX4 habría que mantener un puente Webots↔PX4 propio o migrar el mundo a Gazebo. Las dos cosas son un proyecto en sí mismas.
2. **El pipeline es simple y es exactamente lo que ArduCopter hace de serie:** despegar (GUIDED + `TAKEOFF`), mantener posición, cambiar altura, girar en el sitio, apuntar el gimbal y aterrizar con precisión (`LAND` con `PLND`). No hace falta control de trayectorias ni investigación en control de bajo nivel.
3. **El aterrizaje de precisión sobre nuestro ArUco encaja directamente:** nuestro software detecta el marcador y envía `LANDING_TARGET`; el autopiloto se encarga del lazo de control, del descenso final y de qué hacer si pierde el marcador.
4. **El mismo código vale en el campo:** el componente que habla MAVLink con el SITL hablará MAVLink con el dron real sin cambios.
5. **Admite hardware pequeño**, coherente con el dron de ~250 g que se plantea transportar sobre el Husky.

**Cuándo convendría PX4:** si el proyecto pasara a ROS 2 y Gazebo, o si se quisiera investigar en control de vuelo (PX4 Offboard es más directo para eso). No es el caso de este pipeline.

**Por qué no DJI para este pipeline:** el aterrizaje autónomo de precisión sobre una plataforma propia exige cerrar el lazo con nuestra visión. Con DJI ese lazo pasa por el mando y una app Android, con latencia, y sin un modo abierto de aterrizaje sobre marcador. Sigue siendo válido si se renuncia al aterrizaje autónomo o si se usa una base fija con aterrizaje del propio producto. **Esta decisión condiciona el hardware:** el Mini 4 Pro de `drone.md` quedaría descartado para este pipeline, y el dron real sería uno con controladora ArduPilot.

---

## 4. Arquitectura propuesta

```
                Webots (mundo actual: huerto + Husky + plataforma)
                  │
   ┌──────────────┴──────────────────────────────────────────┐
   │ InspectionDrone (proto)                                  │
   │   controlador ardupilot_vehicle_controller (de ArduPilot)│◄── física ↔ SITL (UDP, paso a paso)
   │   cámara 130° + gimbal                                   │
   └──────────────┬──────────────────────────────────────────┘
                  │ (imagen, ángulo del gimbal)
                  ▼
   ArduCopter SITL  ◄──── MAVLink ────►  drone_bridge (RoboComp)
   (EKF, control,                          ├─ misión: modo planta / modo localización
    PLND, geovalla,                        ├─ cámara → DDS (con latencia y pérdidas simuladas)
    failsafe)                              ├─ ArUco → LANDING_TARGET (aterrizaje de precisión)
                                           ├─ pose ESTIMADA del dron y de la cámara → FullPoseEstimation
                                           └─ joystick → consignas (control manual de reserva)
                                                    │
                                                    ▼
                                    robot_concept / DroneLink → nodos `dron` y `camera` del grafo
```

Cambios respecto a hoy:

- `flight_controller.h` **desaparece**. El vuelo lo hace ArduCopter.
- `drone_bridge` pasa de estabilizador a **ejecutor de misiones + cliente MAVLink**.
- El mando sigue sirviendo, pero ahora envía consignas al autopiloto (como en un dron real) en vez de mover motores.
- La pose que llega al grafo es la **estimada** (EKF del autopiloto + ArUco). La verdad del Supervisor se conserva solo como **referencia de evaluación**, en un canal aparte, nunca para volar ni para el grafo.

### 4.1 Interfaz para el robot

Los agentes no deberían hablar MAVLink. `drone_bridge` debería ofrecer una interfaz de misión de alto nivel, por ejemplo:

- `inspectPlant(plant_id, altura_min, altura_max, n_capturas, rumbo)`
- `overview(altura_objetivo)`
- `abort()` (volver y aterrizar)
- estado: `en_plataforma | despegando | inspeccionando | aterrizando | fallo`

El agente que decide la misión (el robot) no necesita saber qué autopiloto hay debajo.

---

## 5. Los dos modos, paso a paso

### 5.1 Modo planta

1. El robot está parado junto a la planta, cuyo nodo `plant` tiene posición y altura en el mapa semántico.
2. `drone_bridge` calcula el **rumbo** hacia la planta (robot → planta en el mapa) y la altura de la copa.
3. GUIDED → armar → `TAKEOFF` a una altura segura sobre la plataforma.
4. Girar en el sitio hasta el rumbo de la planta; gimbal a 0° (frontal).
5. Subir por pasos entre la altura mínima y la máxima de la copa; en cada paso, estabilizar y capturar imagen + pose estimada + ángulo del gimbal + marca de tiempo.
6. Bajar hasta la altura de aproximación; gimbal a −90°; aterrizaje de precisión (5.3).

**Riesgo principal:** el dron sube **a ~1 m de la copa**. Un GNSS normal deriva 1–2 m, que es suficiente para tocar ramas. Para mantener la posición hace falta referencia local: **flujo óptico + telémetro hacia abajo**, o el ArUco de la plataforma mientras sea visible, o **GNSS RTK**. Es un requisito de hardware, no solo de software.

### 5.2 Modo localización

1. La altura objetivo es una **entrada**: `min(límite legal 120 m, geovalla fijada, límite por viento, reserva de batería, alcance del enlace)`.
2. Mejor aún: derivar la altura de lo que se necesita ver. Con una cámara de 130° de FOV horizontal, el ancho cubierto es `2·h·tan(65°) ≈ 4,3·h`. Para ver 60 m de huerto bastan ~14 m de altura (≈ 4,7 cm/píxel a 1280 px). **Subir lo más alto posible no siempre mejora el resultado**: pierde resolución y aumenta el viento.
3. GUIDED → armar → `TAKEOFF`; gimbal a −90° (abajo); subir en vertical hasta la altura objetivo.
4. Capturar varias imágenes estabilizadas con pose + marca de tiempo.
5. Bajar y aterrizaje de precisión.

### 5.3 Aterrizaje autónomo en la plataforma

1. El dron vuelve sobre la plataforma por la posición estimada (al subir y bajar en vertical apenas se ha alejado).
2. Gimbal a −90°; `drone_bridge` detecta el ArUco (`DICT_4X4_50`, ID 23, 0,17 m) en cada imagen y calcula su posición respecto a la cámara (necesita la calibración de la cámara y su pose respecto al dron).
3. Envía `LANDING_TARGET` al autopiloto con `PLND_TYPE` = companion; `LAND` hace el resto: centrarse, descender y desarmar al tocar.
4. Si se pierde el marcador: el autopiloto se detiene a reintentar (`PLND_STRICT`, `PLND_RET_MAX`); si no lo recupera, se aborta según la política fijada.
5. Ya no hace falta el "desarmado manual" de hoy (mantener el stick abajo 1,5 s): el autopiloto detecta el aterrizaje.

Detalle a resolver: el marcador mide 0,17 m. Con la cámara de 130° a 10 m de altura ocupa ~5 píxeles (a 3 m, ~17): demasiado pequeño para detectarlo. El aterrizaje empezará a usarlo por debajo de ~3–4 m. Un **marcador anidado** (uno grande y otro pequeño dentro) amplía ese rango.

---

## 6. Realismo a añadir en la simulación (más allá del autopiloto)

| Qué | Cómo |
|---|---|
| Ruido y sesgo de sensores | Parámetros `SIM_*` del SITL de ArduPilot (IMU, barómetro, GNSS, brújula) |
| Viento y ráfagas | `SIM_WIND_*` en SITL, o fuerzas en Webots |
| Batería | `SIM_BATT_*`: voltaje, capacidad, caída con la carga; failsafe de batería |
| Latencia y pérdidas del vídeo | En `drone_bridge`: retraso configurable, compresión, frames perdidos, cortes |
| Latencia del enlace de mando | Retraso y pérdidas en el canal MAVLink de pruebas |
| Efecto suelo / plataforma | Revisar la respuesta de ArduCopter al aterrizar en Webots; vibración/inclinación de la plataforma |
| Pose de la cámara | Desde el ángulo del gimbal (`camera pitch sensor`) + calibración, no desde el Supervisor |
| Pose dron↔Husky | Desde el EKF + ArUco, no desde el Supervisor |

Y un principio: **cada valor de ruido o latencia debe venir de una hoja de datos o de una medición del equipo elegido**, no de suposiciones. Hasta tener el hardware, mejor rangos conservadores declarados como tales.

---

## 7. La cámara (`camera_dron.md`)

La cámara de 130° de FOV propuesta en `camera_dron.md` es adecuada para los dos modos (copa completa a corta distancia; mucha superficie desde arriba). Antes de aplicarla en el proto, corregir:

1. **Cobertura:** a 1 m, 130° de FOV horizontal cubren `2·1·tan(65°) ≈ 4,3 m`, no 3,1 m. 3,1 m correspondería a ~115°.
2. **Campo del proto:** `lensRadialCoefficients` no es un campo de `Camera` en Webots. La distorsión se declara con un nodo `lens Lens { radialCoefficients [...] }`.
3. **Proyección:** con proyección plana, a 130° los bordes salen muy estirados, y una lente real tan abierta se parece más a un ojo de pez. Hay que decidir el modelo (plano con distorsión radial, o `projection "spherical"`) y que coincida con la calibración que se use en el robot.
4. **Intrínsecos en el grafo:** el nodo `camera` debe anunciar también los coeficientes de distorsión, no solo FOV y focal, para que `retina` y los demás desdistorsionen igual que en el real.
5. **Sincronización:** los sistemas de vídeo FPV (opción A de `camera_dron.md`) no transmiten marca de tiempo de captura ni pose. Para proyectar al mapa hay que asociar cada imagen con la pose del autopiloto en ese instante. Hay que medir la latencia del enlace o usar una cámara en un ordenador de a bordo (opción B) que marque los tiempos.

---

## 8. Plan por fases

1. **ArduCopter SITL + Webots con el dron actual:** sustituir `flight_controller.h` por el controlador de ArduPilot; despegar, mantener posición y aterrizar con `pymavlink` desde un script. Conservar la verdad del Supervisor solo para medir el error de la estimación.
2. **`drone_bridge` como cliente MAVLink:** mando → consignas GUIDED; pose estimada → `FullPoseEstimation` → grafo.
3. **Aterrizaje de precisión:** detección del ArUco + `LANDING_TARGET` + `PLND`. Medir la tasa de éxito y el error de posado con viento y ruido.
4. **Modos planta y localización** como misiones de alto nivel invocables desde el robot.
5. **Cámara de 130°** con distorsión, calibración e intrínsecos en el grafo; latencia y pérdidas del enlace de vídeo.
6. **Realismo:** viento, batería, failsafes, geovalla, todo con valores del hardware elegido.
7. **Hardware:** elegir controladora ArduPilot, sensores de posición local (flujo óptico + telémetro o RTK), enlace de vídeo y de telemetría, y validar en banco antes de volar.

---

## 9. Estado: fase 1 hecha (5 de noviembre de 2026)

**ArduCopter 4.7.2 en SITL vuela el dron en Webots.** Prueba reproducible: `scripts/sitl_selftest.sh` (despegue GUIDED a 3 m, vuelo estacionario, giro de 90°, LAND, desarmado automático).

### Qué hay

| Pieza | Qué hace |
|---|---|
| `~/robocomp/ardupilot` | ArduPilot `Copter-4.7.2` compilado para SITL (`build/sitl/bin/arducopter`). Fuera del repositorio. |
| `~/robocomp/ardupilot-venv` | Python con `pymavlink`, `empy 3.3.4`, `dronecan` (compilar ArduPilot y scripts MAVLink). |
| `src/ardupilot_link.*` | Lado físico de la interfaz JSON de ArduPilot: recibe los PWM, devuelve IMU, posición, velocidad y actitud. Lockstep: un intercambio por paso de Webots. Marcos: mundo Webots x = Norte, y = Oeste, z = Arriba; cuerpo FLU → NED/FRD con S = diag(1, −1, −1). |
| `drone_bridge` con `Flight.Backend = "ardupilot"` | La física del dron para el SITL. Cámara, pose y grafo siguen igual. |
| `InspectionDrone.proto` | Nuevo `Accelerometer` (el autopiloto necesita la fuerza específica) y campo `externalPropulsion`. |
| `etc/ardupilot/inspection_drone.parm` | Parámetros del dron, cada uno con su porqué. |
| `scripts/run_sitl.sh`, `scripts/test_flight.py` | Arrancar el SITL; vuelo de prueba por MAVLink (como una estación de tierra). |

### Lo que hubo que resolver (medido)

1. **Paso de física.** A 8 ms el autopiloto se niega a armar ("Gyro rate < loop rate × 1,8"; recibe una muestra de IMU por paso). A 2 ms llegan ~505 Hz de IMU → bucle de control a **250 Hz** (`SCHED_LOOP_RATE 250`). 400 Hz exigiría pasos de 1 ms.
2. **Los rotores de Webots tardan segundos en dar empuje.** Medido en el aire: 0,5 s después de pedir 1,3 × el peso no había empuje, y tardaba ~3,5 s. Con ese retraso ningún controlador estabiliza (oscilación creciente y vuelco). Solución: `externalPropulsion TRUE` en el proto (las hélices solo giran) y `drone_bridge` aplica el empuje `kT·ω²` y el par de reacción `−kQ·|ω|·ω` de cada rotor en su posición, con un retraso de primer orden de **30 ms** (`ArduPilot.MotorTimeConstant`, típico de un motor pequeño). Medido después: 63 % del empuje a los 30 ms.
3. **Sentido de giro de las hélices.** FL y RR giran antihorario visto desde arriba, FR y RL horario: es el marco `X_REV` de ArduPilot (`FRAME_TYPE 18`); servo 1..4 = trasero derecho, delantero derecho, trasero izquierdo, delantero izquierdo. Comprobado en lazo abierto (más empuje a la derecha → alabeo a la izquierda) y en vuelo (giro de guiñada en el sentido pedido).
4. **Ganancias.** Las de serie (para ~1,5 kg) dan oscilación de alabeo de 7°; reducidas (P 0,03, I 0,03, D 0,0008) dan 0,9–2,7°. **No están autoajustadas**: en el dron real, AUTOTUNE.

### Resultados

- Vuelo (`sitl_selftest.sh`): subida a 3 m en ~6 s; estacionario: altura 2,87–3,11 m, deriva horizontal ~0,2 m (GPS simulado del SITL), inclinación < 3°; giro de 90°; aterrizaje y desarmado automáticos.
- Coste del mundo a 2 ms con toda la capa sensoriomotora del Husky y el dron volando (sin agentes, sin interfaz): factor de tiempo real 0,93 → **0,89**; LiDAR 19,4 → 18,5 Hz, ZED 18,6 → 17,7 Hz, IMU 116 → 111 Hz (~5 %).
- La pose que llega al grafo sigue siendo la del Supervisor (fase 3).

### Fase 2 hecha: mando → ArduPilot y lanzador (5 de noviembre de 2026)

- `dos_fields_dron.wbt` a **2 ms** y con `externalPropulsion TRUE`; los demás mundos y lanzadores no cambian.
- `etc/config.toml`: `Flight.Backend = "ardupilot"`.
- `sub_husky.toml`: componente `drone_sitl` (`scripts/run_sitl.sh --wipe`), con la misma condición `when` que el dron.
- `src/mavlink_pilot.*`: el mando habla MAVLink con el autopiloto por SERIAL1 (tcp 5762; el 5760 queda libre para una estación de tierra o los scripts de misión): stick ↑ posado = GUIDED + armar + despegue a 1 m; sticks = velocidades en el marco del rumbo; soltar = mantener posición; ↓ sostenido por debajo de 0,5 m = LAND (aterriza y desarma el autopiloto). Mensajes del autopiloto en el log del dron (`[ardupilot] …`).
- Probado con el mando simulado: sin interfaz (despegue a 1,1 m; 3 s adelante = 3,04 m en línea recta; mantener ±10 cm; B; LAND y desarmado) y con `./apple_dron.bash` completo (despegue, vuelo, LAND, `--stop` limpio).

Detalles resueltos por el camino: el SITL esperaba a una estación de tierra en el 5760 antes de arrancar (`--serial0 tcp:0`); ignoraba SIGTERM una vez parada la física (el script lo convierte en SIGINT); los flujos `EXTENDED_SYS_STATE`/`GLOBAL_POSITION_INT` pedidos durante el arranque se perdían (se piden hasta que llegan) y, sin ellos, las consignas de velocidad cancelaban el despegue.

A vigilar: con el mundo a 2 ms, `--stop` ha tenido que forzar el cierre de Webots tras 15 s (el lanzador ya lo hace).

### Pendiente

- Fases 3–7 de la sección 8: aterrizaje de precisión (ArUco → `LANDING_TARGET`), modos planta/localización, pose estimada en el grafo, cámara de 130°, realismo (viento, batería, enlace).

---

## Referencias

- ArduPilot, [SITL con Webots (Python)](https://ardupilot.org/dev/docs/sitl-with-webots-python.html) y [SITL con Webots (C/C++)](https://ardupilot.org/dev/docs/sitl-with-webots-cpp.html).
- ArduPilot, [aterrizaje de precisión](https://ardupilot.org/copter/docs/precision-landing-with-irlock.html) y [protocolo MAVLink Landing Target](https://mavlink.io/en/services/landing_target.html).
- PX4, [precision landing](https://docs.px4.io/v1.13/en/advanced_features/precland) (simulación documentada en Gazebo).
- EASA, [categoría abierta](https://www.easa.europa.eu/en/the-agency/faqs/training-requirements-open-category).
