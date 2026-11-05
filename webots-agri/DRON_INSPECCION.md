# Mundo de inspección aérea: Husky + dron

Desde la raíz del repositorio:

```bash
./apple_dron.bash
```

Abre `webots-agri/worlds/dos_fields_dron.wbt` (el huerto de dos campos con el Husky portador y el dron) y arranca las capas sensoriomotora y cognitiva. `--help`, `--dry-run` y `--stop` funcionan igual que en `apple_field.bash`. No lanzar dos mundos a la vez.

## Qué hace (2026-11-05)

| Pieza | Qué hace |
|---|---|
| `webots-drone-bridge/` (`drone_bridge`) | Controlador *extern* del dron. Publica la **cámara RGB en DDS** (`rc/drone/rgb`, dominio 7, `ImageFrame` RGB8, igual que la ZED), sirve el **descriptor** (`MediaPlaneDDS`, puerto 12110) y la **pose** (`FullPoseEstimation`, puerto 12111). Es la **física de ArduCopter SITL** y traduce el mando a órdenes **MAVLink** para él. |
| ArduCopter SITL (`~/robocomp/ardupilot`, `scripts/run_sitl.sh`) | El autopiloto real de ArduPilot (4.7.2) ejecutándose en el PC: estima la posición (EKF3 con GPS, IMU, barómetro y brújula simulados) y vuela el dron. Lo arranca `sub_husky.toml` (componente `drone_sitl`) con la misma condición que el dron. |
| `robot_concept` (`src/drone_link.*`) | Si `drone_bridge` responde, cuelga del cuerpo del robot `body ──RT──▶ dron ──RT──▶ camera`. El nodo `camera` lleva el `media_descriptor` (topic DDS) y los intrínsecos (`cam_rgb_width/height`, `cam_fov`, focales). Si deja de responder 3 s, borra los nodos. |
| Mando Xbox (`python_xbox_controller`) | **A** = controlar el dron, **B** = volver al robot. El objetivo viaja en `TData.id` (`"drone"` / `"robot"`). |
| Puente del Husky (`webots-bridge`) | Con el control en el dron, para el robot e ignora los sticks hasta pulsar B. |
| Lanzadores | Exportan `WEBOTS_WORLD`; los componentes `drone` y `drone_sitl` de `sub_husky.toml` llevan `when = …` y **solo arrancan si el mundo contiene un `InspectionDrone`**. Sin dron, todo corre solo con el robot, como antes. |
| Mundo `dos_fields_dron.wbt` | Física a **2 ms** (los demás mundos siguen a 8) y el dron con `externalPropulsion TRUE`. Lo exige el autopiloto; ver `webots-drone-bridge/src/control_dron.md`. |

### Mando en modo dron (estilo "modo 2" de emisora)

| Control | Acción |
|---|---|
| **A** | El mando pasa a controlar el dron. |
| Stick izquierdo ↑ con el dron posado | **Despegue**: ArduCopter pasa a GUIDED, arma y sube solo a 1 m (`Pilot.TakeoffAltitude`). Hay que esperar ~50 s tras lanzar a que el EKF use el GPS; si el autopiloto rechaza el armado, lo dice en el log (`[ardupilot] PreArm: …`) y se reintenta mientras se mantenga el stick. |
| Stick izquierdo ↑/↓ en vuelo | Subir / bajar (máx. 1 m/s). |
| Stick izquierdo ←/→ | Girar (máx. 1 rad/s). |
| Stick derecho ↑/↓ | Avanzar / retroceder (máx. 2 m/s). |
| Stick derecho ←/→ | Desplazarse de lado. |
| Soltar los sticks | Se queda quieto: el autopiloto mantiene la posición con su GPS/EKF (deriva típica ~10–20 cm). |
| Stick izquierdo ↓ sostenido cerca del suelo | Por debajo de 0,5 m sobre el punto de despegue (`Pilot.LandAltitude`) pasa a **LAND**: el autopiloto baja, detecta el contacto y **desarma solo**. Subir el stick durante el LAND lo cancela. |
| **LB / RB** | Cámara frontal / cámara hacia abajo (también `F` / `D` en el teclado de Webots). |
| **B** | Vuelve el control al robot. El dron, si está volando, se queda quieto donde está. |

El aterrizaje es "a ciegas" sobre la posición GPS: todavía no hay aterrizaje de precisión sobre la plataforma (fase 3 de `control_dron.md`). Si el Husky se ha movido, el dron aterriza en el suelo.

### Grafo

```
husky (robot) ── body ──RT──▶ dron (vehicle) ──RT──▶ camera (camera)
```

Marcos con la convención del robot: **+Y adelante / eje óptico, +X derecha, +Z arriba** (como `zed`). `body→dron` es la pose del dron respecto al Husky; `dron→camera`, la de la cámara respecto al dron (incluye la inclinación de la cámara). Ambas con marca de tiempo. En simulación son la pose real del Supervisor de Webots (`source = webots-supervisor`).

Un consumidor de la imagen usa el patrón de siempre: `rc::media::make_image_subscriber_from_graph(G, "camera", "rgb")`.

## Hardware del dron

- `InspectionDrone.proto`: derivado del Mavic2Pro oficial de Cyberbotics R2025a.
- Campo `scale` (por defecto 0.5, tamaño tipo DJI Mini; 1 = Mavic 2 Pro). Geometría, articulaciones y colisiones ×s; **todas** las masas ×s² (0,23 kg a escala 0,5); empuje ×s³; par de hélice ×s⁴. Escalar solo algunas masas adelantaba el centro de masas ~3 cm respecto al empuje y el dron volcaba al despegar.
- Resistencia aerodinámica en el propio dron (`damping` lineal y angular 0,5), la misma que el mundo oficial del Mavic pone a todo el mundo. Sin ella las oscilaciones de actitud crecen.
- Cámara RGB 1280 × 720, FOV horizontal 60°. Solo `camera pitch` es móvil (0 = frontal, π/2 = abajo); arranca mirando abajo.
- GPS (0,05 m, correlación 0,9), IMU, giróscopo y brújula.

## Control de vuelo

**Actual: ArduCopter SITL** (`Flight.Backend = "ardupilot"` en `webots-drone-bridge/etc/config.toml`). Diseño, problemas resueltos y medidas: `webots-drone-bridge/src/control_dron.md`. Prueba sin interfaz: `webots-drone-bridge/scripts/sitl_selftest.sh`.

### Estabilizador propio anterior (`flight_controller.h`, `Flight.Backend = "internal"`)

Se conserva como alternativa; necesita el mundo a 8 ms y `externalPropulsion FALSE`. Consigna de velocidad → inclinación → actitud → mezcla de rotores (la de Cyberbotics). Altura con PID e integrador anti-saturación. Valores medidos en Webots:

- Equilibrio: 97 rad/s (el Mavic oficial, 68,6 rad/s; ×√(1/s)).
- Ganancias de actitud del ejemplo oficial `mavic2pro.c` (alabeo 50, cabeceo 30, velocidad angular 1).
- Los rotores simulados aceleran como mucho ~45 rad/s por segundo: el empuje común se limita a 20 rad/s² para dejar siempre margen a la corrección de actitud, y al seleccionar el dron se ponen al ralentí (60 rad/s) para que despegue enseguida.
- Prueba sin interfaz: `Test.Script = "hover"` en `etc/config.toml` hace un vuelo programado (subir, avanzar, girar, lateral, bajar, aterrizar). Resultado: altura estable ±5 cm, avance a 0,76 m/s, aterrizaje y desarmado correctos.

## Plataforma

- 0,40 × 0,40 m, apoyada en la placa superior, centrada en (0, 0,15, 0,383) del Husky; cara superior a 0,395 m. `InspectionDeck.proto` es paramétrico (`size`, `markerSize`, `markerOffset`, `postHeight`).
- Motivo: la plataforma alta anterior (0,70 m a 0,66 m) y el Mavic tapaban al LiDAR el suelo de delante entre 0,5 y 6 m. Medido: impactos en plataforma+dron 2989 → 641 por barrido; rayos bajos del sector frontal que pasan de 1 m: 25 % → 67 %, igual que el Husky sin plataforma.
- ArUco `DICT_4X4_50`, ID 23; lado negro **0,17 m**; centro (0, 0,23, 0,396) respecto al Husky.
- El dron no va anclado: pensado para conducir con cuidado o con el robot parado.

## Pendiente

- La pose del dron es la verdad del simulador. En el robot real habrá que estimarla (GPS/IMU del dron, el ArUco, o el cruce semántico de copas con los nodos `plant`).
- El dron no evita obstáculos: vuela donde lo lleva el piloto.

## Procedencia

Modelo y meshes derivados de https://github.com/cyberbotics/webots/tree/R2025a/projects/robots/dji/mavic. Se conserva la atribución y la referencia a la licencia de activos de Webots en `InspectionDrone.proto`. El marcador se genera de forma determinista con OpenCV.
