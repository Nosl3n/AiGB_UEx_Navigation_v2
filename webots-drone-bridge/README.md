# drone_bridge

Controlador *extern* de Webots para el dron de inspección (`webots-agri/protos/InspectionDrone.proto`).

- **Cámara → DDS**: `rc/drone/rgb`, dominio 7, `ImageFrame` RGB8 (mismo transporte que la ZED, `active_inference/common/media_transport`).
- **MediaPlaneDDS** (puerto 12110): descriptor del plano + intrínsecos (`width`, `height`, `hfov_rad`).
- **FullPoseEstimation** (puerto 12111): `getFullPoseEuler()` = dron en el marco del Husky; `getFullPoseMatrix()` = cámara en el marco del dron. Convención del robot (+Y adelante, +X derecha, +Z arriba).
- **JoystickAdapter**: vuela con los mensajes cuyo `id` es `"drone"` (A en el mando; B devuelve el control al robot).
- Vuelo, según `Flight.Backend`:
  - `"ardupilot"` (configurado): **ArduCopter SITL** por su interfaz JSON (`src/ardupilot_link.*`) y el mando traducido a MAVLink (`src/mavlink_pilot.*`). El mundo debe ir a 2 ms y el `InspectionDrone` con `externalPropulsion TRUE` (así está `dos_fields_dron.wbt`). Arranque: `scripts/run_sitl.sh` (lo lanza `sub_husky.toml`); prueba sin interfaz: `scripts/sitl_selftest.sh`. Detalles y medidas: `src/control_dron.md`.
  - `"internal"`: el estabilizador propio anterior, `src/flight_controller.h` (mundo a 8 ms, `externalPropulsion FALSE`).

Requisito de ArduPilot (una vez, fuera del repositorio):

```bash
git clone --depth 1 --branch Copter-4.7.2 https://github.com/ArduPilot/ardupilot.git ~/robocomp/ardupilot
cd ~/robocomp/ardupilot && git submodule update --init --depth 1 modules/mavlink modules/waf modules/DroneCAN/dronecan_dsdlc \
    modules/DroneCAN/libcanard modules/DroneCAN/pydronecan modules/DroneCAN/DSDL modules/gtest modules/littlefs \
    modules/lwip modules/Micro-XRCE-DDS-Client modules/Micro-CDR modules/gbenchmark
python3 -m venv ~/robocomp/ardupilot-venv && ~/robocomp/ardupilot-venv/bin/pip install empy==3.3.4 future pexpect pymavlink dronecan lxml numpy
PATH=~/robocomp/ardupilot-venv/bin:$PATH ./waf configure --board sitl && PATH=~/robocomp/ardupilot-venv/bin:$PATH ./waf copter -j8
```

`drone_bridge` toma las cabeceras MAVLink de esa compilación (`build/sitl/libraries/GCS_MAVLink/include`), así que ArduPilot se compila antes que él.

`robot_concept` (`src/drone_link.*`) convierte todo esto en los nodos `dron` y `camera` del grafo, solo mientras este componente responde.

```bash
cmake -B build && make -C build -j8
WEBOTS_CONTROLLER_URL=ipc://1234/inspection_drone bin/drone_bridge etc/config.toml
```

Lo arranca `sub_husky.toml` (junto con el SITL, componente `drone_sitl`) solo si el mundo cargado tiene un `InspectionDrone`. Detalles y mando: `webots-agri/DRON_INSPECCION.md`.
