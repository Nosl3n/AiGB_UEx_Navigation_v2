# drone_bridge

Controlador *extern* de Webots para el dron de inspección (`webots-agri/protos/InspectionDrone.proto`).

- **Cámara → DDS**: `rc/drone/rgb`, dominio 7, `ImageFrame` RGB8 (mismo transporte que la ZED, `active_inference/common/media_transport`).
- **MediaPlaneDDS** (puerto 12110): descriptor del plano + intrínsecos (`width`, `height`, `hfov_rad`).
- **FullPoseEstimation** (puerto 12111): `getFullPoseEuler()` = dron en el marco del Husky; `getFullPoseMatrix()` = cámara en el marco del dron. Convención del robot (+Y adelante, +X derecha, +Z arriba).
- **JoystickAdapter**: vuela con los mensajes cuyo `id` es `"drone"` (A en el mando; B devuelve el control al robot).
- Estabilizador: `src/flight_controller.h`.

`robot_concept` (`src/drone_link.*`) convierte todo esto en los nodos `dron` y `camera` del grafo, solo mientras este componente responde.

```bash
cmake -B build && make -C build -j8
WEBOTS_CONTROLLER_URL=ipc://1234/inspection_drone bin/drone_bridge etc/config.toml
```

Lo arranca `sub_husky.toml` solo si el mundo cargado tiene un `InspectionDrone`. Detalles y mando: `webots-agri/DRON_INSPECCION.md`.
