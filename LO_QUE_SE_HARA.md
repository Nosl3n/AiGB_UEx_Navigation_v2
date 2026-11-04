# LO QUE SE HARÁ

Revisé el active_inference original (`~/robocomp/components/active_inference`) y el de este repo. Codex y Antigravity hicieron el primer barrido y yo comprobé las afirmaciones clave en el código. El sistema original no funciona exactamente como lo describes: varias de esas piezas existen, pero hoy están desactivadas.

## Cómo funciona en el original

### 1. Mapa inicial (`room_concept`)

- **Dos modos.** `MapMode = "given"` carga el polígono de la habitación y solo localiza; es el que está activo. `"estimate"` estima las paredes con el LiDAR a la vez que la pose.
- **Exploración.** Cuando el mapa está completo lo congela (`FreezeLayoutWhenPublishable`) y pasa de `Searching` a `Localizing`. Mientras tanto ofrece un affordance de exploración (`afford_room`) hacia los puntos con más ganancia de información.

### 2. Objetos y affordances

- **Nacimiento.** Las mesas, neveras, etc. nacen de las detecciones de la cámara: el nodo se crea antes de acercarse.
- **Acercamiento.** Después, cada objeto publica un affordance epistémico, tipo `Reach`, `Servo` u `Orient`, para ir a observarlo mejor y reducir su incertidumbre.
- **Elección.** El controller decide qué affordance ejecutar según la ganancia de información frente al coste de llegar. El protocolo de estados (`Offered`, `Executing`, `Completed`) está en `common/affordance_protocol`.
- **Búsqueda por prior: eliminada.** El modo de buscar objetos esperados (prior-scaffold) existió, pero se quitó (`table_concept/src/specificworker_lifecycle.cpp:45-48`). Los priors que quedan son de clase (tamaño, altura) y los del `kitchen_metaconcept`; no hay una lista de objetos que buscar.

### 3. Los objetos fijos corrigen la pose: esto sí está activo y es lo más valioso

- **Cómo funciona.** `ObjectAnchorSource` toma objetos validados y los mete como landmarks (`Se2LandmarkFactor`) en el optimizador de pose de `room_concept`.
- **Ancla fija para evitar el círculo.** Cuando el objeto ya es preciso, se congela un ancla. Si no, la posuló a partir del robot, acabaría "confirmando" la propia pose del robot.
- **Solo la nevera.** Ahora está limitado a la nevera, porque un ancla de mesa mal fijada provocaba que el robot se deslocalizara (`room_concept/etc/config.toml:1184-1192`).

### 4. Memoria a largo plazo (`ltsm_agent`)

- **Qué hace.** Archiva una habitación entera, con sus objetos, al pasar a otra, en un grafo aparte, y puede guardarla en disco.
- **Desactivado.** Tanto el archivado como el guardado vienen apagados por defecto (`Eviction.enabled`, `Memory.persist`).
- **No se reutiliza.** No hay ningún camino que devuelva esos objetos al grafo de trabajo para relocalizar al robot.

## Qué hay ya en este repo para las plantas

- **Lo que hay.** `trunk_concept` detecta troncos con la ZED y los confirma con el LiDAR (unos 3 cm de error). `row_metaconcept` deduce las filas y `vegetation_concept` las copas. Todos se colocan en el marco `field`.
- **No corrigen la pose.** Los troncos no corrigen la pose del robot: el EKF de `openfield_concept` solo usa ción va en sentido contrario: los troncos se colocan con la pose ya estimada.
- **No hay memoria.** Al arrancar se borran los nodos anteriores (`delete_stale_trunk_nodes`).
- **No hay exploración ni affordances.** Ningún agente del huerto publica affordances ni tiene un modo de exploración; solo está `run_route.py` con waypoints fijos.
