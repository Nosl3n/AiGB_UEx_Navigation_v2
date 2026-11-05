# Estado actual y trabajo pendiente: mapeo, exploración y memoria

**Revisión: 5 de noviembre de 2026.** Este documento actualiza el análisis anterior contrastándolo con el código y las configuraciones presentes en `AiGB_UEx_Navigation_v2` y en el original `~/robocomp/components/active_inference`. Se trata de una revisión estática: encontrar una función o una opción activada no demuestra por sí solo que funcione correctamente en todas las simulaciones o en el campo.

## Resumen

La idea sigue pendiente de completar. El proyecto ya **percibe plantas, infiere filas y propone observaciones para inspeccionar el campo**. Todavía falta convertir el resultado en un mapa de referencia persistente y utilizar ese mapa para corregir la pose o relocalizarse después de un reinicio.

| Capacidad | Estado observado |
|---|---|
| Detectar plantas y sus partes | Implementado en `plant_concept`; integra tronco y dosel en un individuo |
| Inferir filas | Implementado en `row_metaconcept`, a partir de plantas con tronco observado |
| Explorar el campo | Implementado y habilitado en la configuración revisada mediante `afford_openfield` |
| Pose del Husky | EKF con ruedas, giróscopo, GPS y yaw de IMU; las filas no se incorporan como correcciones en la interfaz revisada |
| Mapa semántico durante la ejecución | Las plantas, filas y sus relaciones se publican en el grafo DSR |
| Guardar y recuperar un mapa de referencia | Pendiente en el flujo agrícola revisado |
| Relocalizarse al reiniciar usando las filas | Pendiente |

## 1. Sistema original: `room_concept`

### 1.1. Mapa y localización

- **Modo `given`.** La configuración `room_concept/etc/config.toml` tiene `RoomShape.MapMode = "given"`: carga la geometría de la habitación y la emplea para localizarse.
- **Modo `estimate`.** El código puede estimar simultáneamente paredes y pose a partir del LiDAR. Parte de una hipótesis rectangular y actualiza un modelo de paredes y polígono. Este modo existe, pero no es el seleccionado en la configuración citada.
- **Congelación opcional.** `FreezeLayoutWhenPublishable = true` está configurado, pero solo tiene sentido cuando se aprende la geometría. Si se alcanza un polígono publicable y se cumplen las condiciones adicionales, se fija la geometría y el estado pasa de `Searching` a `Localizing`. Sin congelación, el estado de aprendizaje puede continuar. Que el modelo sea publicable es un criterio geométrico y de incertidumbre; no prueba por sí solo que se haya observado todo el entorno.
- **Exploración.** `room_concept` dispone del planificador y del affordance `afford_room` para ofrecer vistas informativas. Su ejecución depende de la configuración, de las condiciones de la escena y de que el controlador acepte la propuesta; no debe suponerse que cada ejecución explora autónomamente.

Fuentes: `room_concept/etc/config.toml`, `room_concept/src/room_concept.cpp`, `room_concept/src/room_concept.h`, `room_concept/src/epistemic_planner.cpp` y `room_concept/src/room_scene_graph.cpp`.

### 1.2. Objetos y affordances

- En el flujo revisado de `table_concept`, las observaciones de cámara permiten crear y asociar hipótesis de objetos; no es necesario que el robot llegue primero a una posición de observación mejor.
- El agente de mesa dispone de un affordance epistémico para proponer una vista que mejore su estimación. El controlador valora propuestas según ganancia y coste; el protocolo común incluye estados `Offered`, `Executing` y `Completed`. Los tipos concretos y la disponibilidad dependen del agente y de la configuración: no debe generalizarse que todos los objetos publiquen siempre todos los tipos `Reach`, `Servo` y `Orient`.
- El código de ciclo de vida de la mesa indica que el antiguo mecanismo `prior-scaffold` fue retirado. Existen otros priors y metaconceptos, pero esta revisión no establece exhaustivamente todos los mecanismos de búsqueda de todos los agentes.

Fuentes: `table_concept/src/specificworker_lifecycle.cpp`, `table_concept/src/specificworker.cpp`, `common/affordance_protocol/` y `controller/src/specificworker.cpp`.

### 1.3. Objetos como referencias para la pose

`room_concept` incorpora observaciones de objetos validados como restricciones de landmarks para la estimación. El código contempla fijar una referencia cuando alcanza el criterio de validación; así evita reutilizar continuamente una posición calculada a partir de la misma pose como si fuera evidencia nueva e independiente. Una fijación equivocada puede, sin embargo, introducir una referencia errónea.

`[ObjectAnchor].enable = true` en la configuración revisada. **La restricción «solo nevera» depende del escenario:** `Scenario.apartamento` selecciona `refrigerator`, mientras que `Scenario.waf` y `Scenario.beta` seleccionan `table`. Los archivos de `robot_concept` revisados asignan, por ejemplo, Shadow a `apartamento` y P3Bot a `waf`. Por tanto, el texto anterior que afirmaba que el sistema completo estaba limitado a la nevera era demasiado general.

Fuentes: `room_concept/etc/config.toml`, `room_concept/src/object_anchor_source.cpp`, `room_concept/src/object_anchor_factor.cpp`, `room_concept/src/room_gn_solver.cpp` y `robot_concept/etc/config_{shadow,p3bot}.toml`.

### 1.4. Memoria a largo plazo del original

`ltsm_agent` mantiene un grafo de memoria separado y dispone de mecanismos para archivar información de una habitación al pasar a otra y guardarla en disco. En la configuración normal revisada las opciones `Eviction.enabled` y `Memory.persist` están ausentes y el código trata esas opciones ausentes como desactivadas. Otras configuraciones de ensayo pueden elegir valores distintos.

La persistencia puede recargar el **grafo de memoria** cuando está habilitada. No se identificó en el flujo revisado un mecanismo que restaure automáticamente esos objetos al grafo de trabajo y los use para relocalizar al robot. Esta conclusión se refiere al camino inspeccionado, no a una demostración formal de que ningún otro código pueda hacerlo.

Fuentes: `ltsm_agent/src/specificworker.cpp`, `ltsm_agent/src/memory_store.h`, `ltsm_agent/src/room_eviction.cpp` y `ltsm_agent/etc/config.toml`.

## 2. Estado actual del huerto

### 2.1. Plantas: `plant_concept`

El diseño actual utiliza **`plant_concept`**, no agentes separados para el tronco y la vegetación. Los directorios antiguos `trunk_concept` y `vegetation_concept` siguen en el repositorio como componentes sustituidos; su presencia en disco o en documentación antigua no demuestra que se usen en el lanzador cognitivo actual.

`plant_concept` procesa máscaras semánticas de la cámara y datos LiDAR. Mantiene submodelos de tronco y dosel y los compone en un único nodo `plant_<id>`. Una planta puede tener solo tronco, solo dosel o ambas partes. Si se observa el tronco, su posición sirve como origen de la planta; en caso contrario se emplea el centro del dosel. Se publican atributos de las partes y una covarianza de posición.

La confirmación LiDAR y otros comportamientos dependen de la configuración y pueden desactivarse para ablaciones. El número «3 cm de error» del documento anterior no se conserva como prestación general: esta revisión no ha reproducido una prueba ni establecido a qué métrica y condiciones correspondía.

Fuentes: `active_inference/plant_concept/src/specificworker.cpp`, `plant_composer.h`, `trunk_map.h`, `vegetation_map.h`, `plant_concept/etc/config.toml` y `active_inference/utils/cognitive_husky.toml`.

### 2.2. Filas y marco de referencia

`row_metaconcept` lee las plantas con tronco observado e infiere agrupaciones alineadas. Publica las filas como metaconceptos y relaciona sus miembros mediante aristas `group_member`; también contempla incorporar plantas de solo dosel dentro de la región de una fila. La geometría es un rectángulo que representa orientación, longitud y anchura, con márgenes y parámetros configurables. `plot_metaconcept` agrupa filas en parcelas, pero no es imprescindible para definir la idea del mapa de referencia basado en filas.

El contrato actual usa **`field`** como marco exterior y concepto del lugar. El nodo anterior `world` fue retirado en esta versión. Las posiciones se expresan respecto a `field`; las relaciones de pertenencia son independientes de las transformaciones espaciales.

Fuentes: `active_inference/conceptos.md`, `active_inference/row_metaconcept/src/specificworker.cpp`, `active_inference/common/vegetation/rows.h` y `active_inference/openfield_concept/src/openfield_scene_graph.cpp`.

### 2.3. Pose e incertidumbre

`openfield_concept` mantiene un EKF de estado plano. Predice con velocidades de ruedas y velocidad angular del giróscopo; corrige con posición GPS y, si está habilitado, yaw absoluto de la IMU. El modelo tiene en cuenta el desplazamiento de la antena GPS respecto al robot y mantiene una covarianza de pose.

En `openfield_concept/etc/config.toml` se usa `GpsSigmaSource = "fixed"`, `GpsSigmaM = 0.05` m, `GpsLikelihood = "gauss"` y `UseYaw = true`. Son parámetros de la configuración revisada, no una garantía de error real de 5 cm. El código ofrece otras opciones, como incertidumbre del receptor y verosimilitud Student-t, que no son las seleccionadas ahí.

**Las plantas y filas no corrigen todavía la pose en la interfaz EKF revisada.** Sus posiciones se construyen utilizando la pose estimada. Para cerrar el bucle será necesario utilizar observaciones relativas y gestionar la correlación entre pose y mapa; volver a introducir una posición calculada con esa misma pose como si fuera independiente produciría confianza artificial.

Fuentes: `active_inference/openfield_concept/src/openfield_ekf.{h,cpp}`, `openfield_variants.{h,cpp}`, `specificworker.cpp` y `etc/config.toml`.

### 2.4. Exploración y affordances: ya existe una primera implementación

El documento anterior decía que no había exploración ni affordances; **esa afirmación ya no es correcta**. `openfield_concept` incorpora `FieldExplorer` y `OpenFieldExploration`, y `[Exploration].Enabled = true` en la configuración revisada.

- El campo se representa como una rejilla con una medida modelada de lo que queda por observar.
- Se valoran vistas por reducción esperada de incertidumbre y coste de desplazamiento. El modelo también contempla plantas incompletas, por ejemplo un dosel cuyo tronco aún no se ha observado.
- El agente ofrece objetivos mediante `afford_openfield`; el controlador puede reclamarlos y comunicar su resultado.
- El estado pasa por `Waiting`, `Inspecting` y `Complete`. `model_stable` y `model_uncertainty` resumen el estado de inspección según el modelo empleado.
- El criterio de finalización significa que ninguna vista candidata compensa su coste bajo esa configuración. **No garantiza que se hayan visto todas las plantas reales ni valida por sí mismo un mapa persistente.**

`run_route.py` y `apple_field.bash --route` siguen existiendo como ruta fija opcional; no son la única forma de movimiento disponible. La ejecución completa depende de que los agentes y el controlador estén iniciados, conectados y puedan llegar a los objetivos.

Fuentes: `active_inference/openfield_concept/src/field_explorer.h`, `openfield_exploration.{h,cpp}`, `active_inference/openfield_concept/etc/config.toml`, `active_inference/controller/` y `apple_field.bash`.

### 2.5. Memoria y reinicio

Las plantas y filas observadas se mantienen en el grafo durante la ejecución. `plant_concept` limpia nodos antiguos de plantas o troncos en su entrada a operación y `row_metaconcept` limpia filas antiguas. Esto evita adoptar entidades residuales de una ejecución anterior, pero no constituye una recuperación de mapa.

El código puede adoptar un nodo `field` ya existente en determinadas circunstancias; eso no demuestra que exista un ciclo completo de persistencia y relocalización. En el flujo de `openfield_concept` revisado no se encontró un almacén de referencia de filas ni una corrección de pose basada en su recuperación. La falta de una coincidencia en una búsqueda de código no descarta cualquier archivo o herramienta experimental externa.

Fuentes: `active_inference/plant_concept/src/specificworker.cpp` (`delete_stale_nodes`), `row_metaconcept/src/specificworker.cpp` (`delete_stale_row_nodes`) y `openfield_concept/src/openfield_scene_graph.cpp`.

## 3. Qué falta para completar la idea

1. **Consolidar un mapa de referencia compacto:** seleccionar filas estables, extremos observados y, si son necesarios para resolver ambigüedades, unos pocos individuos o patrones distintivos. Guardar sus incertidumbres, marco y relación geográfica.
2. **Persistirlo y recuperarlo:** guardar una referencia validada y mantener los checkpoints de aprendizaje separados. Al reiniciar, distinguir mapa ausente, corrupto, incompatible o correspondiente a otro lugar.
3. **Relocalizarse:** contrastar observaciones actuales de filas y referencias con el mapa guardado; usar GPS con su incertidumbre para reducir el conjunto de hipótesis. Las filas paralelas por sí solas pueden no resolver la posición a lo largo de ellas ni identificar de forma única la fila.
4. **Corregir la pose con evidencia del entorno:** integrar observaciones de filas o plantas en la estimación sin contar dos veces las medidas y sin tratar como exacto un mapa creado con una pose incierta.
5. **Completar la inspección con priors del agricultor:** admitir, según disponibilidad, zona, número esperado de filas o plantas y su grado de confianza. Comparar cobertura y expectativas sin inventar entidades para satisfacer un conteo.
6. **Validar el ciclo completo:** primera exploración, guardado, apagado, nueva posición inicial, recuperación, relocalización y prueba con GPS degradado. Medir errores longitudinal, lateral y angular, asociaciones incorrectas y calidad de incertidumbre.

La propuesta detallada está en `~/NELSON/mapeo.md`. Ese documento es una especificación de trabajo, no evidencia de que las funcionalidades allí descritas ya estén implementadas.

## 4. Límites de esta revisión

No se han ejecutado simulaciones ni campañas nuevas para esta actualización. El repositorio contiene documentación histórica y componentes sustituidos; el estado descrito se ha determinado principalmente a partir de los archivos de código y configuración indicados. La configuración efectiva de una ejecución concreta debe comprobarse en sus lanzadores y registros. El repositorio original y el agrícola pueden cambiar después de la fecha de revisión.
