# Conceptos del huerto en el grafo DSR

Este documento es el **contrato** de los nodos que representan el huerto en el grafo DSR: qué es cada concepto, qué
agente es su dueño, con qué se relaciona y **qué atributos lleva**. Cuando se añada una característica nueva a un
concepto (por ejemplo, frutos en la planta), se documenta aquí *antes* de publicarla.

## Concepto y metaconcepto

- **Concepto**: un *individuo* con cuerpo propio y sus características. Sus partes (tronco, dosel, frutos…) son
  **atributos de su nodo**, no nodos aparte, igual que una persona es un nodo y no un nodo por brazo.
  Ejemplos: `plant`, `person`.
- **Metaconcepto**: un *conjunto o disposición de individuos*. No tiene cuerpo propio (su forma es la de sus
  miembros) y sus miembros son individuos completos; si se quita uno, el conjunto sigue existiendo.
  Tipo de nodo `metaconcept`. Ejemplos: `row` (una hilera es un conjunto de plantas), `plot` (una parcela es un
  conjunto de hileras), `field` (el campo es el conjunto de parcelas).

## Jerarquía

```
robot                               robot_concept        el robot existe por sí mismo …
field                field          openfield_concept    … y se ubica en un campo: el marco Y el concepto del lugar
 └─ plot_<id>        metaconcept    plot_metaconcept     parcela: hileras plantadas juntas
     └─ row_<id>     metaconcept    row_metaconcept      hilera: conjunto de plantas
         └─ plant_<id>  plant       plant_concept        planta: un individuo (tronco, dosel, … como atributos)
```

> **2026-11-04:** desaparece el nodo `world`. El **`field` es a la vez el marco de referencia del robot y el concepto
> del lugar** ("creo que estoy en un campo"); lleva su polígono y, en el futuro, su georreferencia GPS para situarse
> en un mapa mayor. `field_metaconcept` (que creaba un `field_0` aparte) queda obsoleto: `openfield_concept` agrupa
> las parcelas en el `field`. `common/world_frame/world_frame.h` resuelve el marco (`field` o `room`).

| Relación | Arista | Quién la escribe |
|---|---|---|
| dónde está el robot | `robot —RT→ field` (guarda T_robot←field, como room_concept) | openfield_concept |
| dónde está cada cosa | `field —RT→ plot_<id> / row_<id> / plant_<id>` | el dueño de cada nodo |
| parcelas del campo | `field —group_member→ plot_<id>` | openfield_concept |
| hileras de una parcela | `plot_<id> —group_member→ row_<id>` | plot_metaconcept |
| plantas de una hilera | `row_<id> —group_member→ plant_<id>` | row_metaconcept |

Las posiciones cuelgan siempre de `field` (nada cuelga de su grupo): la pertenencia es la arista `group_member`,
no la cadena de transformaciones.

---

## `field` — el campo: marco, concepto del lugar y modo de inspección (openfield_concept)

- **Nodo**: tipo `field`, nombre `field`. **Dueño**: `openfield_concept` (id de agente 25). Es el marco del robot en el
  exterior (x este, y norte, origen GPS) y el concepto del lugar: lleva el polígono del campo (`delimiting_polygon_x/y`,
  una entrada: la zona que da el agricultor), la pose del robot (RT `robot → field`) y sus parcelas (`group_member`).
- **Modo de inspección** (`openfield_concept/src/field_explorer.h`, `openfield_exploration.h`): la versión de campo
  abierto de la exploración de room_concept. El campo es una rejilla cuyas celdas están **sin ver** hasta que el robot
  las mira; el punto que más reduce esa entropía (menos el coste de llegar, en nats) se ofrece al controller como
  el affordance **`afford_openfield`**. Estados: `Waiting → Inspecting → Complete`.
- **Qué hay dentro antes que cubrir terreno**: una planta con solo una parte (dosel sin tronco o tronco sin dosel)
  es una pregunta abierta. P(la parte falta pero existe sin verse) empieza en 0,5 (algo anda mal) y baja con cada
  mirada que debería haberla visto; el robot vuelve a mirarla mientras la duda es alta, y cuando la duda se resuelve
  la planta queda tal cual en el mapa semántico, revisada. Esa información pesa más que la cobertura
  (`Exploration.ConceptWeight`).
- **Fin de la inspección**: cuando ningún punto del campo compensa su recorrido (signo de la energía libre esperada,
  no un umbral): ni cobertura ni preguntas sobre plantas que valga la pena ir a resolver.

| Atributo en `field` | Tipo | Significado |
|---|---|---|
| `model_stable` | bool | la inspección está completa |
| `model_uncertainty` | float [0, 1] | fracción de la entropía del campo que queda por ver |

### `afford_openfield` (affordance de exploración)

- **Nodo**: tipo `affordance`, nombre `afford_openfield`, hijo de `field` (atributo `parent` y arista de affordance).
  **Dueño**: `openfield_concept`. Sustituye a `afford_room` de room_concept.
- **Contrato** (`common/affordance_protocol`): política `Reach`; `epistemic_target_x_m/_y_m/_yaw_rad` = el punto y
  hacia dónde mirar (lo que queda sin ver); `epistemic_gain` = la ganancia **marginal** en nats (lo que aporta ir allí
  frente a quedarse); protocolo `Offered → Executing → Completed` (`epistemic_pending`, `active`).
- **Resultado**: si el controller no pudo llegar (`unreachable`, `infeasible`, `outside_world`) el punto queda
  suprimido un tiempo (decae con `Exploration.FailedSuppressS`).
- En el controller compite con los affordances de objetos con `ganancia − λ·distancia`; mientras un objeto compite,
  su ganancia se escala por `Controller.AffordanceExplorationGainScale` (antes `AffordanceRoomGainScale`).

---

## `plant` — la planta (concepto)

- **Nodo**: tipo `plant`, nombre `plant_<id>`. Un nodo por planta.
- **Dueño**: `plant_concept` (id de agente 29). Ningún otro agente escribe en él.
- **Origen (RT `field → plant`)**: donde está la planta. Es el **eje del tronco** si el tronco se ha visto; si no,
  el **centro del dosel**. La arista lleva `rt_covariance` (varianza de la posición en x, y).
- **Planta incompleta**: una planta puede tener solo tronco (dosel aún no visto) o solo dosel (tronco tapado por el
  follaje). Cuando aparece la parte que falta, **se completa la misma planta** y conserva su `id`.
- **Emparejamiento**: un tronco pertenece a la planta cuyo dosel lo contiene (el tronco está dentro del disco del
  dosel); uno a uno, empezando por los más cercanos.

### Atributos

| Atributo | Tipo | Unidad | Significado |
|---|---|---|---|
| `obj_id` | int | — | identificador estable de la planta (el `<id>` del nombre) |
| `obj_exist_prob` | float | [0, 1] | creencia de que la planta existe: 1 si se ha visto su dosel; si no, la de su tronco |
| **Tronco** | | | |
| `plant_has_trunk` | bool | — | el tronco se ha visto y confirmado (ZED + LiDAR) |
| `plant_trunk_exist_prob` | float | [0, 1] | creencia de que el tronco existe; baja solo al mirar sin oclusión y no encontrarlo |
| `plant_keepout_radius_m` | float | m | radio del disco alrededor del tronco en el que el robot no puede entrar (0 sin tronco) |
| `plant_trunk_radius_m` | float | m | radio **físico** del tronco, medido con el LiDAR (0 = aún sin medir) |
| **Dosel** | | | |
| `plant_has_canopy` | bool | — | el dosel (follaje visto desde arriba) se ha visto |
| `plant_canopy_radius_m` | float | m | radio del dosel visto desde arriba (0 sin dosel) |
| `plant_canopy_offset` | vector\<float\> [dx, dy] | m | centro del dosel menos el origen de la planta, en ejes del `field` |
| **Planta completa** | | | |
| `plant_height_m` | float | m | punto de vegetación más alto observado en el dosel (0 = desconocido) |
| **Vista del grafo** | | | |
| `pos_x`, `pos_y` | float | px | solo para el visor: debajo de su hilera, o en la zona "sin hilera" |
| `timestamp_creation`, `creation_datetime` | — | — | sello de creación (common/graph_provenance) |

### Cómo se percibe cada parte (modelo generativo de `plant_concept`)

- **Tronco** (`plant_concept/src/trunk_map.h`): las máscaras ADE20K `tree` de la ZED, en la franja entre el suelo y
  la primera rama (z 0,30–0,55 m), proponen un tronco; el LiDAR helios lo **confirma** con puntos en la misma franja
  y lo coloca (~cm). Su **radio** se mide en cada barrido: los rayos del LiDAR están equiespaciados en ángulo, así que
  sobre la silueta del tronco los puntos se reparten uniformes a lo ancho y R = 2 · mediana(|desplazamiento lateral|),
  tras separar el tutor por el hueco que lo aísla de la silueta; se promedia entre barridos. Es permanente hasta que se **verifica** que no está: la cámara mira donde debería estar, sin que
  lo tape el follaje ni una persona, y no lo encuentra.
- **Dosel** (`plant_concept/src/vegetation_map.h`): todos los puntos de las máscaras de vegetación acumulados en una
  rejilla de evidencia sobre el suelo; un disco de radio fijo (`Canopy.RadiusM`, 0,5 m). No se retira.
- **Composición** (`plant_concept/src/plant_composer.h`): junta las dos partes en individuos con identidad estable.

### Quién lee la planta

Todos a través de `common/vegetation/plant_discs.h`, que devuelve dos discos por planta:

| Disco | Centro | Radio | Lo usa |
|---|---|---|---|
| tronco | origen | `plant_keepout_radius_m` | controller: obstáculo (solo plantas que no están en una hilera) |
| dosel | origen + `plant_canopy_offset` | `plant_canopy_radius_m` | residual (espacio explicado), human (ninguna persona dentro), oclusor del tronco |

Además, con `read_plants()` (solo visualización):
- **mapa 2D del controller**: el dosel (verde), el tronco a su diámetro medido (marrón) y un círculo rojo discontinuo
  donde falta una parte (anillo grande: dosel no visto; pequeño: tronco no visto);
- **mundo 3D de openfield_concept**: tronco como cilindro de su radio, dosel como cono, partes no vistas en rojo.

### Características futuras (aún no publicadas)

Se añadirán como atributos `plant_*` del mismo nodo, nunca como nodos nuevos:

- frutos: número, madurez, posición en el dosel;
- hojas: densidad / área foliar;
- clase de planta (madura, joven, débil) y estado sanitario;
- posición dentro de su hilera (índice a lo largo de la hilera).

---

## `row` — la hilera (metaconcepto)

- **Nodo**: tipo `metaconcept`, nombre `row_<id>`. Un nodo por hilera.
- **Dueño**: `row_metaconcept` (id de agente 28).
- **Origen (RT `field → row`)**: centro del rectángulo de la hilera, con su rumbo como giro (yaw).
- **Cómo se infiere**: no se ve; sale de dónde están los **troncos** de las plantas (`common/vegetation/rows.h`):
  troncos cercanos sobre una línea forman una hilera (mínimo 3), un rectángulo que crece al unirse más troncos.
- **Miembros** (`group_member`): las plantas cuyos troncos forman la hilera, y las plantas con solo dosel (tronco
  aún no visto) cuyo dosel cae dentro del rectángulo.
- **Uso**: el controller marca el rectángulo de cada hilera como espacio prohibido (en vez de un disco por tronco).

### Atributos

| Atributo | Tipo | Unidad | Significado |
|---|---|---|---|
| `obj_id` | int | — | identificador estable de la hilera |
| `width_m` | float | m | longitud del rectángulo **a lo largo** de la hilera |
| `depth_m` | float | m | anchura del rectángulo **a través** de la hilera |
| `height_m` | float | m | altura publicada (la de los troncos, para visores) |
| `pos_x`, `pos_y` | float | px | solo para el visor: capa de hileras, de oeste a este |

---

## `plot` — la parcela (metaconcepto)

- **Nodo**: tipo `metaconcept`, nombre `plot_<id>`. **Dueño**: `plot_metaconcept` (id de agente 30).
- **Cómo se infiere** (`common/vegetation/plots.h`): dos hileras son de la misma parcela si son **paralelas**
  (`Plots.ParallelTolRad`, ~10°), están **una al lado de la otra** (se solapan a lo largo) y separadas como mucho
  `Plots.SpacingFactor` (1,5) veces la **distancia de plantación**, que no se configura: es la mediana de la distancia
  de cada hilera a su vecina más cercana. Un hueco mayor (cabecera, camino, otra plantación) separa parcelas.
- **Identidad**: conserva su `id` mientras conserve sus hileras; crece al unirse hileras; si dos se funden queda la
  más antigua.
- **Miembros**: `plot_<id> —group_member→ row_<id>`.

| Atributo | Tipo | Unidad | Significado |
|---|---|---|---|
| `obj_id` | int | — | identificador estable de la parcela |
| `width_m` | float | m | longitud del rectángulo **a lo largo** de las hileras |
| `depth_m` | float | m | anchura del rectángulo **a través** de las hileras |
| RT `field → plot` | — | m, rad | centro del rectángulo y rumbo de las hileras (yaw) |

## (obsoleto) `field_0` — el campo como metaconcepto aparte

Hasta 2026-11-04 `field_metaconcept` publicaba un nodo `field_0` (tipo `metaconcept`) con el rectángulo que envolvía
las parcelas. Ya no existe: el campo es el nodo `field` de arriba.

---

## Cómo añadir una característica nueva a un concepto

1. Documentarla aquí (atributo, tipo, unidad, significado, quién la escribe).
2. Registrar el atributo en cortex (`core/include/dsr/core/types/type_checking/dsr_attr_name.h`,
   `REGISTER_TYPE(plant_<nombre>, <tipo>, false)`) y **reinstalar cortex** (lo hace Nelson, requiere `sudo`).
3. Publicarla desde el agente dueño del concepto (para `plant`, solo `plant_concept`).
4. Si otros agentes la necesitan, leerla a través del lector común del concepto (`plant_discs.h` para `plant`).
