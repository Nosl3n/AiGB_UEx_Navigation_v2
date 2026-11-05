# Modelo de planta: repositorios, datasets y siguientes pasos

Fecha: 2026-11-04. Recopilación de modelos de planta, frutales y datasets públicos útiles para mejorar el concepto
`plant` (ver `conceptos.md`) y para probar el stack con datos reales (rosbags).

> Aviso: no se ha descargado ni ejecutado nada salvo los cuatro repos de la sección 1, que se han clonado y leído
> (estructura y documentación; no se ha instalado GroIMP). Las licencias marcadas como "sin comprobar" hay que
> confirmarlas en la página oficial antes de reutilizar código o datos.

---

## 0. Punto de partida: nuestro modelo de planta hoy

Un nodo `plant_<id>` por planta, dueño `plant_concept`, con sus partes como atributos:

- **tronco**: posición, `plant_trunk_radius_m` (medido con el LiDAR), `plant_keepout_radius_m`, existencia;
- **dosel**: disco visto desde arriba (`plant_canopy_radius_m`, `plant_canopy_offset`);
- **altura**: `plant_height_m`.

Pendiente según `conceptos.md`: frutos, hojas o área foliar, clase de planta (joven, madura, débil), estado
sanitario y posición en la hilera.

**Regla de diseño que se mantiene:** la planta es **un solo nodo**. Los modelos de abajo representan la planta
como un grafo de cientos de órganos; para nosotros eso se resume en **atributos `plant_*`**, nunca en nodos nuevos
del DSR.

---

## 1. Repositorios analizados en detalle (código clonado y leído)

Los cuatro son **FSPM** (modelos funcional-estructurales de planta) escritos para **GroIMP**, una plataforma en
Java con el lenguaje XL, que hace crecer la planta con reglas de reescritura de grafos. Representan la planta como
un **grafo de órganos** (el fitómero: entrenudo, hoja, yema, flor y fruto). En cada paso calculan la luz que recibe
cada órgano, la fotosíntesis y el reparto del carbono, y con eso hacen crecer la planta.

Son **simuladores agronómicos**: predicen cómo *crece* una planta, no la *perciben*.

### 1.1 FruitCropXL — el relevante para nosotros

- **Repo**: <https://github.com/Plant-Food-Research-Open/FruitCropXL>
- **Licencia**: GPL-3 (comprobada), compatible con este repo.
- **Estado**: activo, unos 600 archivos, con documentación, tests y CI.
- **Especies**: manzano y vid, seleccionables con `module_configuration.species`.

**Organización**

- **Configuración por JSON** (`Model_scenarios/`): `model.options.*.json` activa y desactiva módulos;
  `plant.parameters.*.json`, `initial.conditions.*.json` y `soil-parameters-*.json` dan los valores.
- **Código** en `Scripts/`:
  - `organs/`: los órganos;
  - `alterModules/`: variantes por especie de initiation, develop, updates, fruit, buds y flowers;
  - `physioFunctions/`: las funciones de fisiología;
  - `config/` y `environment/`: configuración y ambiente.
- **Ejecución**: `init()` carga el escenario; en cada paso se ejecutan `updatesService` (estado) y, si la
  arquitectura es dinámica, `developService` (nuevos órganos).

**Fisiología** (cada módulo se activa o desactiva por separado)

- **Luz**: trazado de rayos en CPU o GPU con posición solar, o bien *surrogates* baratos (regresión de Beer-Lambert,
  modelos sustitutos, entrada directa).
- **Fotosíntesis y transpiración**: FvCB, acoplado al transporte de agua de Tardieu-Davies.
- **Carbono**: bien un depósito común de asimilados, bien transporte por el floema.
- **Fruto**: cuatro modelos (`simpleFruit`, `complexBerry`, `virtualFruit`, `cellmlFruit`).

**Datos de manzano incluidos** (`Model_input/`, `Model_scenarios/`)

- **Arquitectura medida de un manzano en espaldera** (`SG2024_Rojo_NZ_Apple_PlanarCordon_Jan1_LA25.csv`):
  - 12 969 segmentos de 1 árbol, con columnas `organ_type`, `parent`, `x,y,z`, `diameter`, `length`, `order` y
    `Age`.
  - Tipos de órgano: Trunk (3), Cordon (52), Upright (543), brotes anuales vegetativos y florales (~2 360),
    oldShoot (557), hojas (2 104), hojas de roseta (627) y **frutos (419)**.
- **Parámetros de *Royal Gala*** (`plant.parameters.RG.json`), por categorías: planta, yema, flor, hoja, entrenudo,
  madera, raíz estructural, raíz fina, intercambio gaseoso y fruto (carbono, agua y nitrógeno de cada una).
- **Perfiles de tamaño de órganos** de manzano (Angers, 2017) y **crecimiento de fruto** (Havelock North, 2022 y
  2023).

**Uso para nuestro trabajo**

1. **Vocabulario**: los nombres y unidades de los futuros atributos `plant_*` (frutos, área foliar, carga de
   cosecha, fenología).
2. **Prior del modelo generativo**: relaciones tronco–dosel–frutos (alometría) sacadas de sus parámetros y de su
   arquitectura medida.
3. **Verdad de terreno**: convertir su CSV de arquitectura en un árbol Webots, donde se conoce cada fruto y cada
   diámetro.

**No usar** GroIMP dentro del bucle del robot: es Java, es pesado y simula fisiología que el robot no percibe.

### 1.2 FSPM_BASIC (zip subido por Nelson)

- **Qué es**: el esqueleto genérico de un FSPM. Módulos `FieldBase → PlantBase → BranchBase → Apex →
  Internode/Leaf/Flower`, `Seed` y `RootSystem`.
- **Reglas de crecimiento** (`rewrites.rgg`): "un ápice produce un entrenudo, *n* hojas, una flor y un nuevo
  ápice", en versión determinada e indeterminada.
- **Crecimiento**: por fuentes y sumideros.
- **Parámetros**: 16 juegos (`parameters0..15.rgg`: maíz, girasol, quinoa, tulipán…).
- **Estado**: su README dice que **ya no se mantiene**.
- **Uso**: solo didáctico, para entender cómo se estructura un FSPM.

### 1.3 vtc (tomate joven)

- **Repo**: <https://github.com/ksmolen2/vtc>
- **Qué es**: un derivado directo de FSPM_BASIC (mismos módulos), calibrado con datos de invernadero.
- **Artículo**: *in silico Plants* (2025), <https://doi.org/10.1093/insilicoplants/diaf022>.
- **Licencia**: **no tiene archivo de licencia**, así que no se puede reutilizar su código sin permiso.
- **Uso**: solo como referencia.

### 1.4 FSP-model-dwarf-tomato (tomate enano, granja vertical)

- **Repo**: <https://github.com/Micbut/FSP-model-dwarf-tomato>
- **Qué aporta**: nivel cultivo (densidad de plantación), espectro de luz LED e **importador MTG** (el formato
  estándar de arquitectura de plantas).
- **Licencia**: **no tiene archivo de licencia**.
- **Uso**: solo como referencia (el importador MTG es un buen ejemplo).

### 1.5 quantitative-plant.org

- **Web**: <https://www.quantitative-plant.org/>
- **Qué es**: no es código, es un **catálogo** de 201 herramientas de análisis de imagen de plantas, 42 datasets y
  120 modelos.
- **Uso**: buscar herramientas de percepción, por ejemplo detección de frutos o reconstrucción de árboles a partir
  de imágenes.

---

## 2. Otros repositorios y modelos parecidos (investigados, sin clonar)

| Proyecto | Lenguaje | Frutales | Uso para nosotros |
|---|---|---|---|
| **Helios** (UC Davis) — <https://github.com/PlantSimulationLab/Helios> | **C++** (bindings Python: `pyhelios3d`) | **manzano, manzano en muro frutal**, almendro, olivo, nogal, vid (VSP y Wye), tomate, tomate cherry, fresa | **El más prometedor.** Mismo lenguaje que nuestro stack, funciona en Linux y su plugin de arquitectura trae más de 20 especies. Tiene plugins de LiDAR (procesar escaneos; LiDAR aéreo simulado con CUDA). Serviría para generar árboles con verdad de terreno para Webots. Licencia sin comprobar. |
| **MAppleT** (INRAE) — <https://hal.inrae.fr/hal-02663340> | L-studio | manzano | Modelo clásico de arquitectura de manzano: topología estocástica (cadenas semi-Markov) más flexión de ramas por el peso. Solo el artículo; no se encontró código. |
| **OpenAlea** (L-Py, PlantGL, MTG) — <https://openalea.readthedocs.io> | Python | genérico | El estándar para describir arquitectura (formato MTG). Incluye un generador procedural de árboles (WeberPenn). |
| **Virtual Plant Laboratory** — <https://virtualplantlab.com> | Julia, MIT | genérico | Alternativa moderna a GroIMP: grafo de la planta, trazado de rayos y simulación. |
| **PlantSimEngine** / **XPalm** — <https://github.com/VirtualPlantLab/PlantSimEngine.jl> | Julia, MIT | palma (XPalm) | Motor de simulación de procesos de planta. |
| **QualiTree** (INRAE) — <https://hal.inrae.fr/hal-02646867> | — | melocotonero | Calidad del fruto según el manejo. Solo el artículo. |
| **Fresa procedural para Gazebo** (Lincoln) | Gazebo | fresa | Plantas aleatorias pero plausibles para un simulador de robot. |
| Generador procedural de frutales en Blender (Duboudin et al., 2019) | Python / Blender | genérico | Imágenes sintéticas; habría que adaptarlo a Webots. |

No se encontró ninguna librería abierta de árboles frutales procedurales para Webots: nuestro
`webots-agri/protos/AppleTree.proto` ya va más allá de lo que hay publicado.

---

## 3. Qué frutales podemos usar

Criterios: que exista modelo y datos, y que el cultivo importe en Extremadura (el robot está referenciado en
Badajoz).

| Frutal | Modelo | Datos públicos | Relevancia regional | Encaje con el robot |
|---|---|---|---|---|
| **Manzano** (actual) | FruitCropXL, Helios, MAppleT | los más abundantes | baja | excelente: hileras en eje central |
| **Olivo** | Helios | pocos (sin nubes de puntos públicas) | **enorme**: ~265 000 ha (almazara y mesa) | bueno en **olivar en seto** (superintensivo) |
| **Cerezo** | ninguno encontrado | nubes de puntos de poda (WSU) | **alta**: Valle del Jerte, ~7 000 ha | bueno |
| **Ciruelo, melocotonero/nectarina** | QualiTree (sin código) | casi nada | **alta**: Vegas del Guadiana | árbol en vaso: no forma un seto continuo |
| **Almendro** | Helios | imágenes ACFR | media, en crecimiento | bueno |
| **Vid** | FruitCropXL, Helios | TEMPO-VINE (viñedo, robot) | media (Tierra de Barros) | excelente: espaldera |

Las superficies de Extremadura proceden de los anexos estadísticos de la UEX (2013–2015) y de informes del MAPA,
así que pueden estar desfasadas.

**Decisión propuesta**: seguir con el **manzano** para desarrollar (modelos y datos) y dejar el **olivo en seto**
como segundo cultivo (relevancia regional; Helios ya lo modela). La jerarquía `plant → row → plot → field` vale
igual para los dos.

---

## 4. Datasets disponibles y para qué los usaríamos

### 4.1 Robot en huerto, multisensor (rosbags para probar el stack)

| Dataset | Contenido | Uso nuestro |
|---|---|---|
| **Fraunhofer apple orchard** (Altes Land, Alemania) — <https://fordatis.fraunhofer.de> (README del dataset 417) | LiDAR 128 capas a 10 Hz, cámaras Basler de 20 MP a 5 fps, IMU a 500 Hz, GNSS RTK a 10 Hz; **rosbag** | **El más parecido a nuestro Husky.** Reproducir el bag hacia la capa sensoriomotora y probar localización (`openfield_concept`), troncos y plantas (`plant_concept`) y las hileras con datos reales. |
| **MAgro** | manzano y peral; 9 sesiones en 5 meses, 3,06 km; LiDAR 3D, dos cámaras estéreo, RTK, IMU y odometría | Pruebas a lo largo de la temporada: la misma planta con y sin hojas o frutos (persistencia del mapa semántico). |
| **TreeScope** — <https://treescope.org>, <https://arxiv.org/abs/2310.02162> | LiDAR en huertos (incluye pistachar) y bosques; **más de 1 800 troncos etiquetados con diámetro medido en campo** | **Validar el estimador de radio de tronco** (`trunk_map.cpp`) con datos reales. |
| **CitrusFarm** | robot con ruedas en cítricos; multimodal | Navegación y localización en otro cultivo. |
| **TEMPO-VINE** — <https://arxiv.org/html/2512.04772v1> | viñedo; VLP-16 y Livox Mid-360, RGB-D, AHRS, RTK; bags ROS | Probar hileras y localización en espaldera. |

Pendiente de comprobar en cada uno: enlace de descarga, licencia, versión de ROS (1 o 2) y nombres de los topics,
para escribir el puente bag → capa sensoriomotora.

### 4.2 Frutos en 3D (para un futuro atributo `plant_fruit_*`)

| Dataset | Contenido | Uso nuestro |
|---|---|---|
| **LFuji-air** (UdL, Lleida) — <https://www.grap.udl.cat/en/publications/lfuji-air-dataset/> | LiDAR VLP-16 + RTK sobre 11 manzanos Fuji; **1 353 manzanas en cajas 3D**; CC-BY; ~451 MB | Validar la **posición 3D y el conteo de frutos** por planta. |
| **Fuji-SfM** (UdL) — <https://grap.udl.cat/en/publications/fuji-sfm-dataset/> | 288 imágenes con máscaras, una secuencia de 582 imágenes y una nube SfM con la posición real de cada manzana; ~6,4 GB; licencia en conflicto (la web oficial dice solo investigación) | Detectar en 2D y localizar en 3D, el mismo esquema que retina + plant_concept. |
| **PFuji-Size** (UdL) | nubes de fotogrametría con tamaño real de fruto | Atributo de tamaño o calibre del fruto. |

### 4.3 Frutos en imagen 2D (para entrenar o ajustar `retina`)

| Dataset | Contenido | Uso nuestro |
|---|---|---|
| **MinneApple** — <https://conservancy.umn.edu/handle/11299/206575> | más de 41 000 manzanas con máscaras de instancia, en 1 000 imágenes, más conteo | Entrenar un segmentador de manzanas para la ZED. |
| **APPLE MOTS** | 1 673 imágenes, 86 000 instancias, **con seguimiento** entre fotogramas | Contar cada manzana una sola vez mientras el robot avanza. |
| **ACFR multifruit 2016** — <https://data.acfr.usyd.edu.au/ag/treecrops/2016-multifruit> | manzana, mango y almendra (cajas o círculos) | Detección multicultivo; almendro. |
| MangoYOLO, StrawDI_Db1, DeepBlueberry | mango, fresa y arándano | Solo si se cambia de cultivo. |
| Fruits-360 | clasificación de 100×100 px | **No sirve**: no es detección en campo. |

### 4.4 Arquitectura del árbol (esqueletos)

| Dataset | Contenido | Uso nuestro |
|---|---|---|
| **Marks et al., ICCV 2025** (Bonn) | **280 manzanos escaneados 10 veces durante la temporada**, con esqueleto de referencia | Estadística real de manzanos (altura, ramas, dosel) para el **prior** de la planta y para clasificar plantas como jóvenes, maduras o débiles. |
| **synthetic-trees** — <https://github.com/uc-vision/synthetic-trees> | nubes sintéticas con el esqueleto exacto, con hojas y sin ellas | Pruebas de medición de tronco y ramas con verdad exacta. |
| Cerezo para poda (WSU) — <https://arxiv.org/pdf/2103.02833> | 82 nubes de cerezo | Base si se pasa al cerezo. |
| BRANCH_v2 — <https://github.com/ividovic/BRANCH_pipeline> | peral RGB-D antes y después de la poda | Referencia. |

---

## 5. Siguientes pasos

### Paso 1 — Probar el stack con datos reales (rosbags)
1. Descargar el bag de **Fraunhofer** (o de MAgro). Comprobar licencia, versión de ROS y topics.
2. Escribir un **puente bag → capa sensoriomotora**: publicar el LiDAR, la cámara, la IMU y el GNSS con las mismas
   interfaces que hoy dan `lidar3d_dds`, `zed_camera`, `imu_dds` y el GPS del webots-bridge. Diferencias a tener en
   cuenta: el LiDAR (128 capas frente a helios) y la cámara (monocular frente a la estéreo ZED; sin profundidad
   habrá que apoyarse en el LiDAR).
3. Ejecutar `openfield_concept` (localización y campo), `plant_concept`, `row_metaconcept` y `plot_metaconcept`, y
   revisar el mapa semántico resultante.

### Paso 2 — Validar la medición del tronco
- Con **TreeScope**: comparar `plant_trunk_radius_m` con los diámetros medidos en campo. Hoy, en simulación, la
  mediana del error es +0,9 cm y la media del error absoluto 1,8 cm.

### Paso 3 — Mejorar el modelo de planta (atributos nuevos)
Documentarlos primero en `conceptos.md` y después registrarlos en cortex. Candidatos, con el vocabulario de
FruitCropXL:
- `plant_fruit_count`, `plant_fruit_positions` (en el marco de la planta), `plant_fruit_ripeness`;
- `plant_leaf_area` o densidad foliar;
- `plant_class` (joven, madura o débil) y `plant_phenology` (estado fenológico).

### Paso 4 — Prior generativo de la planta (active inference)
- Un modelo alométrico simple en C++ dentro de `plant_concept`: con el diámetro del tronco y la altura, predecir el
  radio de dosel y los frutos esperados.
- Ajustarlo con los parámetros de FruitCropXL y la estadística de Marks et al.
- Para qué sirve:
  - predecir la parte no vista de una planta incompleta;
  - detectar plantas anómalas (débiles o enfermas) por desviación del modelo.

### Paso 5 — Árboles más realistas en Webots
- Clonar **Helios**: comprobar licencia y compilación, y generar un manzano y un olivo.
- Exportar la geometría a Webots (o convertir el CSV de arquitectura de FruitCropXL), con **verdad de terreno**
  exacta de troncos y frutos.

### Paso 6 — Segundo cultivo
- Mundo de **olivar en seto** con Helios, para probar que la jerarquía `plant → row → plot → field` generaliza.
