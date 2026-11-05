# Pipeline de Generación Autónoma de Layout y Mapa: De `room_concept` a `openfield_concept`

Este documento detalla la arquitectura, fundamentos matemáticos y flujo de ejecución del pipeline de estimación y construcción autónoma de mapas (*Wall-SLAM / Model-First*) desarrollado en [`room_concept`](file:///home/nosl3n/robocomp/components/active_inference/room_concept), con el objetivo de servir como base para implementar la inicialización y descubrimiento autónomo del límite y disposición de parcelas en [`openfield_concept`](file:///home/nosl3n/robocomp/AiGB_UEx_Navigation_v2/active_inference/openfield_concept).

---

## 1. Fundamento Teórico: Enfoque *Model-First*

En la robótica clásica, el mapeo suele abordarse como una colección no estructurada de primitivas (*bag of walls* o nubes de puntos) o como una rejilla de ocupación (*grid map*). Este enfoque genera problemas críticos en entornos ambiguos:
* La deriva odométrica produce paredes duplicadas rotadas (*hairballs*).
* Las entidades libres no tienen restricciones de contigüidad o cierre topológico.

`room_concept` introduce un cambio de paradigma (**Model-First**):
1. **El espacio de estados es un polígono cerrado:** El mapa es un ciclo ordenado antihorario (CCW) de líneas de Hesse:
   $$n(\phi) \cdot p = d$$
   donde las intersecciones consecutivas de estas líneas **son** los vértices del recinto.
2. **Cierre y simplicidad como invariantes:** Un segmento no puede existir de forma aislada; cualquier cambio estructural debe preservar un polígono simple sin auto-intersecciones.
3. **Inferencia Activa y Balance Bayesiano:** Toda modificación es un compromiso entre la evidencia observada (verosimilitud) y la complejidad del modelo (prior / navaja de Ockham).

---

## 2. Inyección y Representación del Conocimiento Previo (*Priors*)

El conocimiento previo en `room_concept` se gestiona mediante dos modos operativos fundamentales (`RoomShape.MapMode`):

### 2.1. Modo `given`: Conocimiento Previo Explícito (Plano Vectorial)
Cuando se dispone de un mapa previo del entorno:
* **Plano en archivo SVG (`RoomLayoutSvg`):** Se entrega un archivo vectorial (ej. `beta_layout.svg`, `piso_layout.svg`) que define los vértices del recinto en coordenadas métricas mediante una etiqueta `<polygon id="room_contour" points="...">`.
* **Carga e inicialización (`SvgRoomLoader`):** Al iniciar, el componente parsea el SVG y crea las aristas y esquinas teóricas de la habitación.
* **Pose inicial y desambiguación:** Se puede suministrar una pose aproximada $[x, y, \theta]$ mediante un archivo semilla, o el agente ejecuta un barrido en rejilla (*grid search*) evaluando el ajuste de rayos LiDAR contra el plano para resolver la orientación (por ejemplo, distinguir giros de $180^\circ$ por simetría).
* **Invarianza:** En este modo la geometría está congelada (`frozen`); el robot únicamente se localiza optimizando su pose dentro del plano conocido.

### 2.2. Modo `estimate`: Conocimiento Previo Estructural e Inductivo (Wall-SLAM)
Cuando el robot no dispone de ningún plano y debe construir el mapa desde cero, el conocimiento previo consiste en **sesgos inductivos y priors bayesianos** sobre el mundo físico:
1. **Prior de Mundo Manhattan (*Manhattan World Assumption*):**
   * *Hipótesis:* Las paredes de los recintos humanos son perpendiculares u ortogonales ($90^\circ$).
   * *Implementación:* El modelo aplica una penalización angular suave mediante un prior gaussiano centrado en la orientación base $\theta_0$ (`manhattan_sigma_rad = 2°`), evitando derivas rotacionales espurias.
2. **Prior Semilla de Forma (Rectángulo Inicial / OBB):**
   * *Hipótesis:* Todo recinto interior puede aproximarse inicialmente como una caja convexa de 4 paredes.
   * *Implementación:* El primer escaneo LiDAR se procesa mediante una caja orientada mínima (*Oriented Bounding Box* - OBB). Las 4 paredes iniciales nacen con una incertidumbre previa en sus distancias (`rect_prior_sigma_d = 0.5 m`).
3. **Prior de Navaja de Ockham (Penalización por Complejidad):**
   * *Hipótesis:* La explicación más simple es la más probable. No deben crearse nuevas paredes a menos que la evidencia sensorial sea abrumadora.
   * *Implementación:* Para que una pared se agregue al ciclo (*splice jump*), debe aportar un incremento de energía libre decisivo:
     $$\Delta F > \text{birth\_nats} \approx 4{,}605 \text{ nats} \quad (\text{Factor de Bayes } > 100)$$
4. **Prior de Conservación ante Oclusiones (*Step-Back Operator*):**
   * *Hipótesis:* Los objetos dinámicos o muebles pueden tapar temporalmente una pared, pero la pared continúa existiendo detrás.
   * *Implementación:* Si un rayo impacta antes de la distancia esperada (*short ray*), se trata como oclusión y se preserva la creencia. Solo los rayos que atraviesan la pared sin colisionar (*beyond rays*) restan credibilidad a su existencia.

---

## 3. Pipeline de Construcción del Mapa en `room_concept`

```
[Primer Scan LiDAR] ──> [OBB / Extent Estimator] ──> [initialize_rect (Prior θ₀, 4 Paredes)]
                                                                    │
┌───────────────────────────────────────────────────────────────────┘
▼
[Nuevo Scan LiDAR]
       │
       ▼
[wall_segmenter] ──> Segmentos observados (φ, d, Σ_seg)
       │
       ├──> [Puerta de Mahalanobis χ²₂] ──> ¿Asocia a pared existente?
       │           │                                    │
       │          SÍ                                   NO
       │           │                                    │
       │           ▼                                    ▼
       │     [Gauss-Newton Solver]             [Lista de Candidatos]
       │     (Refina φ, d, pose)               (Rastreo multi-frame)
       │                                                │
       │                                                ▼
       │                                     ¿ΔF > birth_nats (ln 100)?
       │                                                │
       │                                               SÍ
       │                                                │
       │                                                ▼
       │                                     [Operador de Splice]
       │                                     (Corte esquina / Notch)
       │                                                │
       │                                                ▼
       ▼                                     ¿Polígono simple CCW?
[Step-back Operator] ──────────────────────>      [Commit al Ciclo]
(Poda por rayos atravesados)                            │
                                                        ▼
                                                [Grafo DSR: room]
```

---

### Etapa 1: Inicialización y Semilla (Bootstrap Prior)
* **Origen de coordenadas:** En el fotograma $t=0$, el robot asume su pose inicial como el origen local del mapa:
  $$q_0 = [x=0, y=0, \theta=0]^T$$
* **Estimador de Caja Orientada (OBB):** A partir del primer barrido LiDAR, [`PointcloudCenterEstimator`](file:///home/nosl3n/robocomp/components/active_inference/room_concept/src/pointcloud_center_estimator.cpp) calcula la caja orientada mínima que engloba el conjunto de impactos. Si el cálculo del OBB numérico no converge, se utiliza el rectángulo envolvente (*extent*) del escaneo.
* **Creación del Rectángulo Semilla (`initialize_rect`):** Se instancian 4 paredes iniciales que forman un rectángulo convexo ordenado en sentido antihorario.
* **Fijación de la Orientación Base ($\theta_0$):** Se establece la orientación principal del recinto. Este prior angular ($\sigma_{\theta_0}$) impide la rotación espuria de las paredes debida al ruido angular de la odometría en los primeros compases.
* **Regla de seguridad:** La caja semilla nunca debe superar el recinto real; una pared inicial generada más allá de la pared real jamás recibirá impactos LiDAR y se volvería inmortal (las oclusiones no refutan hipótesis).

---

### Etapa 2: Extracción de Segmentos (`wall_segmenter`)
* En cada ciclo sensorial, el escaneo LiDAR se procesa mediante segmentación geométrica (RANSAC / split-and-merge adaptativo).
* Cada segmento $i$ se transforma en coordenadas de la habitación y se parametriza como:
  * Ángulo normal de la recta: $\phi_i$
  * Distancia perpendicular al origen: $d_i$
  * Longitud del segmento y nube de puntos asociada.
  * Covarianza de ajuste: $\Sigma_{\text{seg}, i} \in \mathbb{R}^{2 \times 2}$

---

### Etapa 3: Asociación de Datos (Puerta de Mahalanobis)
* Se evalúa si el segmento observado pertenece a una de las paredes ya existentes en el ciclo del polígono.
* Se calcula la distancia de Mahalanobis en el espacio $(\phi, d)$:
  $$D_M^2 = \Delta^T \Big( \Sigma_{\text{seg}} + \Sigma_{\text{wall}} + J \Sigma_{\text{pose}} J^T + \Sigma_{\text{model}} \Big)^{-1} \Delta$$
  donde:
  * $\Delta = [\phi_{\text{seg}} - \phi_{\text{wall}}, d_{\text{seg}} - d_{\text{wall}}]^T$
  * $\Sigma_{\text{model}} = \operatorname{diag}(\sigma_{\text{map\_phi}}^2, \sigma_{\text{map\_d}}^2)$ es el término de incertidumbre irreducible del modelo (evita que paredes hiper-convergidas rechacen sus propias re-observaciones y generen paredes duplicadas).
* **Criterio de aceptación:** Se aplica una compuerta $\chi^2$ con 2 grados de libertad al $95\%$ de confianza:
  $$D_M^2 < 5{,}991$$
* Si supera la compuerta, el segmento se asocia a la pared y aporta información al optimizador.

---

### Etapa 4: Optimización y Refinamiento del Estado
* Los puntos pertenecientes a segmentos asociados se convierten en factores de observación de distancia punto-pared.
* El solucionador Gauss-Newton ([`room_gn_solver`](file:///home/nosl3n/robocomp/components/active_inference/room_concept/src/room_gn_solver.cpp)) optimiza conjuntamente:
  * La pose del robot en la ventana deslizante: $q_k = [x_k, y_k, \theta_k]^T$.
  * La distancia y ángulo de cada pared: $(\phi_j, d_j)$.
* Al descartar fotogramas antiguos de la ventana deslizante, su información observada se marginaliza e integra en la matriz de información de las paredes correspondientes.

---

### Etapa 5: Acumulación de Candidatos no Explicados
* Todo segmento que no supera la compuerta de Mahalanobis contra ninguna pared actual pasa a la lista de **candidatos** (`candidates`).
* Para evitar falsas alarmas debidas a ruido o personas en movimiento:
  * El candidato debe observarse al menos en $N \ge 2$ fotogramas distintos (`birth_min_frames`).
  * Debe observarse desde diferentes posiciones del robot para garantizar paralaje y descartar reflexiones espurias.

---

### Etapa 6: Saltos Topológicos (*Splice Jumps*)
* Las nuevas paredes no "nacen" flotando; se incorporan mediante saltos topológicos directos en la secuencia cíclica del polígono.
* **Cálculo de Energía Libre / Factor de Bayes:** Un candidato propone un empalme si su soporte acumulado supera el coste de penalización por complejidad (parámetro de Ockham):
  $$\Delta F > \text{birth\_nats} \approx 4{,}605 \text{ nats} \quad (\text{equivalente a un factor de Bayes } > 100)$$
* **Tipos de operaciones de Splice:**
  1. **Corte de esquina (*Corner cut*):** Transforma una esquina entre dos paredes perpendiculares $[E, N]$ en una secuencia $[E, \text{jog}, C, N]$, permitiendo que el rectángulo inicial adopte formas en 'L' o en 'U'.
  2. **Muesca / Extensión (*Notch*):** Inserta un saliente $[E, \text{jog}, C, \text{jog}, E]$ en mitad de una pared para modelar retranqueos, columnas o aperturas a otras salas.
  3. **Bisel (*Chamfer*):** Inserción oblicua sin esquinas a $90^\circ$.
* **Validación geométrica:** El salto solo se confirma si el polígono resultante es simple, no auto-intersecante y orientado en sentido antihorario. Si no cumple la condición, el salto se descarta.

---

### Etapa 7: Operador de Desaparición y Poda (*Step-Back Operator*)
* La refutación de hipótesis es tan rigurosa como su creación:
  * Cada rayo del sensor LiDAR proyectado hacia la extensión de una pared se discretiza en contenedores espaciales (*bins*).
  * Impacto en la pared $\rightarrow$ **Soporte positivo** (la pared existe en ese punto).
  * Rayo que supera la pared de largo (*beyond*) sin impactar $\rightarrow$ **Refutación** (la pared no está ahí).
  * Rayo interrumpido antes de llegar (*short*) $\rightarrow$ **Oclusión** (se mantiene la creencia actual sin penalizar).
* Si una pared pierde soporte por debajo del umbral simétrico de muerte ($< -\text{birth\_nats}$), se desempalma del ciclo y las paredes vecinas se fusionan.

---

### Etapa 8: Publicación y Estabilidad en el Grafo DSR
* Cuando la incertidumbre estadística acumulada de todas las esquinas del polígono desciende de un umbral métrico ($\sigma_{\text{corner}} \le 0{,}06\text{ m}$), el mapa se considera estabilizado:
  * Se marca el atributo `model_stable = true`.
  * Se actualizan en el nodo `room` del grafo DSR los atributos `delimiting_polygon_x` y `delimiting_polygon_y`.
  * Se mantiene la transformación espacial `robot —RT→ room` que ubica al robot dentro del recinto.

---

## 4. Adaptación del Pipeline y Conocimiento Previo a `openfield_concept`

En el ámbito agrícola de exterior ([`openfield_concept`](file:///home/nosl3n/robocomp/AiGB_UEx_Navigation_v2/active_inference/openfield_concept)), el entorno no presenta paredes sólidas continuas, sino **hileras de árboles frutales (manzanos), caminos y perímetros de parcela**. La traslación directa de los conceptos se define así:

| Concepto en `room_concept` | Equivalente en `openfield_concept` | Implementación en Campo |
|---|---|---|
| **Conocimiento Previo Explícito (`given`)** | **Catastro / Perímetro GPS aportado por el agricultor** | Vértices en lat/lon o UTM cargados en configuración (`poly_lat/poly_lon` o `poly_x/poly_y`). |
| **Pared (*Wall landmark*)** | **Hilera de árboles / Límite de parcela** | Línea de Hesse orientada $n(\phi_{\text{row}}) \cdot p = d_{\text{row}}$ que agrupa troncos y doseles continuos. |
| **Prior Manhattan ($\theta_0$ a $90^\circ$)** | **Orientación dominante del huerto ($\theta_{\text{rows}}$)** | Ángulo dominante de las calles de plantación. Las hileras son paralelas entre sí y perpendiculares a las cabeceras (*headlands*). |
| **Rectángulo semilla inicial (`initialize_rect`)** | **Bloque inicial de plantación** | OBB que engloba el primer conjunto de árboles detectados por LiDAR/cámara, expandido con márgenes de cabecera (`margin_x`, `margin_y`). |
| **Puerta de asociación (Mahalanobis)** | **Asociación de troncos/árboles a hileras** | Evaluación de si una nueva planta detectada pertenece a una hilera existente según su distancia transversal a la línea de la hilera. |
| **Salto estructural (*Splice Jump*)** | **Descubrimiento de nueva hilera o parcela** | Cuando se confirman suficientes árboles alineados fuera del límite actual, el polígono `field` se expande integrando la nueva hilera. |
| **Marco de referencia del grafo** | **Nodo `field` (sustituye a `room`)** | `openfield_concept` publica el nodo `field` con su polígono delimitador `delimiting_polygon_x/y` y la arista `robot —RT→ field`. |

---

## 5. Hoja de Ruta para la Integración en `openfield_concept`

1. **Módulo de Bootstrap de Parcela:**
   * Adaptar la lógica de `PointcloudCenterEstimator` para calcular el rectángulo inicial del campo a partir de los centros de plantas detectados por `plant_concept` o los primeros barridos LiDAR 3D.
2. **Generador del Polígono Delimitador (`field_polygon`):**
   * Sustituir la dependencia estática de `Field.Source = "config"` o `Field.Source = "webots"` por un modo `"estimate"` que construya dinámicamente el polígono `field` conforme el Husky y el dron inspeccionan el terreno.
3. **Cierre de Bucle con Orientación de Hileras:**
   * Utilizar la orientación de las hileras ($\theta_{\text{rows}}$) inferida por `row_metaconcept` como el ancla angular que reemplace al prior Manhattan de interiores.
