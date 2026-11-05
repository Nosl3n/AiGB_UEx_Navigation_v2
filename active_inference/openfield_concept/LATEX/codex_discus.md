# Discusión: mapeo y localización en openfield_concept

Este documento recoge la discusión sobre la construcción de un mapa de referencia del campo, inspirado en `room_concept` y adaptado a un entorno agrícola. Se irá ampliando conforme se tomen decisiones. Las propuestas se distinguen del funcionamiento observado en el código.

## 1. Hipótesis inicial y conocimiento previo

La idea general es que el robot empieza con cierta información previa, explora y actualiza su conocimiento con las observaciones. Sin embargo, los componentes no descubren automáticamente desde cero que el entorno es una habitación o un campo: esa interpretación inicial viene dada por la configuración y los componentes que se ejecutan.

Un prior puede describir tanto la clase de entorno y la estructura esperada como una distribución de probabilidad sobre características desconocidas. En un campo, por ejemplo, el agricultor podría proporcionar una zona aproximada, el número esperado de filas, su orientación o separación. Estas indicaciones pueden ser inciertas y deben poder revisarse frente a la evidencia.

## 2. Referencia: room_concept

En la configuración original de `room_concept` existen dos modos:

- **`given`**, activo en la configuración revisada: carga un mapa de la habitación y lo utiliza para localizar al robot.
- **`estimate`**: parte de una geometría inicial aproximada obtenida de la primera observación LiDAR y ajusta conjuntamente la geometría de la habitación y la pose del robot.

El modo de estimación usa hipótesis estructurales, entre ellas una preferencia por orientaciones de paredes compatibles con una geometría tipo Manhattan. Cuando el mapa cumple las condiciones configuradas para considerarse publicable, puede congelar el layout y pasar a localización. También dispone de un mecanismo de exploración mediante affordances, sujeto a configuración.

El paso a localización no implica por sí solo que el mapa se guarde en memoria persistente ni que se recupere automáticamente en un arranque posterior.

## 3. Estado actual de openfield_concept

La configuración revisada utiliza `[Field] Source = "webots"`. El componente consulta el mundo de Webots para obtener información de objetos del escenario y construir un polígono del campo. También puede utilizar un polígono proporcionado mediante configuración.

El polígono obtenido de Webots emplea información privilegiada de la simulación: no es un límite descubierto por el robot mediante sus sensores. Puede utilizarse como delimitación de la zona de trabajo, pero debe distinguirse del mapa que se desea aprender.

Existe además una base para explorar: el componente mantiene una representación de incertidumbre del área y evalúa objetivos considerando la información esperada y el coste de desplazamiento. Utiliza supuestos configurables, incluida una probabilidad inicial de encontrar plantas. Es un punto de partida para la exploración, pero todavía no constituye el sistema completo de creación, persistencia y recuperación de un mapa de filas para localización.

El filtro actual asigna al GPS una desviación configurable de `0,05 m`. Este valor es un parámetro del modelo, no una garantía de precisión del sensor real; deberá ajustarse al dispositivo y a las condiciones de operación.

## 4. Comparación

| Aspecto | `room_concept` | `openfield_concept` actual |
| --- | --- | --- |
| Hipótesis inicial | Se configura para trabajar con una habitación. | Se configura para trabajar con un campo. |
| Información inicial | Puede recibir un mapa o partir de una forma aproximada. | Recibe o construye un polígono inicial. |
| Evidencia principal para el mapa | Paredes observadas con LiDAR. | Plantas observadas y filas inferidas a partir de ellas. |
| Objetivo para la localización | Un layout de la habitación. | Se propone un layout persistente de filas; aún debe implementarse. |

## 5. Pipeline propuesto

La propuesta es que `openfield_concept` gestione la creación del mapa, la localización y la recuperación de un mapa guardado. El layout del campo se basará principalmente en **filas**, sin necesidad de guardar todas las plantas para localizar al robot.

### 5.1. Inicialización del campo

El sistema recibe una zona aproximada de trabajo y, si están disponibles, priors del agricultor: número esperado de filas, orientación aproximada, separación u otra información útil. Debe representarse la incertidumbre de estos datos y permitirse su revisión cuando las observaciones los contradigan.

El polígono inicial delimita la misión; no debe confundirse automáticamente con una frontera medida del campo ni con el layout aprendido.

### 5.2. Selección del modo

Si existe un mapa de referencia compatible con el campo, se carga y se intenta localizar al robot. Si no existe, se inicia la construcción de un mapa nuevo.

La identidad inicial del campo puede proceder de la misión, de la posición GPS o de una combinación de ambas. Se trata de una hipótesis que debe contrastarse con las observaciones.

### 5.3. Observación y construcción de filas

Durante la exploración se observan plantas y se estiman las filas a partir de esas evidencias. Para cada fila se propone mantener su posición, orientación, longitud e incertidumbre. Las filas, no paredes imaginarias, constituirán la estructura principal del layout.

### 5.4. Exploración dirigida

`openfield_concept` seleccionaría objetivos que puedan reducir la incertidumbre del layout: por ejemplo, observar una fila incompleta o explorar una zona donde se espera encontrar otra. El mecanismo existente de exploración por ganancia de información puede servir como punto de partida.

### 5.5. Corrección de la pose

Las filas observadas se asociarían con las filas del mapa y se utilizarían como evidencia para localizar al robot. Una fila larga puede ayudar a estimar la orientación y la distancia lateral, pero por sí sola puede no determinar la posición a lo largo de ella. Las filas similares también pueden confundirse entre sí.

El diseño debe tratar la incertidumbre y las correlaciones entre pose y mapa, evitando tomar como evidencia independiente una fila calculada a partir de la misma pose que se pretende corregir.

### 5.6. Finalización y guardado

Cuando la cobertura y la incertidumbre del layout cumplan criterios por definir, se guardaría un mapa compacto: geometría e incertidumbre de las filas y, cuando ayuden a resolver ambigüedades, algunos puntos distintivos como extremos de fila observados.

La extensión conocida o el número esperado de filas pueden ayudar a evaluar la finalización, pero no prueban por sí solos que se haya observado toda la zona. Los criterios probabilísticos y la política para actualizar un mapa guardado quedan pendientes de diseño.

### 5.7. Arranque posterior

En una visita posterior, el sistema cargaría el mapa, utilizaría el GPS y su incertidumbre para acotar la posición inicial y contrastaría las filas observadas con las guardadas. Si varias posiciones fueran plausibles, el robot podría explorar o buscar un extremo de fila para distinguirlas.

## 6. Fuentes locales revisadas

- `~/robocomp/components/active_inference/room_concept/etc/config.toml` y código de inicialización de `room_concept`.
- `~/robocomp/AiGB_UEx_Navigation_v2/active_inference/openfield_concept/etc/config.toml` y código de `openfield_concept`, incluida su exploración y construcción del campo.

Las propuestas de este documento son decisiones de diseño pendientes de implementación y validación; la revisión descrita se basa en el código y la configuración, no en una prueba de ejecución.

## 7. ¿Cómo se proporciona el conocimiento previo?

En `room_concept` el conocimiento previo procede de varias fuentes, no de una única lista de hechos:

1. **La configuración y el modelo del componente.** El agente ya está diseñado para representar una habitación mediante un polígono y paredes. Ejecutarlo introduce esa interpretación del entorno; no supone que haya clasificado autónomamente el lugar como habitación.
2. **Un plano suministrado, en modo `given`.** `MapMode = "given"` selecciona el uso de un layout SVG indicado mediante `RoomLayoutSvg` y `LayoutDir`. El componente carga el contorno y se localiza con respecto a él. Esta es la configuración activa revisada.
3. **Una hipótesis inicial obtenida del LiDAR, en modo `estimate`.** Sin cargar un plano, el primer barrido se utiliza para inicializar una caja orientada aproximada. Esta semilla se convierte en un rectángulo con cuatro paredes iniciales; la pose inicial define el origen local. Si no se obtiene la caja orientada, se usa la extensión de los puntos y existe un último rectángulo de respaldo para una extensión degenerada. La primera observación aporta datos, pero convertirla en un rectángulo es una hipótesis estructural del modelo.
4. **Parámetros de incertidumbre y estructura.** La configuración incluye, entre otros, `RectPriorSigmaD = 0.50 m` para la incertidumbre inicial de posición de cada lado y `RectPriorSigmaPhiDeg = 15°` para su orientación. También incluye `ManhattanSigmaDeg = 2°` para una preferencia probabilística por orientaciones paralelas o perpendiculares; no es una regla rígida y se contempla que una pared no pertenezca a esas clases. Las medidas del LiDAR y el modelo de movimiento tienen sus propios parámetros de ruido.

Estos valores son los de la configuración examinada, no propiedades universales de las habitaciones. Una hipótesis inicial equivocada o demasiado segura puede perjudicar el resultado; el propio código advierte del peligro de sembrar paredes detrás de las reales.

## 8. Adaptación propuesta para un campo

Se propone entregar la información inicial al componente mediante un archivo de configuración de misión, y más adelante, si resulta útil, mediante una interfaz para el agricultor. El formato siguiente es ilustrativo: **estos campos no se presentan como configuración ya implementada**.

```toml
# Ejemplo conceptual; valores ficticios, no calibrados.
[Mision]
campo_id = "campo_prueba"
modo = "auto"                 # cargar mapa compatible o crear uno nuevo
zona_trabajo = "poligono.geojson"

[Filas]
numero_esperado = 5          # opcional; no equivale a filas observadas
separacion_media_m = 3.5     # opcional
separacion_sigma_m = 0.5     # incertidumbre de la separación
orientacion_grados = 10.0    # opcional; convención por definir
orientacion_sigma_grados = 15.0
```

La zona de trabajo define dónde puede planificarse y buscarse; no equivale automáticamente al límite real observado del cultivo. El identificador facilita encontrar un mapa candidato, pero no confirma por sí mismo que el robot se encuentre en él. Los números son ejemplos ficticios y no deben interpretarse como datos del campo real ni de `dos_fields.wbt`.

En un campo nuevo, el modelo puede asumir que hay filas aproximadamente paralelas, pero comenzar sin filas afirmadas y crear hipótesis a partir de las observaciones. El número esperado, la separación y la orientación guían la exploración y la evaluación de hipótesis; no deben fabricar filas que no se hayan visto. Una ausencia de detección solo aporta evidencia cuando realmente se esperaba que el sensor pudiera ver la zona.

Con un mapa guardado se usaría su layout como referencia, de modo análogo al plano suministrado en `given`, pero verificando las asociaciones con filas observadas y teniendo en cuenta cambios desde la visita anterior. GPS y su incertidumbre pueden reducir las posiciones candidatas; las filas y los extremos observados ayudarían a comprobar la posición. El cambio de modo y la persistencia son propuestas pendientes, no capacidades de `openfield_concept` ya verificadas.

Actualmente `[Field] Source`, `PolygonX`/`PolygonY` o `PolygonLat`/`PolygonLon`, y `[Exploration] PriorPlantP` sí son parámetros existentes. La fuente `webots` obtiene información privilegiada del simulador; para ensayar una misión más próxima al caso real puede proporcionarse una zona mediante `Source = "config"`. El soporte actual de GPS permite elegir una incertidumbre fija o la declarada por el receptor, cuando se dispone de la información necesaria.
