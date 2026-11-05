# Dron de inspección: estado simulado y opciones futuras

**Documento de trabajo.** Esta nota conserva la comparación realizada el 5 de noviembre de 2026 entre el dron de Webots y posibles plataformas reales. Las características comerciales, disponibilidad, precios y compatibilidad de SDK pueden cambiar; deben verificarse de nuevo antes de adquirir un equipo.

## Objetivo

El dron se concibe como una cámara aérea móvil que complementa las observaciones del Husky. El procesamiento y el mapa semántico pueden permanecer en el robot o en un servidor. Una primera misión podría elevar el dron, capturar imágenes de las filas y descender. Las imágenes deben acompañarse de la pose de la cámara, su calibración y el instante de captura.

Quedan abiertas dos formas de operación:

1. **Dron transportado por el Husky:** despega y vuelve a una plataforma sobre el robot, inicialmente con el Husky detenido.
2. **Dron desde una base fija:** despega y aterriza en un lugar preparado del huerto; comparte observaciones con el Husky por comunicaciones.

No es necesario elegir ahora. La base fija amplía las opciones de tamaño, masa, batería y plataforma de aterrizaje; transportarlo en el Husky acerca el sensor al punto de inspección, pero exige resolver sujeción, despeje de sensores y aterrizaje sobre el vehículo.

## Código y configuración actuales

- `flight_controller.h`: control y estabilización del dron simulado.
- `specificworker.cpp`: interacción con Webots, sensores, órdenes, publicación de pose e imagen.
- `camera_publisher.cpp`: publicación de la cámara en DDS.
- `../etc/config.toml`: configuración del puente, cámara, mando, DDS y control de vuelo.
- `../../webots-agri/protos/InspectionDrone.proto`: geometría, masa, sensores y cámara simulados.
- `../../webots-agri/protos/InspectionDeck.proto`: plataforma y marcador.
- `../../webots-agri/worlds/dos_fields_dron.wbt`: escenario con Husky y dron.
- `../../active_inference/robot_concept/src/drone_link.cpp`: integración de dron y cámara en el grafo del robot.
- `../../webots-agri/DRON_INSPECCION.md`: documentación funcional y de operación de la versión simulada.
- `camera_dron.md`: especificación técnica de la cámara simulada (gran angular 130° FOV) y opciones de hardware ligero.

El lanzador `../../apple_dron.bash` abre el escenario y las capas correspondientes. Comprobar siempre los archivos de configuración de la versión que se ejecute.

## Modelo simulado revisado

`InspectionDrone.proto` deriva del Mavic 2 Pro de Webots R2025a, pero utiliza `scale = 0.5`: geometría y articulaciones se reducen a la mitad. Las masas y las constantes de hélices siguen reglas de escala elegidas para que el modelo simulado pueda estabilizarse. **No es una réplica validada de un dron comercial concreto.**

| Característica | Modelo revisado |
|---|---|
| Masa | Aproximadamente 230 g según el código y la documentación del proyecto |
| Distancia entre ejes de motores opuestos | Aproximadamente 19 cm, calculada a partir de los centros del modelo |
| Extensión de los discos de hélices | Aproximadamente 24 × 27 cm, calculada; no es una medición física del contorno completo |
| Cámara | RGB, 1280 × 720, campo horizontal de 60° |
| Orientación | Un eje `camera pitch`: frontal y hacia abajo |
| Sensores simulados | GPS, IMU, giróscopo y brújula |
| GPS | `accuracy = 0.05 m`, con ruido correlacionado: parámetro de simulación, no prestación de un equipo real |
| Publicación de cámara | DDS, según la configuración del puente |
| Pose | Obtenida actualmente del supervisor de Webots: verdad de simulación |
| Plataforma del Husky | 40 × 40 cm según `DRON_INSPECCION.md` |
| Marcador | ArUco `DICT_4X4_50`, ID 23; lado negro de 0,17 m según la documentación actual |

El puente dispone de estabilización y control mediante mando. La documentación local indica que no hay despegue ni aterrizaje automáticos y que el dron no evita obstáculos de forma autónoma. La pose proporcionada por el supervisor sirve para evaluar la integración, pero no demuestra que el sistema estime la pose mediante sensores reales. No se ha establecido una autonomía de batería equivalente a la de un producto comercial.

## Alternativas de operación

### Base sobre el Husky

El Husky transporta el dron hasta la zona de interés. Puede detenerse, solicitar una observación aérea y esperar su regreso. Esto permite obtener una vista desde el punto donde surge una duda y reduce la distancia entre ambos sistemas.

Hay que estudiar el espacio libre para las hélices, la interacción con el LiDAR y la cámara del robot, la sujeción mientras el Husky circula y el margen de aterrizaje. La plataforma actual está pensada para el modelo simulado; no debe considerarse validada para un dron real de dimensiones distintas.

Un marcador en la plataforma puede ayudar a estimar la posición relativa de la cámara y el Husky durante el descenso. Para ello se necesitan calibración de cámara, tamaño físico del marcador, transformación cámara–dron, sincronización y un controlador capaz de reaccionar a pérdida de visión y latencia. La presencia del marcador no significa que el aterrizaje autónomo ya esté implementado.

### Base fija

El dron puede partir de un punto fijo, observar la parcela y enviar imágenes y pose al Husky o a un servidor. La base puede tener una plataforma más amplia y facilitar alimentación o cambio de batería. El Husky deja de transportar la aeronave, aunque aumenta la importancia de la cobertura de comunicación, el alcance de la misión y la identificación del marco común.

La base fija no obliga a mantener el peso o las dimensiones del dron actual. Permite considerar aeronaves empresariales o plataformas de investigación mayores, siempre que sus prestaciones se ajusten a las tareas y al presupuesto.

## Opciones reales identificadas

La comparación siguiente se basa en fichas oficiales consultadas el 5 de noviembre de 2026. Las dimensiones no siempre usan el mismo criterio: algunas incluyen hélices y otras no. La tabla no acredita que ninguno de los equipos pueda aterrizar autónomamente sobre el Husky mediante el SDK.

| Modelo | Masa y tamaño publicados | Características relevantes | Consideración para el proyecto |
|---|---|---|---|
| **DJI Mini 4 Pro** | Menos de 249 g con batería estándar; 298 × 373 × 101 mm desplegado **con hélices** | Cámara RGB; gimbal de tres ejes, inclinación controlable de −90° a +60° | Cercano a la masa simulada. Es el candidato inicial para evaluar transporte por el Husky. El ancho con hélices deja poco margen respecto a la plataforma de 40 cm. |
| **DJI Mavic 3 Enterprise / 3E** | DJI publica 915 g con hélices en la configuración estándar; también describe una configuración EU C1 de 865 g | Cámara RGB de 20 MP; opciones empresariales y RTK según configuración | Más grande y pesado. Puede ser adecuado desde una base fija o con rediseño de la plataforma. |
| **DJI Mavic 3M** | 951 g con hélices y módulo RTK; 347,5 × 283 × 139,6 mm desplegado **sin hélices** | RGB y cámara multiespectral; RTK; compatibilidad con Mobile SDK 5 | Opción para investigación agrícola multiespectral. Requiere estudiar coste, tamaño y base. La señal multiespectral por sí sola no diagnostica una enfermedad. |
| **Holybro X500 V2** | Distancia entre motores de 500 mm; la ficha indica 610 g para el kit, cuyo alcance exacto debe verificarse | Plataforma configurable con PX4 o ArduPilot; cámara y comunicaciones dependen del montaje | Mayor libertad de investigación y desarrollo de control, pero requiere integrar y validar más componentes. Su tamaño supera la escala del dron actual. |

La ficha oficial del Mini 4 Pro publica 298 × 373 mm **con hélices**. Frente a una plataforma de 400 × 400 mm, la diferencia en el lado más largo es de solo 27 mm en total. Es una comparación geométrica aproximada: hay que medir la huella de apoyo, la orientación, el barrido de hélices y los márgenes efectivos antes de diseñar una maniobra real.

## SDK y comunicaciones

DJI incluye actualmente el Mini 4 Pro, el Mavic 3 Enterprise y el Mavic 3M en la lista de productos compatibles con Mobile SDK. La documentación enumera combinaciones concretas de firmware y mando. La compatibilidad general **no confirma automáticamente** que estén disponibles todas las funciones necesarias para esta misión.

Antes de elegir un producto deben verificarse, para el modelo, mando y firmware concretos:

1. Acceso al flujo de vídeo desde una aplicación propia y su latencia.
2. Lectura de posición, orientación, tiempo y estado de la cámara/gimbal.
3. Comandos de vuelo y de inclinación de cámara admitidos.
4. Restricciones para control de descenso, proximidad a la plataforma y aterrizaje.
5. Compatibilidad con el dispositivo de desarrollo y permisos de uso del SDK.
6. Forma de transmitir datos a RoboComp/DSR y comportamiento ante pérdida de enlace.

Una arquitectura posible con Mobile SDK es: dron → mando y aplicación Android → red → ordenador del Husky o servidor → RoboComp. Debe comprobarse con una prueba real; el DDS de la simulación no demuestra que el dron comercial exponga directamente ese mismo canal.

## Qué adaptar en Webots si se selecciona un equipo

- Dimensiones desplegadas y masa con batería y accesorios concretos.
- Geometría de apoyo y espacio ocupado por las hélices.
- Campo de visión, resolución, orientación y latencia de cámara.
- Rangos y precisión del gimbal, con su transformación respecto al cuerpo.
- Ruido y disponibilidad de GNSS/IMU coherentes con el hardware y el entorno; los 5 cm actuales no se deben atribuir a un Mini sin mediciones.
- Separación explícita entre pose estimada por sensores y verdad del supervisor de Webots.
- Batería, duración, viento y límites de vuelo solo cuando se disponga de un modelo y datos para justificarlos.
- Plataforma, marcador y enlace de comunicaciones acordes con la alternativa de base elegida.

## Próximos pasos sugeridos

1. Mantener el escenario actual como punto de partida y registrar sus parámetros y limitaciones.
2. Comprobar disponibilidad y funciones concretas del SDK del Mini 4 Pro con el mando y firmware que se puedan adquirir.
3. Medir la plataforma del Husky y el dron real desplegado, incluida su huella de apoyo y el espacio de hélices.
4. Probar adquisición de imagen y pose con el dron estacionario, antes de integrar navegación o aterrizaje.
5. Comparar después las alternativas de base fija y base móvil con criterios de cobertura, autonomía, comunicaciones, complejidad y coste.
6. Para pruebas exteriores, revisar la normativa y las condiciones de operación aplicables con la autoridad competente. Un equipo de menos de 250 g con cámara puede requerir registro del operador según las condiciones aplicables.

## Referencias

- DJI, [especificaciones del Mini 4 Pro](https://www.dji.com/mini-4-pro/specs).
- DJI, [Mobile SDK: productos y versiones compatibles](https://developer.dji.com/doc/mobile-sdk-tutorial/en/).
- DJI Enterprise, [especificaciones del Mavic 3 Enterprise](https://enterprise.dji.com/mavic-3-enterprise/specs).
- DJI Enterprise, [Mavic 3M](https://enterprise.dji.com/mavic-3-m) y [especificaciones](https://enterprise.dji.com/mavic-3-m/specs).
- Holybro, [PX4 Development Kit X500 V2](https://holybro.com/products/px4-development-kit-x500-v2).
- EASA, [requisitos de formación y registro en categoría abierta](https://www.easa.europa.eu/en/the-agency/faqs/training-requirements-open-category).
- Cyberbotics, [modelo Mavic de Webots R2025a](https://github.com/cyberbotics/webots/tree/R2025a/projects/robots/dji/mavic).
