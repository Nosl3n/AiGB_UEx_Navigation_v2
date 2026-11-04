# Mundo de inspección aérea: hardware

Desde la raíz del repositorio:

```bash
./apple_dron.bash
```

El lanzador es una copia de `apple_field.bash` que abre `webots-agri/worlds/dos_fields_dron.wbt`. Conserva el arranque de los componentes existentes. No modifica sus fuentes ni sus configuraciones. `--help`, `--dry-run` y `--stop` conservan el comportamiento del lanzador original. No lanzar ambos mundos simultáneamente con los mismos puertos y componentes.

El mundo nuevo conserva el terreno y los árboles de `dos_fields.wbt`. Usa una variante de hardware `HuskyDroneCarrier.proto`, copiada del original y con una plataforma añadida. `HuskyA300.proto`, el mundo original y el controlador/bridge del Husky permanecen intactos. La variante mantiene los motores, sensores, parámetros, nombre y controlador del Husky original. El soporte puede producir oclusiones físicas reales; no se ha modificado el filtrado de sensores del robot.

## Hardware añadido

- `InspectionDrone.proto`: derivado del Mavic2Pro oficial de Cyberbotics R2025a, con hélices físicas, GPS, IMU, giróscopo y brújula.
- Se han sustituido las articulaciones de yaw y roll de cámara por uniones rígidas. Solo queda `camera pitch` y su encoder `camera pitch sensor`.
- Cámara RGB de 1280 × 720, FOV horizontal de 60 grados. Pitch 0: frontal (+X local del dron). Pitch pi/2: inferior (-Z local). La orientación global también depende de la actitud del dron.
- Servo limitado a [0, pi/2], velocidad máxima 1 rad/s y par máximo 0.5 N·m. Son parámetros de simulación, no especificaciones verificadas de un servo comercial.
- GPS con desviación configurada de 0.05 m y correlación 0.9. En este mundo sus coordenadas son WGS84, como indica `WorldInfo`.
- Emisor `inspection telemetry`, canal 42, y receptor `inspection command`, canal 43, disponibles para una integración futura. No hay un protocolo de imágenes o conexión RoboComp implementado.
- Plataforma solidaria al Husky de 0.70 × 0.70 m, centrada en (0, 0.35, 0.65) en su marco, con soporte, geometría de colisión y masa añadida de 0.8 kg. El centro de su cara superior queda a z=0.6625 m respecto al origen del Husky.
- Marcador ArUco `DICT_4X4_50`, ID 23. Lado exterior negro: **0.24 m**; textura con margen blanco: 0.32 m. Centro respecto al Husky: (0, 0.51, 0.663). Usar el lado negro, no el tamaño de la textura, para estimar pose.
- El dron empieza físicamente apoyado sobre la plataforma, orientado hacia el frente del Husky. No está soldado al robot: esta versión está pensada para la prueba con Husky detenido. No incluye anclaje de transporte para conducir con el dron encima.

## Prueba manual de cámara

Seleccionar el dron en Webots y enfocar la vista 3D para que reciba el teclado:

- `F`: frontal, pitch 0.
- `D`: inferior, pitch pi/2.

El controlador local `inspection_hardware` activa cámara, encoder y sensores, y mantiene las cuatro hélices apagadas. Al arrancar orienta la cámara hacia abajo. La imagen puede verse en la ventana de dispositivos del dron. Esperar a que termine el movimiento del servo para interpretar una captura con la pose seleccionada.

**No implementa despegue, estabilización de vuelo, retorno, aterrizaje autónomo ni un SDK del fabricante.** El marcador y los sensores preparan esas funciones; disponer del modelo de Webots no proporciona el piloto automático comercial. Tampoco se ejecuta la detección ArUco durante el funcionamiento normal en esta entrega de hardware.

No usar `--route` para la prueba estática de inspección. El lanzador conserva esa opción heredada, pero no existe un bloqueo nuevo del movimiento del Husky ni retención del dron sobre la plataforma.

## Procedencia

Modelo y meshes derivados de:
https://github.com/cyberbotics/webots/tree/R2025a/projects/robots/dji/mavic

Se conserva la atribución y referencia a la licencia de activos Webots en `InspectionDrone.proto`. Los meshes están en `protos/Mavic2Pro/meshes`; la textura de hélice procede del mismo release de Webots. El marcador se genera determinísticamente con OpenCV y no contiene datos de percepción reales.

## Verificación

Se probó en Webots un escenario aislado con la plataforma y el dron: el encoder alcanza frontal e inferior, no existen dispositivos móviles de yaw/roll de cámara y OpenCV identifica el marcador 23 desde la imagen inferior renderizada. La elevación y fijación de pose de ese escenario son exclusivamente un mecanismo de prueba externo, no un controlador de vuelo incluido.

También se verifica la carga del mundo completo con controladores temporales de prueba, sin arrancar RoboComp, y el apoyo del dron sobre el Husky. La sintaxis del lanzador se comprueba con `bash -n`. Estos ensayos no validan vuelo ni aterrizaje autónomo.
