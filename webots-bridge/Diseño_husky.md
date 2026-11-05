# Dimensiones de diseño: Husky A300 y A200

Este documento reúne dimensiones publicadas por Clearpath Robotics para orientar el diseño del modelo de Webots, sus sensores y posibles accesorios como una plataforma para el dron. Las medidas corresponden a las bases comerciales; los sensores, mástiles y accesorios instalados pueden aumentar las dimensiones totales.

## Dimensiones exteriores

| Medida | Husky A300 | Husky A200 |
|---|---:|---:|
| Largo | 990 mm (0,990 m) | 990 mm (0,990 m) |
| Ancho | 698 mm (0,698 m) | 670 mm (0,670 m) |
| Alto de la base | 372 mm (0,372 m) | 390 mm (0,390 m) |
| Altura libre al suelo | 130 mm (0,130 m) | 130 mm (0,130 m) |
| Distancia entre ejes de ruedas | 512 mm (0,512 m) | 512 mm (0,512 m) |
| Vía: separación entre ruedas izquierda y derecha | 566 mm (0,566 m) | 555 mm (0,555 m), según el manual antiguo |
| Diámetro nominal de rueda | 336 mm (0,336 m) | 330 mm (0,330 m), según la ficha antigua |

Las dimensiones exteriores del A300 y A200 proceden de la [comparación oficial de Clearpath](https://clearpathrobotics.com/husky-spec-comparison/). El detalle de ruedas y ejes del A300 procede del [manual del A300](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a300/user_manual_husky/). Para el A200 se han consultado el [manual antiguo](https://manuals.plus/m/1f32a1f0e1d6c5d42f66833476b8e5edd66345bc0a137a9157af5d234e6bfff3) y la [ficha antigua de Clearpath](https://www.clearpathrobotics.com/wp-content/uploads/2013/08/HUSKY_DATA_SHEET-v2_editeda.pdf).

**Discrepancia de altura del A300:** el manual técnico y la comparación A200/A300 indican **372 mm**, mientras que la [página comercial del A300](https://clearpathrobotics.com/husky-a300-unmanned-ground-vehicle-robot/) indica **381 mm**. Antes de fabricar un soporte o usar una altura como referencia exacta, confirmar el plano y la configuración del robot físico concreto.

## Comparación de tamaño

En planta, las bases ocupan aproximadamente:

```text
A300: 99,0 cm de largo × 69,8 cm de ancho
A200: 99,0 cm de largo × 67,0 cm de ancho
```

El A300 es 28 mm más ancho que el A200 según esas fichas. Estas medidas son del contorno exterior del vehículo: **no equivalen al tamaño libre de la placa superior ni a la zona disponible para aterrizaje**.

## Relación con la simulación

El modelo utilizado por el proyecto está en [`webots-agri/protos/HuskyA300.proto`](../webots-agri/protos/HuskyA300.proto). Sus comentarios indican que parte de dimensiones del A200 y que los valores que puedan diferir en el A300 deben confirmarse. Por tanto, el nombre del PROTO no garantiza que toda su geometría reproduzca el A300 comercial.

Antes de ajustar el modelo se deben comparar, como mínimo:

- Contorno exterior y huella utilizada por el planificador.
- Distancia entre ejes, vía y radio efectivo de rueda.
- Altura del chasis y distancia libre al suelo.
- Posición de la placa superior, mástiles y sensores respecto al marco del robot.
- Geometría de colisión y geometría visual.

Cualquier cambio debe mantener coherentes las transformaciones, el filtrado de puntos del propio robot y la huella de navegación. Las dimensiones aquí anotadas **no modifican automáticamente** el modelo de Webots ni la configuración del puente.

## Plataforma del dron

La documentación actual de la simulación describe una plataforma de **400 × 400 mm** sobre el Husky. Cabe dentro de la huella exterior de cualquiera de las dos bases en vista superior, pero eso no demuestra que exista espacio de montaje suficiente ni que el dron pueda aterrizar con seguridad.

Para un diseño físico se debe verificar el plano de la placa superior, los puntos de fijación, la posición del LiDAR y las cámaras, la huella de apoyo del dron y el barrido de sus hélices. También debe comprobarse que el soporte no oculte zonas importantes de los sensores ni interfiera con el robot al moverse.

Referencia del escenario actual: [`webots-agri/DRON_INSPECCION.md`](../webots-agri/DRON_INSPECCION.md).

## Fuentes

- Clearpath Robotics, [Husky A300 User Manual](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a300/user_manual_husky/).
- Clearpath Robotics, [comparación de especificaciones Husky A300 y A200](https://clearpathrobotics.com/husky-spec-comparison/).
- Clearpath Robotics, [página comercial del Husky A300](https://clearpathrobotics.com/husky-a300-unmanned-ground-vehicle-robot/).
- Clearpath Robotics, [ficha técnica antigua del Husky](https://www.clearpathrobotics.com/wp-content/uploads/2013/08/HUSKY_DATA_SHEET-v2_editeda.pdf).
