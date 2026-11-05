# Especificación de la Cámara del Dron de Inspección (`camera_dron.md`)

Este documento define la **cámara seleccionada para la simulación en Webots** y detalla las **opciones de hardware comercial** para su futura implementación física, considerando que el mini dron solo portará hardware ligero de captura y transmisión de vídeo hacia el Husky / estación base.

---

## 1. Cámara Seleccionada para Webots (Configuración de Trabajo)

### 1.1 El Problema del Ángulo Estrecho Actual
En [`InspectionDrone.proto`](/home/nosl3n/robocomp/AiGB_UEx_Navigation_v2/webots-agri/protos/InspectionDrone.proto#L27) y [`drone.md`](/home/nosl3n/robocomp/AiGB_UEx_Navigation_v2/webots-drone-bridge/src/drone.md#L39), la cámara simulada tiene:
$$\text{fieldOfView} = 1{,}04719755\text{ rad} \approx 60^\circ$$
A una distancia cercana de la copa ($1{,}0\text{–}1{,}5\text{ m}$), un ángulo de $60^\circ$ solo permite ver un recorte muy reducido del árbol ($\sim 1\text{ m}$ de cobertura), perdiendo la visión de conjunto de la copa, las ramas vecinas y los frutos circundantes.

### 1.2 Configuración Oficial de Trabajo: Lente Gran Angular $130^\circ$
Para resolver esto, se adopta como estándar de simulación una **cámara gran angular de $130^\circ$ FOV** (equivalente óptico a un sensor de $1/2.3''$ con lente focal corta de $2{,}1\text{ mm}$ en montura M12).

* **Ángulo de visión (FOV):** **$130^\circ$** ($2{,}2689\text{ rad}$). A solo $1\text{ m}$ de la copa, el plano observado cubre **$3{,}1\text{ m}$ de diámetro**, capturando la copa completa del manzano en una sola toma cenital u oblicua.
* **Resolución:** **$1280 \times 720$ (720p HD)** a 30/60 fps. Excelente compromiso entre detalle visual para detección de manzanas y baja latencia de procesamiento.
* **Enfoque cercano (`near`):** $0{,}02\text{ m}$ ($2\text{ cm}$), evitando cortes si el dron se aproxima a ramas altas.
* **Servo de orientación:** Mantiene el eje `camera pitch` existente en [`InspectionDrone.proto`](/home/nosl3n/robocomp/AiGB_UEx_Navigation_v2/webots-agri/protos/InspectionDrone.proto) (desde frontal $0^\circ$ hasta cenital $90^\circ$ hacia abajo).

### 1.3 Bloque PROTO para Webots (`InspectionDrone.proto`)

```vrml
# Sustituir en la línea 27 de InspectionDrone.proto:
field MFNode cameraSlot [
  Camera {
    name "camera"
    width 1280
    height 720
    fieldOfView 2.2689                  # 130° Gran Angular
    near 0.02
    lensRadialCoefficients [ -0.12 0.03 ] # Curvatura óptica realista de lente corta
  }
]
```

---

## 2. Flujo de Procesamiento en el Husky (Arquitectura Off-Board)

Siguiendo la decisión de que el dron sea lo más ligero posible, la arquitectura reparte las tareas así:

```
[ MINI DRON ] (Carga útil ultra ligera < 30 g)
   ├── Mini cámara gran angular (130° FOV)
   └── Módulo de transmisión de vídeo digital / WiFi
                     │
                     │  Enlace inalámbrico (RTSP / HDMI / V4L2)
                     ▼
[ HUSKY A300 / ESTACIÓN BASE ] (GPU RTX, batería de alta capacidad)
   ├── 1. Recepción del stream de vídeo (/dev/video* o DDS)
   ├── 2. Desdistorsión en tiempo real (OpenCV cv2.undistort / remap en GPU)
   ├── 3. Inferencia de YOLO26 en retina (detección de manzanas y dosel)
   └── 4. Proyección 3D de detecciones al grafo DSR (plant_concept)
```

### Corrección de Distorsión en Software (OpenCV en el Husky):
```python
import cv2
import numpy as np

# Calibración intrínseca de la lente gran angular:
# K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
# D = [k1, k2, p1, p2, k3] (coeficientes radiales y tangenciales)

# Mapa de rectificación precalculado una sola vez:
map1, map2 = cv2.initUndistortRectifyMap(K, D, None, new_K, (1280, 720), cv2.CV_32FC1)

# En cada fotograma recibido del dron:
frame_rectificado = cv2.remap(frame_raw, map1, map2, interpolation=cv2.INTER_LINEAR)
# frame_rectificado entra a YOLO26 sin deformación de barril
```

---

## 3. Matriz Comparativa de Opciones de Hardware Físico

A continuación se resumen las opciones comerciales evaluadas para el dron, ordenadas por viabilidad técnica y peso:

| Opción | Hardware | FOV Lente | Peso Total | Conexión / Salida | Ventajas principales | Desventajas / Retos |
|---|---|---|---|---|---|---|
| **Opción A (Recomendada - Máxima ligereza)** | **Caddx Walksnail Avatar Mini 1S Lite** o **RunCam Link / DJI O3 Lite** | **$155^\circ$ FOV** | **$\sim 16\text{–}25\text{ g}$** | Enlace digital inalámbrico 1080p 60fps | Cero peso en el dron; calidad de imagen profesional estabilizada; latencia ultrabaja ($< 22\text{ ms}$). | Requiere receptor HDMI/USB dedicado en la base del Husky. |
| **Opción B (Modular con lentes intercambiables)** | **Arducam IMX477 / IMX519 con Montura M12** + micro transmisor WiFi | **$120^\circ\text{–}140^\circ$ FOV** (Lente M12 2.1 mm) | **$\sim 18\text{–}30\text{ g}$** | MIPI CSI a micro-emisor o USB | Permite cambiar lentes M12 desenroscando; sensor de $12{,}3\text{ MP}$ con gran rango dinámico para sol y sombras. | Requiere encapsulado ligero para proteger el cable plano MIPI de vibraciones. |
| **Opción C (Económica / WiFi IP directo)** | **ESP32-S3 Cam con Lente Gran Angular M12** | **$130^\circ$ FOV** | **$\sim 12\text{ g}$** | WiFi RTSP / HTTP MJPEG directo | Extremadamente barata; transmite directamente por la red WiFi del Husky sin hardware receptor adicional. | Menor tasa de frames ($15\text{–}20\text{ fps}$ en 720p) y menor rango dinámico en luz solar intensa. |
| **Opción D (Industrial Plug & Play)** | **e-con Systems e-CAM24_CUNX / e-CAM27** | **$125^\circ\text{–}135^\circ$ FOV** | **$\sim 28\text{–}35\text{ g}$** | USB-C / MIPI industrial | Carcasa metálica sellada contra polvo y vibraciones; balance de blancos y HDR automático calibrado de fábrica. | Mayor coste; el peso se acerca al límite superior para un mini dron clase 250 g. |

---

## 4. Conclusión y Recomendación de Adopción

1. **En Webots (Inmediato):** Se adopta la **cámara de $130^\circ$ FOV** configurada en [`InspectionDrone.proto`](/home/nosl3n/robocomp/AiGB_UEx_Navigation_v2/webots-agri/protos/InspectionDrone.proto), permitiendo realizar vuelos de inspección cenital y lateral a corta distancia cubriendo la copa completa.
2. **En Hardware Futuro:**
   * La opción más robusta y liviana es un **sistema de transmisión digital HD con cámara de $130^\circ\text{–}155^\circ$ (Opción A o B)**, dejando el peso de cómputo y corrección en la GPU del Husky A300.
