# Tequila Traction
### Marcos Useros Álvaro Quintana
---
Este documento describe con detalle el desarrollo del sistema siguelíneas con comunicaciones IoT para la asignatura de Sistemas Empotrados y de Tiempo Real, utilizando Arduino Uno y ESP32.
## 1. Estructura del Proyecto
## 1.1 Arduino Uno

El Arduino Uno realiza las siguientes funciones principales:

- **Leer los sensores infrarrojos** para detectar la posición de la línea negra.
- **Implementar un controlador PD** que calcula la corrección necesaria en la dirección del coche.
- **Comunicar eventos al ESP32** a través del puerto serie, utilizando un protocolo definido por nosotros mismos.

El sistema está diseñado utilizando **FreeRTOS**, lo que proporciona:
- Exclusividad en el acceso a recursos.
- Un planificador (scheduler) eficiente para cumplir con deadlines.
- La capacidad de añadir tareas críticas y asegurar la periodicidad de las mismas.

Tras varias pruebas, concluimos que con tres tareas (`Followline`, `Obstacle` y `Ping`), el sistema funciona de manera fluida y cumple perfectamente con las funcionalidades requeridas, evitando excesivos cambios de contexto o complejidad innecesaria.

Cálculo del error: El error representa la desviación del robot respecto a la línea. Este cálculo depende de las combinaciones de las lecturas de los tres sensores:

* Línea detectada a la izquierda: Si el sensor izquierdo detecta la línea y el derecho no, significa que el robot está desviado hacia la derecha.
      Error: -1.0 (corrección hacia la izquierda).

*  Línea detectada a la derecha: Si el sensor derecho detecta la línea y el izquierdo no, significa que el robot está desviado hacia la izquierda.
      Error: 1.0 (corrección hacia la derecha).

 * Línea detectada al centro: Si solo el sensor central detecta la línea, el robot está perfectamente alineado.
      Error: 0.0.

*  Línea no detectada: Si ningún sensor detecta la línea, significa que el robot la ha perdido. En este caso, se añade un error mayor en la dirección en la que se perdió la línea, permitiendo al robot corregir rápidamente:
    ```
    error = (last_err <= 0) ? -3.1 : 3.1;
    ```
Una vez calculado el error, utilizamos un controlador Proporcional-Derivativo (PD) para ajustar las velocidades de los motores en función del error actual (P) y de la velocidad con la que está cambiando ese error (D). Inicialmente, se consideró incluir una componente Integral (I), pero debido a la naturaleza del error y la alta frecuencia de iteraciones por segundo, resultó difícil acumular el error de forma efectiva. Por esta razón, el controlador se implementó solo con las componentes proporcional y derivativa.

`correction = Kp * error + Kd * dError`

Las velocidades de los motores izquierdo y derecho se ajustan en función de la corrección calculada:

```C
speed_left = base_speed + correction
speed_right = base_speed - correction
```
La tarea de detección de obstáculos utiliza el sensor de ultrasonidos para medir la distancia al obstáculo que indica el final del circuito. Si la distancia detectada es menor a 25 cm, se reduce la velocidad base del controlador PD, lo que permite una desaceleración progresiva y evita un frenado brusco que podría provocar un derrape. Cuando la distancia es inferior a 7 cm, los motores se detienen por completo para evitar una colisión. Además, esta tarea envía al ESP las instrucciones correspondientes, notificando la detección del obstáculo con el mensaje "OBSTACLE_DETECTED" y marcando el final de la vuelta con "END_LAP".

La tarea ping se encarga de notificar al ESP, a través del puerto serie, que es necesario realizar un ping al servidor.

#### 1.2 ESP32
* Recibir datos desde el Arduino Uno por el puerto serie.
* Publicar eventos y estados en un servidor MQTT mediante la librería Adafruit_MQTT.

