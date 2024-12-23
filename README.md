# **Tequila Traction**  
### **Marcos Useros** | **Álvaro Quintana**

Este documento describe con detalle el desarrollo del sistema siguelíneas con comunicaciones IoT para la asignatura de **Sistemas Empotrados y de Tiempo Real**, utilizando **Arduino Uno** y **ESP32**.

[RESULTADO](https://www.dropbox.com/scl/fi/vf3ox0qhced8rwrcxt0r5/final_setr.mp4?rlkey=n12yoovoc0uieee8ikziyfw2y&st=q68ok6g2&dl=0)
---

## **1. Arduino Uno**

El **Arduino Uno** realiza las siguientes funciones principales:

- **Leer los sensores infrarrojos** para detectar la posición de la línea negra.
- **Implementar un controlador PD** que calcula la corrección necesaria en la dirección del coche.
- **Comunicar eventos al ESP32** a través del puerto serie, utilizando un protocolo definido por nosotros mismos.

El sistema está diseñado utilizando **FreeRTOS**, lo que proporciona:

- Exclusividad en el acceso a recursos.
- Un planificador eficiente para cumplir con los deadlines.
- La capacidad de añadir tareas críticas y asegurar la periodicidad de las mismas.

### **Tareas**

- **`task_follow_line`**:
  - **Prioridad**: 3.
  - **Frecuencia**: 20 ms.
  
- **`task_obstacle`**:
  - **Prioridad**: 2.
  - **Frecuencia**: 100 ms.
  
- **`task_ping`**:
  - **Prioridad**: 1.
  - **Frecuencia**: 4000 ms.

Tras varias pruebas, concluimos que con tres tareas (`Followline`, `Obstacle` y `Ping`), el sistema funciona de manera fluida y cumple perfectamente con las funcionalidades requeridas, evitando excesivos cambios de contexto o complejidad innecesaria.

### **Cálculo del Error**

El **error** representa la desviación del robot respecto a la línea. Este cálculo depende de las combinaciones de las lecturas de los tres sensores:

- **Línea detectada a la izquierda**: Si el sensor izquierdo detecta la línea y el derecho no, significa que el robot está desviado hacia la derecha.
  - **Error**: -1.0 (corrección hacia la izquierda).
  
- **Línea detectada a la derecha**: Si el sensor derecho detecta la línea y el izquierdo no, significa que el robot está desviado hacia la izquierda.
  - **Error**: 1.0 (corrección hacia la derecha).
  
- **Línea detectada al centro**: Si solo el sensor central detecta la línea, el robot está perfectamente alineado.
  - **Error**: 0.0.
  
- **Línea no detectada**: Si ningún sensor detecta la línea, significa que el robot la ha perdido. En este caso, se añade un error mayor en la dirección en la que se perdió la línea, permitiendo al robot corregir rápidamente.

Una vez calculado el error, utilizamos un controlador **Proporcional-Derivativo (PD)** para ajustar las velocidades de los motores en función del error actual (P) y de la velocidad con la que está cambiando ese error (D). Inicialmente, se consideró incluir una componente **Integral (I)**, pero debido a la naturaleza del error y la alta frecuencia de iteraciones por segundo, resultó difícil acumular el error de forma efectiva. Por esta razón, el controlador se implementó solo con las componentes proporcional y derivativa.

Las velocidades de los motores izquierdo y derecho se ajustan en función de la corrección calculada.

### **Detección de Obstáculos**

La tarea de **detección de obstáculos** utiliza el sensor de ultrasonidos para medir la distancia al obstáculo que indica el final del circuito. Si la distancia detectada es menor a 25 cm, se reduce la velocidad base del controlador PD, lo que permite una desaceleración progresiva y evita un frenado brusco que podría provocar un derrape. Cuando la distancia es inferior a 7 cm, los motores se detienen por completo para evitar una colisión. Además, esta tarea envía al ESP32 las instrucciones correspondientes, notificando la detección del obstáculo con el mensaje **"OBSTACLE_DETECTED"** y marcando el final de la vuelta con **"END_LAP"**.

### **Tarea Ping**

La tarea **ping** se encarga de notificar al ESP32, a través del puerto serie, que es necesario realizar un ping al servidor.

---

## **2. ESP32**

El **ESP32** realiza las siguientes funciones principales:

- **Conexión a la red WiFi** para enviar y recibir mensajes.
- **Conexión al servidor MQTT** para publicar mensajes relacionados con las acciones del robot.
- **Comunicación con el Arduino Uno** a través del puerto serie para recibir comandos y enviar al servidor.

El sistema utiliza las bibliotecas **WiFi** y **Adafruit MQTT** para manejar la conexión y comunicación con el servidor, con el siguiente flujo general:

1. **Conexión a la red WiFi**: El ESP32 se conecta a la red definida en el código utilizando el SSID y la contraseña proporcionados.
2. **Conexión al servidor MQTT**: Una vez conectado a la red, el ESP32 se conecta al servidor MQTT en la dirección IP especificada.
3. **Publicación de mensajes**: El ESP32 publica mensajes en el topic definido para informar sobre las acciones del robot (por ejemplo, **"START_LAP"**, **"OBSTACLE_DETECTED"**, etc.).

### **2.1 Arduino ACK**

El ESP32 espera recibir un mensaje de confirmación (**#ACK**) del Arduino a través del puerto serie antes de proceder con la ejecución de las operaciones. Esto garantiza que la comunicación entre el ESP32 y el Arduino esté establecida correctamente.

### **2.2 Mensaje Base**

El mensaje que se envía al servidor está estructurado en formato **JSON**, lo cual permite una fácil interpretación de los datos. Se define un mensaje base que contiene los valores constantes como el nombre del equipo y su ID. Este mensaje base se concatena con la acción que se desea ejecutar y, en algunos casos, un valor asociado.

### **2.3 Acción y Valor**

Dependiendo de la acción que se reciba, el mensaje se ajusta para incluir el campo correspondiente con su valor. Si la acción requiere un valor, este se agrega de acuerdo al tipo de dato (entero, flotante, largo).

### **2.4 Envío MQTT**

El ESP32 se conecta a un servidor MQTT para publicar las acciones y valores que recibe desde el puerto serie. Una vez que el JSON es generado, se publica utilizando la biblioteca **Adafruit MQTT**.
