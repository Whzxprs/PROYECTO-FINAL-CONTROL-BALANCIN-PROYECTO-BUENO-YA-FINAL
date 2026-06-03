# Balancin ESP32 FreeRTOS

Proyecto ESP-IDF para ESP32 DevKit V1 de 30 pines.

La estructura toma como base el proyecto `Two-wheeled-inverted-pendulum-main`, pero usa:

- El mapeo base de `_TEORIA/MAPEO DE PINES.xlsx`, corregido con pruebas fisicas de sensores y encoders.
- Las constantes y ganancias de `_TEORIA/variables_fisicas.txt`.
- La ley de control del archivo `_TEORIA/embebidoRVCSclas.c`.
- Una tarea FreeRTOS de control a 10 ms.

## Pines

| Funcion | GPIO |
| --- | ---: |
| MPU6050 SDA | 21 |
| MPU6050 SCL | 22 |
| TCRT5000 L A0 | 35 |
| TCRT5000 R A0 | 34 |
| Encoder R fase A | 33 |
| Encoder R fase B | 32 |
| Encoder L fase A | 18 |
| Encoder L fase B | 19 |
| L298N pin 1 / ENA / PWM L | 14 |
| L298N pin 2 / IN1 L | 5 |
| L298N pin 3 / IN2 L | 4 |
| L298N pin 4 / IN3 R | 2 |
| L298N pin 5 / IN4 R | 15 |
| L298N pin 6 / ENB / PWM R | 27 |

GPIO 2, GPIO 5 y GPIO 15 son pines de arranque del ESP32. El proyecto respeta tu mapeo, pero si el ESP32 no inicia, revisa que el L298N no fuerce esos pines durante el reset.

Convencion ajustada con pruebas fisicas: PWM positivo debe avanzar. Motor L queda invertido, motor R no invertido; encoder L normal, encoder R invertido.

Los sensores de linea se leen fisicamente con ADC de 12 bits (`0..4095`), pero el control usa solo los 8 bits mas significativos (`adc8 = adc12 >> 4`, rango `0..255`), como se pidio en la convencion de pruebas. La GUI de pruebas muestra ambos valores para diagnostico.

## Compilar

```bash
idf.py set-target esp32
idf.py build
idf.py flash monitor
```

El firmware se conecta a una red WiFi existente en modo STA. El nombre de red, password, IP destino de la PC y puerto se cambian en `components/board/include/board_pins.h`:

```c
#define BALANCIN_WIFI_SSID "..."
#define BALANCIN_WIFI_PASS "..."
#define BALANCIN_UDP_TELEMETRY_HOST "192.168.4.2"
#define BALANCIN_UDP_TELEMETRY_PORT 5005
#define BALANCIN_UDP_COMMAND_PORT 5006
```

`BALANCIN_WIFI_SSID` y `BALANCIN_WIFI_PASS` son el nombre y password del WiFi al que se conectan la ESP32 y la PC. `BALANCIN_UDP_TELEMETRY_HOST` debe ser la IP de la PC dentro de esa misma red WiFi. En Windows se puede revisar con `ipconfig`, normalmente en el adaptador de WiFi como `Direccion IPv4`.

Para `../monitor/monitor.py` envia telemetria UDP v2 a 5 Hz:

`0xAB, version=2, seq, vd, v, theta, alpha, omega_l, omega_r, u_l, u_r, sl_raw, sr_raw, flags`

Uso sugerido del monitor:

1. Carga este firmware en el ESP32.
2. Conecta la PC al mismo WiFi configurado en `BALANCIN_WIFI_SSID`.
3. Ejecuta `python monitor.py` desde la carpeta `monitor`.
4. Deja el puerto en `5005` y pulsa `Escuchar`.

Los botones de ganancias/prueba del monitor envian comandos UDP a `5006` con formato `0xBB, id, float32_LE`. Este firmware procesa `0..5` ganancias, `6` velocidad de referencia, `8` referencia de inclinacion, `9` filtro complementario, `10` offset de angulo y `12..13` prueba de motores por rueda. `7` y `11` quedan reservados.
