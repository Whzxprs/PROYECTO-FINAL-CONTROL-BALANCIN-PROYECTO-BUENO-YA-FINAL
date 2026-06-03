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

El firmware crea un AP WiFi:

- SSID: `balancin`
- Password: `1q2w3e4r`
- TCP: `192.168.4.1:4545`

Por TCP envia el paquete legacy de 9 bytes:

`0xAA, vd, v, theta, alpha, omega_l, omega_r, u_l, u_r`
