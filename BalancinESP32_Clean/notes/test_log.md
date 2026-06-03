# Test Log

## Etapa 0 - Inventario

- Fecha:
- Hardware:
- ESP32:
- Driver de motores:
- MPU6050:
- Encoders:
- Sensores IR:
- Bateria/fuente:

## Etapa 1 - Hardware minimo

Fecha: 2026-06-03

### Drivers (status)
- motor=1, line=1, encoder=1, mpu=1 — todos inicializaron sin error.

### Sensores IR
- Superficie clara: raw_l=3044, raw_r=3773
- Al aire / superficie oscura: raw_l=58, raw_r=64
- Rango util ~58–3773 (12 bits, atenuacion 12 dB). Ambos sensores responden correctamente.

### MPU6050
- MPU montado de cabeza. Flags corregidos en main.c:
  - MPU_INVERT_AX=1, MPU_INVERT_AY=0, MPU_INVERT_AZ=1
  - MPU_INVERT_GX=1, MPU_INVERT_GY=1, MPU_INVERT_GZ=1
- Calibracion gyro (zero 500): off_gx=1.1528, off_gy=0.4808, off_gz=-0.0954 dps
- Vertical: acc_roll=1.9°, acc_pitch=7.5° (offset mecanico, se corrige con alphad en Etapa 2)
- Adelante ~45°: acc_pitch=+45.8° OK
- Atras ~45°: acc_pitch=-38.9° OK
- gy positivo al caer hacia adelante OK

### Motores (pulse x 2.0V 500ms)
- Motor izquierdo: gira hacia adelante con PWM positivo. MOTOR_LEFT_INVERT=1
- Motor derecho: gira hacia adelante con PWM positivo. MOTOR_RIGHT_INVERT=0

### Encoders (enc 1000, rueda girada a mano)
- Izquierdo adelante: delta_l=+513  Izquierdo atras: delta_l=-266  OK
- Derecho adelante:   delta_r=+712  Derecho atras:   delta_r=-834  OK
- ENCODER_LEFT_INVERT=0, ENCODER_RIGHT_INVERT=1

### Conclusion
Todos los sensores y actuadores verificados con signo correcto. Listo para Etapa 2.

## Etapa 2 - Balanceo

- Prueba:
- Resultado:
- Cambios:

## Etapa 3 - Velocidad

- Prueba:
- Resultado:
- Cambios:

## Etapa 4 - Linea

- Prueba:
- Resultado:
- Cambios:

## Etapa 5 - Monitor

- Prueba:
- Resultado:
- Cambios:
