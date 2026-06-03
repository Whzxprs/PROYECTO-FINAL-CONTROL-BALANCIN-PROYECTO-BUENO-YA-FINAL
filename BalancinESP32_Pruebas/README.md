# Balancin ESP32 Pruebas

Proyecto ESP-IDF independiente para diagnosticar el balancin sin tocar el firmware principal.

Sirve para:

- Probar giros y convenciones de motores L298N con voltaje/PWM firmado.
- Ver si hay que invertir motor izquierdo/derecho.
- Leer encoders y comprobar si el signo coincide con el giro positivo.
- Medir sensores infrarrojos TCRT5000 en ADC 12 bits y ADC 8 bits.
- Calibrar rangos de blanco/negro y sugerir umbrales.
- Leer el MPU6050 con crudos signed 16 bits, acelerometro, giroscopio y filtro complementario en grados.

## Pines usados

Base tomada de `_TEORIA/MAPEO DE PINES.xlsx`, con correcciones encontradas en prueba fisica.

| Funcion | GPIO |
| --- | ---: |
| MPU SDA | 21 |
| MPU SCL | 22 |
| TCRT5000 L A0 | 35 |
| TCRT5000 R A0 | 34 |
| Encoder R fase A | 33 |
| Encoder R fase B | 32 |
| Encoder L fase A | 18 |
| Encoder L fase B | 19 |
| L298N ENA / PWM L | 14 |
| L298N IN1 L | 5 |
| L298N IN2 L | 4 |
| L298N IN3 R | 2 |
| L298N IN4 R | 15 |
| L298N ENB / PWM R | 27 |

GPIO 2, GPIO 5 y GPIO 15 son pines de arranque del ESP32. Si no inicia, revisa que el L298N no los fuerce durante reset.

## Convenciones corregidas en prueba fisica

- `+V` debe significar avance. Como `+3 V` hacia atras movia ambas ruedas, quedaron invertidos los dos sentidos respecto a la primera prueba: motor L invertido, motor R no invertido.
- Los encoders estaban intercambiados: el encoder fisico izquierdo va en GPIO18/GPIO19 y el derecho en GPIO33/GPIO32.
- Con la convencion nueva de avance positivo, el encoder izquierdo queda normal y el derecho queda invertido.
- Los sensores IR estaban intercambiados: izquierdo en GPIO35 y derecho en GPIO34.

## Compilar y cargar

Desde esta carpeta:

```bash
idf.py set-target esp32
idf.py build
idf.py flash
```

Luego puedes usar el monitor de ESP-IDF:

```bash
idf.py monitor
```

O el monitor ligero incluido:

```bash
pip install -r requirements.txt
python tools/monitor_pruebas.py --port COM5
```

Tambien hay una interfaz grafica sin PyQt, hecha con `tkinter`:

```bash
python tools/gui_pruebas.py --port COM5
```

Puedes listar puertos con:

```bash
python tools/monitor_pruebas.py --list
```

## Flujo sugerido de pruebas

1. Verifica pines y estado:

```text
pins
status
```

2. Apaga el stream si quieres leer mas facil:

```text
stream off
```

3. Prueba cada motor con pulsos cortos:

```text
pulse l 3 700
pulse r 3 700
```

El comando positivo deberia ser "adelante" para cada rueda. Si una rueda queda al reves, cambia la inversion en RAM y repite:

```text
inv motor l 1
inv motor r 0
```

4. Prueba motor mas encoder:

```text
testmotor l 3 800
testmotor r 3 800
```

Si la rueda giro adelante pero el delta de su encoder salio negativo:

```text
inv enc l 1
inv enc r 1
```

5. Lee sensores infrarrojos:

```text
ir
ircal blanco 3000
ircal negro 3000
threshold
```

La salida te dira si para detectar negro conviene usar `raw > umbral` o `raw < umbral`, segun tus lecturas reales.

6. Lee MPU:

```text
mpu zero 500
mpu
```

`acc_pitch` y `acc_roll` vienen del acelerometro. `gyro_*` viene integrado del giroscopio. `comp_*` usa filtro complementario.

## Resolucion

- IR/TCRT5000: el ESP32 los lee en ADC de 12 bits, rango `0..4095`, pero el valor que debe usarse para el control es el de 8 bits mas significativos: `adc8 = adc12 >> 4`, rango `0..255`.
- MPU6050: los crudos del acelerometro y giroscopio son signed 16 bits. Con la configuracion actual equivalen a +/-2 g y +/-250 grados/s.
- Firmware final: el control usa los sensores IR ya reducidos a 8 MSB, para mantener la convencion pedida por el profesor.

## Comandos rapidos

```text
help
stream on
rate 100
motor b 2.5 2.5
pwm l 80
stop
enc 1000
mpu
```

Los comandos de motor tienen auto-stop por seguridad. El tiempo por defecto es 2500 ms y se cambia con:

```text
timeout 1500
```
