# Balancin ESP32 Clean

Proyecto limpio para reconstruir el balancin paso a paso, sin mezclar parches de
los proyectos anteriores.

## Objetivo

Construir el firmware por etapas verificables:

1. Hardware minimo: pines, motores, encoders, MPU6050 y sensores IR.
2. Balanceo quieto: usar solo MPU6050, encoders y motores.
3. Velocidad: agregar control de velocidad longitudinal.
4. Seguimiento de linea: agregar sensores IR y control de orientacion.
5. Monitor UDP: telemetria y ajuste de parametros en vivo.

## Carpetas

- `theory/`: aqui van los archivos de teoria y referencia matematica.
- `notes/`: bitacora de pruebas, decisiones y valores encontrados.
- `firmware/`: proyecto ESP-IDF limpio. La etapa 1 usa un solo `main/main.c`
  bien comentado para que se pueda leer de arriba a abajo.
- `tools/`: scripts de apoyo para revisar logs, telemetria o datos guardados.

## Regla de trabajo

Cada etapa debe tener:

- Una meta medible.
- Un comando o prueba para verificarla.
- Un punto claro donde modificar constantes.
- Un registro en `notes/test_log.md`.

No se avanza a la siguiente etapa hasta que la anterior responda bien en el
hardware.

## Etapa actual

`firmware/main/main.c` es un diagnostico de hardware por serial. No contiene
balanceo todavia.

Comandos principales:

```text
help
pins
status
ir
mpu
zero 500
pulse l 2.0 500
pulse r 2.0 500
enc 1000
stream on
stream off
```
