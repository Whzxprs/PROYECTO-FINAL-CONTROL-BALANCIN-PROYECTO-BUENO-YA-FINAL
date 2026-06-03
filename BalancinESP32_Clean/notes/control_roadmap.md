# Control Roadmap

## Etapa 1: hardware minimo

Meta: confirmar que cada medicion y cada actuador funciona con signo conocido.

Pruebas:

- Motor izquierdo: PWM positivo debe avanzar.
- Motor derecho: PWM positivo debe avanzar.
- Encoder izquierdo: avanzar debe sumar positivo.
- Encoder derecho: avanzar debe sumar positivo despues de aplicar inversion.
- MPU6050: inclinar hacia adelante debe cambiar `alpha` con signo esperado.
- IR izquierdo/derecho: confirmar blanco/negro y rango ADC.

## Etapa 2: balanceo quieto

Control inicial:

- Estado principal: `alpha`.
- Actuadores: voltaje/PWM comun a ambas ruedas.
- Referencias: `alphad = 0`, `vd = 0`, `theta = 0`.
- No usar sensores de linea todavia.

La meta no es seguir linea ni avanzar. La meta es que si se inclina, los motores
corrijan en la direccion correcta.

## Etapa 3: velocidad

Agregar encoders para estimar:

- `omegal`
- `omegar`
- `v = (omegar + omegal) * R / 2`

Despues agregar PI de velocidad con `vd`.

## Etapa 4: seguimiento de linea

Agregar sensores IR:

- `theta` desde diferencia derecha-izquierda.
- `thetap` desde diferencia de velocidades de ruedas.
- Control de orientacion `taua`.

## Etapa 5: monitor

Agregar UDP:

- Telemetria primero.
- Comandos despues.
- Las ganancias deben tener defaults razonables en firmware y tambien poderse
  ajustar desde el monitor.
