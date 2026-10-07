# Tareas 9B — Control de Robots

Este README describe de forma general el código de la carpeta `tareas9B`, su propósito y cómo ejecutarlo.

## Objetivo

El código en `tareas9B` está orientado a prácticas de **control de robots**, incluyendo modelado, simulación y/o implementación de controladores para seguimiento de trayectoria o regulación de estado.

## Estructura sugerida de la carpeta

Dentro de `tareas9B` normalmente encontrarás archivos como:

- `main.*`: script principal para correr la práctica.
- `robot.*`: funciones o clases del modelo del robot (cinemática/dinámica).
- `controlador.*`: lógica del controlador (por ejemplo PID, estado, etc.).
- `simulacion.*`: integración numérica y lazo de simulación.
- `utils.*`: funciones auxiliares (gráficas, métricas, lectura de datos).

> Nota: los nombres pueden variar según tu implementación.

## Flujo del código

1. **Inicialización**
	- Definición de parámetros del robot.
	- Condiciones iniciales.
	- Configuración de referencia (trayectoria o setpoint).

2. **Lazo de control**
	- Cálculo del error entre referencia y estado actual.
	- Cómputo de la acción de control.
	- Actualización del estado del robot con el modelo.

3. **Resultados**
	- Guardado de variables de interés.
	- Gráficas de posición, error y señales de control.

## Cómo ejecutar

1. Entra a la carpeta del proyecto.
2. Abre la carpeta `tareas9B`.
3. Ejecuta el script principal (`main.*`) con el intérprete correspondiente.

Ejemplo genérico:

```bash
# Python
python main.py

# MATLAB/Octave
main
```

## Qué revisar en los resultados

- Estabilidad del sistema.
- Error en estado estacionario.
- Tiempo de establecimiento.
- Esfuerzo de control (saturaciones/oscilaciones).

## Recomendaciones

- Mantener parámetros y constantes en una sección clara.
- Documentar unidades (m, rad, s, N, etc.).
- Comentar decisiones de diseño del controlador.
- Versionar pruebas y cambios relevantes.

## Autor

Práctica de la materia **Control de Robots**.