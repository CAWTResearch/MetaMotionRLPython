import signal
import time
import csv
import math
import random
import threading

# Variables globales
start_time = None
contador = 0
max_count = 15001
tiempos = []
# Constantes
PERIOD_MS = 0.02  # Ajusta según el periodo deseado en milisegundos
lock = threading.Lock()  # Para manejar la concurrencia con seguridad

# Definir el manejador de la señal (ISR)
def manejador_temporizador(signum, frame):
    global start_time, contador, tiempos

    with lock:  # Asegura que no haya conflictos concurrentes
        if start_time is not None:
            # Calcular el delta_time en milisegundos
            end_time = time.perf_counter()
            delta_time = (end_time - start_time) * 1000  # Convertir a milisegundos
            # Redondear a 3 cifras significativas
            delta_time = round(delta_time, 3)
            tiempos.append(delta_time)

        contador += 1
        # Reiniciar start_time para la próxima interrupción
        start_time = time.perf_counter()

        # Cuando se llegue al contador máximo, detener el temporizador
        if contador >= max_count:
            signal.setitimer(signal.ITIMER_REAL, 0)  # Detener el temporizador

# Función para guardar los tiempos en un archivo CSV
def save_to_csv(tiempos):
    with open('timer_data_isr.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Contador", "Tiempo (ms)"])
        for i, tiempo in enumerate(tiempos):
            writer.writerow([i, tiempo])
    print("Datos guardados en timer_data_isr.csv")

# Configurar la señal para manejar el temporizador
signal.signal(signal.SIGALRM, manejador_temporizador)

# Configurar el temporizador de intervalo: 10 ms (0.01 segundos)
signal.setitimer(signal.ITIMER_REAL, PERIOD_MS, PERIOD_MS)

y = 0.0
x = 0.0

try:
    while contador < max_count:
        # Realiza los cálculos
        y = math.sin(math.pi * x) + math.log(random.random())
    
        # Incrementa x
        x += 0.02
        
        # Define el tiempo de espera en segundos (1 ms = 1e-3 s)
        sleep_time = (PERIOD_MS * 1e-3) / 5  # Equivalente a dividir entre 5
        time.sleep(sleep_time)

except KeyboardInterrupt:
    # Detener el temporizador cuando el usuario presione Ctrl+C
    signal.setitimer(signal.ITIMER_REAL, 0)
    print("Temporizador detenido por el usuario.")

# Guardar los datos en un archivo CSV una vez que el temporizador termine
save_to_csv(tiempos)
