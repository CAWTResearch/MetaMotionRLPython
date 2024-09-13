import signal
import time

# Variable global para almacenar el tiempo antes de la interrupción
start_time = None

# Definir el manejador de la señal (ISR)
def manejador_temporizador(signum, frame):
    global start_time
    if start_time is not None:
        # Calcular el delta_time y resetear start_time
        end_time = time.perf_counter()
        delta_time = end_time - start_time
        print(f"Delta_time de la interrupción: {delta_time:.3f} segundos")
    
    contador = 1
    # print(f"Valor contador: {contador}")
    # Reiniciar start_time para la próxima interrupción
    start_time = time.perf_counter()

# Configurar la señal
signal.signal(signal.SIGALRM, manejador_temporizador)

# Configurar el temporizador de intervalo
signal.setitimer(signal.ITIMER_REAL, 0.01, 0.01)

try:
    while True:
        # Código principal de la aplicación aquí
        contador = 0
        # print(contador)
        time.sleep(1)  # Para mantener viva la ejecución del hilo principal
except KeyboardInterrupt:
    # Detener el temporizador cuando el usuario presione Ctrl+C
    signal.setitimer(signal.ITIMER_REAL, 0)
    print("Temporizador detenido")
