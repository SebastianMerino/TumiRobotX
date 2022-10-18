"""
Script de ejemplo de uso del módulo pixhawk.py para manejar la
comunicación entre Pixhawk y Jetson para el proyecto WAM-V

Prueba de movimiento
"""
from motionControl import *
from dronekit import connect
import time

# Conexión con pixhawk usando clase MyVehicle
# Cambiar puerto COM por usb device path
vehicle = connect('COM9', wait_ready=True, vehicle_class=MyVehicle)
print("Connected!\n")

# Pedir informacion de la PWM 10 veces por segundo 
vehicle.request_pwm(10)

# Habilitar los motores, es necesario tener señal GPS
vehicle.arm()

#----------------------------------------------------------------------------
# COMANDO PARA MOVER AL VEHICULO
#----------------------------------------------------------------------------
vehicle.groundspeed = 1     # Velocidad de movimiento 1m/s
forward = 0
right = -5
vehicle.go_to(forward,right)
print('Enviando al vehículo ',forward,'m al frente y ',right,'m a la derecha')

while True:
    # Calcula distancia al objetivo
    current = vehicle.location.local_frame
    target = vehicle.target
    remainingDistance = get_distance_metres(current, target)
    
    # Muestra info
    print("\nDistance to target: ", remainingDistance)
    print('Current position: ', current)
    print('Target position: ', target)
    print(vehicle.raw_pwm)
    
    if remainingDistance <= 1: # Distancia menor a un metro
        print("Reached target") # La posición debería mantenerse automáticamente
        break
    time.sleep(1)

#----------------------------------------------------------------------------
# COMANDO PARA CAMBIAR ORIENTACIÓN
#----------------------------------------------------------------------------
vehicle.set_heading(-90)
print("Pidiendo al vehículo que mire hacia la izquierda (-90°)")
for _ in range(5):  # Debe parar automaticamente despues de 3 segundos
    time.sleep(1)    
    print(vehicle.attitude.yaw)
    print(vehicle.raw_pwm)

vehicle.close()