"""
Script de ejemplo de uso del módulo pixhawk.py para manejar la
comunicación entre Pixhawk y Jetson para el proyecto WAM-V

Prueba de solicitud de parámetros
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

# Parámetros de prueba
for i in range(3):
    print(vehicle.mode)                     # Modos: LOITER (hold para botes), MANUAL y GUIDED (autonomo)
    print(vehicle.gps_0)                    # fix type (2D, 3D) y número de satelites visibles
    print(vehicle.location.local_frame)     # Ubicación en frame NED local
    print('Velocity: ', vehicle.velocity)   # Velocidad en frame NED
    print(vehicle.attitude)                 # Orientación
    print(vehicle.raw_pwm)                  # Tiempo en alta en us

    (lat,NS,lon,EW) = vehicle.get_location()    # Ubicación global (string tuple)
    print(lat + ',' + NS + ',' + lon + ',' + EW + '\n')
    time.sleep(1)
