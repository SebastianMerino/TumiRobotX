"""
Script de ejemplo de uso del módulo pixhawk.py para manejar la
comunicación entre Pixhawk y Jetson para el proyecto WAM-V

Prueba de movimiento
"""
import logging
from motionControl import *
from dronekit import connect
from serial.tools import list_ports

# Establecer conexion con Pixhawk
Pix_VID = 11694
Pix_PID = 4118
Pix_EST = 0
device_list = list_ports.comports()
for device in device_list:
	if (device.vid == Pix_VID and device.pid == Pix_PID):
			Pix = device.device
			Pix_EST = 1
if Pix_EST == 1:
	print("Pixhawk Detectada")
	vehicle = connect(Pix, wait_ready=True, vehicle_class=MyVehicle)
	print("Pixhawk: Conexion Establecida")
	Pix_EST = 2
else:
	raise Exception("no esta la pixhawk pipipipipi")

# Solo mostrar mensajes de error críticos
logger = logging.getLogger('autopilot')
logger.setLevel(logging.CRITICAL)

# Pedir informacion de la PWM 10 veces por segundo 
vehicle.request_pwm(10)

LINE_UP = '\033[1A'
LINE_CLEAR = '\x1b[2K'


#----------------------------------------------------------------------------
# COMANDO PARA MOVER AL VEHICULO
#----------------------------------------------------------------------------
vehicle.groundspeed = 0.5     # Velocidad de movimiento 1m/s
forward = 0
right = -3

try:
    while vehicle.get_mode()==1:
        print(vehicle.mode)
        time.sleep(0.5)
        print(LINE_UP, end=LINE_CLEAR)
        pass
    print('Enviando al vehículo ',forward,'m al frente y ',right,'m a la derecha')
    vehicle.go_to(forward,right)
    while True:
        current = vehicle.location.local_frame
        print("\nCurrent position:\t", current)
        target = vehicle.target
        print("Target position:\t", target)
        remainingDistance = get_distance_metres(current, target)
        print("Distance to target:\t", remainingDistance)
        print(vehicle.raw_pwm)
        print(vehicle.mode)
        print('Speed:', vehicle.velocity)

        time.sleep(0.5)

        for _ in range(7):
            print(LINE_UP, end=LINE_CLEAR)

finally:
    vehicle.close()


"""
#----------------------------------------------------------------------------
# COMANDO PARA CAMBIAR ORIENTACIÓN
#----------------------------------------------------------------------------
try:
    heading = -90
    print("Pidiendo al vehículo que mire hacia: ", heading)
    theta = reduce_angle(vehicle.attitude.yaw*180/math.pi + heading)

    while True:
        vehicle.set_heading(theta, relative=False) 
        print('Yaw angle:\t', vehicle.attitude.yaw*180/math.pi)
        print('Target angle:\t', theta) 
        difference = reduce_angle(vehicle.attitude.yaw - theta*math.pi/180)*180/math.pi
        print('Difference: ', difference)
        print(vehicle.raw_pwm)
        print(vehicle.mode)
        print('Armed:', vehicle.armed)
        
        time.sleep(1)
        for _ in range(6):
            print(LINE_UP, end=LINE_CLEAR)

finally:
   
    vehicle.close()
"""