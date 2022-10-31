import time
import logging
import threading
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


LINE_UP = '\033[1A'
LINE_CLEAR = '\x1b[2K'

# Pedir informacion de la PWM 10 veces por segundo 
vehicle.request_pwm(10)

stop_print = False
def print_data():
	while not stop_print:
		print(vehicle.raw_pwm)
		time.sleep(5)	# CAMBIAR ESTO
		print(LINE_UP, end=LINE_CLEAR)
print_thread = threading.Thread(target=print_data, daemon=True)


# Simulacion del mando
vehicle.arm()
vehicle.mode = VehicleMode('GUIDED')

print_thread.start()
vehicle.set_heading(-90,relative=True,blocking=True,tolerance=10)