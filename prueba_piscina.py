"""
Script de ejemplo de uso del módulo pixhawk.py para manejar la
comunicación entre Pixhawk y Jetson para el proyecto WAM-V

Prueba de movimiento en piscina
"""
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

try:
	while not vehicle.get_mode() == 2:
		pass
	#----------------------------------------------------------------------------
	# COMANDO PARA MOVER AL VEHICULO
	#----------------------------------------------------------------------------
	vehicle.groundspeed = 0.15     # Velocidad de movimiento 1m/s
	metros_adelante = 5
	metros_derecha = 0

	# 5 m hacia adelante
	vehicle.go_to(metros_adelante,metros_derecha,relative=True,blocking=True,tolerance=1)

	# Giro de 180
	vehicle.set_heading(180,relative=True,blocking=True,tolerance=15)

	# 5m de regreso
	vehicle.go_to(metros_adelante,metros_derecha,relative=True,blocking=True,tolerance=1)

finally:
	vehicle.disarm()
	vehicle.close()