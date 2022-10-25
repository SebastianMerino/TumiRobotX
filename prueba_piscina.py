"""
Script de ejemplo de uso del módulo pixhawk.py para manejar la
comunicación entre Pixhawk y Jetson para el proyecto WAM-V

Prueba de movimiento en piscina
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

try:
	while not vehicle.get_mode() == 2:
		pass
	vehicle.groundspeed = 0.15     # Velocidad de movimiento 1m/s
	"""
	#----------------------------------------------------------------------------
	# PRUEBA IDA Y VUELTA
	#----------------------------------------------------------------------------
	# 5m hacia adelante
	vehicle.go_to(x=5,y=0,relative=True,blocking=True,tolerance=1)
	# Giro de 180
	vehicle.set_heading(180,relative=True,blocking=True,tolerance=10)
	# 5m de regreso
	vehicle.go_to(x=5,y=0,relative=True,blocking=True,tolerance=1)
	"""

	#----------------------------------------------------------------------------
	# PRUEBA GIROS
	#----------------------------------------------------------------------------
	vehicle.set_heading(180,relative=True,blocking=True,tolerance=10)
	time.sleep(1)
	vehicle.set_heading(-90,relative=True,blocking=True,tolerance=10)
	time.sleep(1)
	vehicle.set_heading(90,relative=True,blocking=True,tolerance=10)
	
finally:
	vehicle.disarm()
	vehicle.close()