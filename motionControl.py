"""
Módulo para manejar la comunicación entre la pixhawk y la Jetson para el proyecto WAM-V 
Referencias:
	https://github.com/ArduPilot/ardupilot_wiki/blob/master/dev/source/docs/mavlink-rover-commands.rst
	https://ardupilot.org/rover/docs/guided-mode.html
	https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
	https://dronekit-python.readthedocs.io/en/latest/examples/create_attribute.html
	https://dronekit-python.readthedocs.io/en/latest/automodule.html
	https://mavlink.io/en/messages/common.html
"""

from dronekit import Vehicle, mavutil, LocationLocal, VehicleMode
import time
import math

#-------------------------------------------------------------------
#	Clase para manejar información de las PWM
#-------------------------------------------------------------------
class RawPWM(object):
	""" Clase para manejar la info de las PWM de los motores.
	Contiene tiempo en alta en us """
	def __init__(self, PWMDerecha=None, PWMIzquierda=None):
		"""RawPWM object constructor """
		self.PWMDerecha = PWMDerecha
		self.PWMIzquierda = PWMIzquierda 
		
	def __str__(self):
		""" String representation of the RawPWM object """
		return "RawPWM: PWMDerecha={},PWMIzquierda={}".format(self.PWMDerecha, self.PWMIzquierda)

#-------------------------------------------------------------------
#	Clase para manejar comunicación con Pixhawk
#-------------------------------------------------------------------
class MyVehicle(Vehicle):
	""" Clase para manejar la comunicación entre la Pixhawk y una computadora """
	def __init__(self, *args):
		super(MyVehicle, self).__init__(*args)

		self._raw_pwm = RawPWM()
		self._target = LocationLocal(None,None,None)

		@self.on_message('SERVO_OUTPUT_RAW')
		def listener(self, name, message):
			"""	Actualiza la propiedad raw_pwm cuando se recibe el mensaje """
			self._raw_pwm.PWMDerecha = message.servo1_raw
			self._raw_pwm.PWMIzquierda = message.servo3_raw

			# Notify all observers of new message (with new value)
			self.notify_attribute_listeners('raw_pwm', self._raw_pwm)

	@property
	def raw_pwm(self):
		""" Propiedad que contiene el tiempo en alta de cada PWM en us """
		return self._raw_pwm

	@property
	def target(self):
		""" Propiedad que contiene el tiempo en alta de cada PWM en us """
		return self._target

	def request_pwm(self, freq):
		""" Envía un mensaje al vehículo para que actualice la info de las
		PWM de los motores con la frecuencia especificada """
		msg = self.message_factory.command_long_encode(
			0, 0,    # target_system, target_component
			mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, #command
			0,					#confirmation
			mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
			1/freq * 1000000,	# Time between messages in us
			0,0,0,0,			# Ignored
			0)      			# Response target
		self.send_mavlink(msg)	# send command to vehicle

	def get_location(self):
		""" Obtiene la ubicación global (latitud y longitud) en forma de string """
		global_cords = self.location.global_frame
		if global_cords.lat >= 0:
			latitude = f'{global_cords.lat:.5f}'
			NS_indicator = 'N'
		else:
			latitude = f'{-global_cords.lat:.5f}'
			NS_indicator = 'S'
		if global_cords.lon >= 0:
			longitude = f'{global_cords.lon:.5f}'
			EW_indicator = 'E'
		else:
			longitude = f'{-global_cords.lon:.5f}'
			EW_indicator = 'W'
		return (latitude,NS_indicator,longitude,EW_indicator)

	def arm(self):
		""" Habilita motores y cambia a modo guiado """
		while not self.is_armable:
			print("Vehicle is not armable yet...")
			time.sleep(1)
		print("Arming motors")
		self.mode    = VehicleMode("GUIDED")
		self.armed   = True
		while not self.armed:
			print("Waiting for arming...")
			time.sleep(1)
		print("Armed!")

	def disarm(self):
		""" Deshabilita motores """
		print("Disarming motors")
		self.armed   = False
		while self.armed:
			print("Waiting for disarming...")
			time.sleep(1)
		print("Disarmed")

	def set_heading(self, heading):
		"""
		Envía un comando para que le vehículo mire a cierta orientación relativa
		a la orientación actual, en grados sexagesimales y sentido horario.
		A los 3 segundos el vehículo para automáticamente, así que debe enviarse
		cada 3 segundos con un nuevo objetivo.
		"""
		msg = self.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,	# Relativo a la orientacion actual
			0b100111111111, 	# type_mask (only yaw)
			0, 0, 0,			# x, y, z position (not used) 
			0, 0, 0, 			# x, y, z velocity in m/s  (not used)
			0, 0, 0, 			# x, y, z acceleration (not supported)
			heading*math.pi/180, 0)    		# yaw, yaw_rate
		self.send_mavlink(msg)

	def go_to(self, forward, right):
		"""	
		Envía un comando para que el vehículo vaya a una posición relativa,
		x: meters forward, y: meters right		
		Una vez en el que llega, merodeará/circulará alrededor del destino.
		"""
		dNorth,dEast = self.convert_local_to_NED(forward,right)
		current = self.location.local_frame
		newNorth = current.north + dNorth
		newEast = current.east + dEast

		# Set target atribute
		self._target = LocationLocal(newNorth, newEast,current.down)
		
		# Set target relative to the vehicle's EKF Origin in NED frame 
		msg = self.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
			0b110111111100, 		# type_mask (only x,y positions enabled)
			newNorth, newEast, 0,	# x, y, z position
			0, 0, 0, 				# x, y, z velocity in m/s  (not used)
			0, 0, 0, 				# x, y, z acceleration (not supported)
			0, 0)    				# yaw, yaw_rate (not used) 
		self.send_mavlink(msg)

	def reached_target(self,tolerance):
		""" Función que te dice si has llegado al objetivo o no.
		Se debe ingresar la tolerancia en metros.
		Si la distancia al objetivo es menor que la tolerancia,
		devuelve Verdadero. Si no, devuelve Falso """
		distance = get_distance_metres(self.location.local_frame, self.target)
		if distance < tolerance:
			return True
		else:
			return False

	def convert_local_to_NED(self,dx,dy):
		"""
		Convierte coordenadas del eje local a NED
		dx: Metros al frente
		dy: Metros hacia la derecha
		dN: Metros al Norte
		dE: Metros al este
		"""
		yaw = self.attitude.yaw
		ct = math.cos(yaw)
		st = math.sin(yaw)
		dE = ct*dy + st*dx
		dN = -st*dy + ct*dx
		return dN,dE

#-------------------------------------------------------------------
#	FUNCIONES DE UTILIDAD
#-------------------------------------------------------------------
def get_distance_metres(current,target):
	""" Halla la distancia en metros entre dos ubicaciones locales """
	dNorth = current.north - target.north
	dEast = current.east - target.east
	return math.sqrt(dNorth**2 + dEast**2) 