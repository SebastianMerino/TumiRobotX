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

import math
import time
from dronekit import Vehicle, mavutil, LocationLocal, VehicleMode

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
		return "RawPWM: PWMIzquierda={}, PWMDerecha={}".format(self.PWMIzquierda, self.PWMDerecha)

#-------------------------------------------------------------------
#	Clase para manejar comunicación con Pixhawk
#-------------------------------------------------------------------
class MyVehicle(Vehicle):
	""" Clase para manejar la comunicación entre la Pixhawk y una computadora """
	def __init__(self, *args):
		super(MyVehicle, self).__init__(*args)

		self._raw_pwm = RawPWM()
		self._target = None

		@self.on_message('SERVO_OUTPUT_RAW')
		def pwm_listener(self, name, message):
			"""	Actualiza la propiedad raw_pwm cuando se recibe el mensaje """
			self._raw_pwm.PWMDerecha = message.servo1_raw
			self._raw_pwm.PWMIzquierda = message.servo3_raw
			self.notify_attribute_listeners('raw_pwm', self._raw_pwm)

	#-------------------------------------------------------------------
	#	Propiedades agregadas
	#-------------------------------------------------------------------
	@property
	def raw_pwm(self):
		""" Propiedad que contiene el tiempo en alta de cada PWM en us """
		return self._raw_pwm

	@property
	def target(self):
		""" Propiedad que contiene el tiempo en alta de cada PWM en us """
		return self._target

	#-------------------------------------------------------------------
	#	Funciones para obtener información de la pixhawk
	#-------------------------------------------------------------------
	def request_pwm(self, freq):
		""" Envía un mensaje al vehículo para que actualice la info de las
		PWM de los motores con la frecuencia especificada """
		msg = self.message_factory.command_long_encode(
			0, 0,    # target_system, target_component
			mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, #command
			0,					#confirmation
			mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,	# message
			1/freq * 1000000,	# Time between messages in us
			0,0,0,0,			# Ignored
			0)      			# Response target
		self.send_mavlink(msg)	# send command to vehicle

	def get_location(self):
		""" Obtiene la ubicación global (latitud y longitud) para el protocolo de comunicacion """
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

	def get_mode(self):
		""" Obtiene el System mode para el protocolo de comunicación """
		mode = self.mode.name
		if mode == "LOITER" or mode == "MANUAL" or mode == "HOLD":
			return 1
		elif mode == "GUIDED":
			return 2
		else:
			return 3

	#-------------------------------------------------------------------
	#	Funciones para controlar el movimiento  y orientación
	#-------------------------------------------------------------------
	#	Coordenadas NED
	# 	Con origen EKF (primera vez que se obtiene la señal de GPS) en 
	# 	un frame NED, orientación con 0 en el norte
	#		x: meters North		y: meters East
	#	Coordenadas relativas
	# 	Con origen en el vehículo, orientación con 0 hacia adelante
	#		x: meters forward	y: meters right
	def arm(self):
		""" Habilita motores y cambia a modo guiado """
		while not self.is_armable:
			print("Vehicle is not armable yet...")
			time.sleep(1)
		print("Arming motors")
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

	def go_to(self, x, y, relative=True, blocking=True, tolerance = 1):
		""" Envía un comando para que el vehículo vaya a una posición.	
		Una vez que llega, merodeará/circulará alrededor del destino.
			relative:	Indica si las coordenadas son relativas al vehículo o NED 
			blocking:	Indica si debe bloquear hasta llegar al objetivo
			tolerance:	Tolerancia en metros para decidir si de ha llegado al objetivo """

		# Guarda objetivo en coordenadas absolutas en variable target
		if relative:
			self._target = self.convert_relative_to_NED(x,y)
		else:
			self._target = LocationLocal(x,y,0)
		
		# Generación de comando y envío
		msg = self.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
			0b110111111100, 		# type_mask (only x,y positions enabled)
			self.target.north, self.target.east, 0,		# x, y, z position
			0, 0, 0, 				# x, y, z velocity in m/s  (not used)
			0, 0, 0, 				# x, y, z acceleration (not supported)
			0, 0)    				# yaw, yaw_rate (not used) 
		
		# Espera hasta llegar al objetivo
		while True:
			self.send_mavlink(msg)
			if not blocking or self.reached_target(tolerance):
				break
			#if not self.get_mode() == 2:
				#raise Exception('Command interrupted. Vehicle is not on GUIDED mode')
			time.sleep(0.1)		# Chequea 10 veces por segundo

	def set_heading(self, heading, relative=True, blocking=True, tolerance = 15):
		"""	Envía un comando para que el vehículo mire a cierta orientación. 
		La función puede bloquear hasta que se llegue al objetivo, caso contrario,
		debe enviarse cada 3 segundos con un nuevo objetivo.
			heading: 	Orientación en grados sexagesimales, sentido horario 
			relative:	Indica si la orientación es relativa al vehículo o absoluta 
			blocking:	Indica si debe bloquear hasta llegar al objetivo
			tolerance:	Tolerancia en grados sexagesimales. Solo se utiliza si
						blocking = True. La funcion termina cuando la orientacion
						es la orientecion deseada +- tolerancia """

		# Hallar orientacion en coordenadas absolutas
		if relative:
			theta = reduce_angle(self.attitude.yaw + heading*math.pi/180)
		else:
			theta = reduce_angle(heading*math.pi/180)
		tolerance_rad = tolerance*math.pi/180

		# Generacion del mensaje
		msg = self.message_factory.set_position_target_local_ned_encode(
			0,       	# time_boot_ms (not used)
			0, 0,   	# target system, target component
			mavutil.mavlink.MAV_FRAME_LOCAL_NED,		# Eje relativo o absoluto
			0b100111111111, 	# type_mask (only yaw)
			0, 0, 0,			# x, y, z position (not used) 
			0, 0, 0, 			# x, y, z velocity in m/s  (not used)
			0, 0, 0, 			# x, y, z acceleration (not supported)
			theta, 0)    		# yaw, yaw_rate in rad
		
		# Envío del mensaje en bucle
		while True:
			self.send_mavlink(msg)
			if not blocking or abs(reduce_angle(theta - self.attitude.yaw))<tolerance_rad:
				break
			#if not self.get_mode() == 2:
				#raise Exception('Command interrupted. Vehicle is not on GUIDED mode')
			time.sleep(0.5)

	def point_to(self, x, y, relative=True, blocking=True, tolerance = 15):
		""" Dada unas coordenadas absolutas o relativas, envía un
		comando al vehículo para que apunte a esa dirección """
		# Convertir el objetivo (region of interest) a coordenadas absolutas
		if relative:
			roi = self.convert_relative_to_NED(x,y)
		else:
			roi = LocationLocal(x,y,0)
		
		# Hallar orientacion deseada
		current = self.location.local_frame
		dN = roi.north - current.north
		dE = roi.east - current.east
		theta = math.atan2(dE,dN)

		# Cambia orientacion
		self.set_heading(theta, relative=False,blocking=blocking,tolerance=tolerance)

	def circle(self,x,y,relative=True,r=10,CW=True,num_angles=8):
		""" Manda comandos para rodear un objetivo en las coordenadas (x,y)
		Bloquea hata llegar al objetivo.
			relative:	indica si el centro está en coordenadas relativas o absolutas
			r: 			radio en metros
			CW: 		sentido de giro horario
			num_angles: número de puntos del circulo """
		# Convertir el centro a coordenadas absolutas
		if relative:
			center = self.convert_relative_to_NED(x,y)
		else:
			center = LocationLocal(x,y,0)
		
		# Obtener puntos alrededor del centro
		puntosN = []
		puntosE = []
		vN = center.north - self.location.local_frame.north
		vE = center.east - self.location.local_frame.east
		mod = math.sqrt(vN**2 + vE**2)
		vN = vN/mod
		vE = vE/mod

		for k in range(num_angles):
			theta_k = k*2*math.pi/num_angles
			if not CW:
				theta_k = -theta_k
			PkN = center.north - r*math.cos(theta_k)*vN - r*math.sin(theta_k)*vE
			puntosN.append(PkN)
			PkE = center.east - r*math.cos(theta_k)*vE + r*math.sin(theta_k)*vN
			puntosE.append(PkE)

		# Rodear objeto. No envía el siguiente comando a menos que se acerce al objetivo
		# a una distancia de 10% del radio
		for i in range(num_angles):
			self.go_to(puntosN[i],puntosE[i],relative=False,blocking=True,tolerance=0.1*r)

		# Volver al primer punto
		self.go_to(puntosN[0],puntosE[0],relative=False,blocking=True,tolerance=0.1*r)

	#-------------------------------------------------------------------
	#	Otras funciones de utilidad
	#-------------------------------------------------------------------
	def reached_target(self,tolerance):
		""" Indica si se ha llegado al último objetivo o no. Se debe ingresar
		la tolerancia en metros. Si la distancia al objetivo es menor que la
		tolerancia, devuelve Verdadero. Si no, devuelve Falso """
		distance = get_distance_metres(self.location.local_frame, self.target)
		return distance < tolerance

	def convert_relative_to_NED(self,dx,dy):
		""" Convierte coordenadas relativas a absolutas. Devuelve un objeto LocationLocal
			dx: Metros al frente
			dy: Metros hacia la derecha """
		yaw = self.attitude.yaw
		ct = math.cos(yaw)
		st = math.sin(yaw)
		dE = ct*dy + st*dx		# Metros al este
		dN = -st*dy + ct*dx		# Metros al norte
		current = self.location.local_frame
		newNorth = current.north + dN
		newEast = current.east + dE
		return LocationLocal(newNorth, newEast,0)

#-------------------------------------------------------------------
#	FUNCIONES DE UTILIDAD
#-------------------------------------------------------------------
def get_distance_metres(current,target):
	""" Halla la distancia en metros entre dos ubicaciones locales """
	dNorth = current.north - target.north
	dEast = current.east - target.east
	return float(math.sqrt(dNorth**2 + dEast**2))

def reduce_angle(angle):
	""" Dado un angulo en radianes, lo reduce al ángulo -PI a PI """
	return ((angle + math.pi) % (2*math.pi)) - math.pi