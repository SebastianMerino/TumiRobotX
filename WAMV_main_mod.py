import sys
sys.path.insert(0, '/home/robotx/Desktop/Python_Scripts/Sebastian')

###########################   Librerias Generales  ###############################
import os
import cv2
import json
import time
import serial
import signal
import requests
import threading
import motionControl
from ouster import client
from dronekit import connect
from datetime import datetime
from contextlib import closing
from serial.tools import list_ports



###########################  Librerias Especificas  ###############################
import Lidar_range_funcion1 as Lidar_range_image  


###########################   Definicion de Hilo    ###############################
def taskMsg():
	threading.Timer(1.0, taskMsg).start()
	AEDTDate = datetime.today().strftime('%d%m%y')
	AEDTTime = datetime.today().strftime('%H%M%S')
	if task == "1":
		MessageID = "$RXHRB"
		(Latitude, NSIndicator, Longitud, EWIndicator) = pixhawk.get_location()
		SystemMode = pixhawk.get_mode()
		msg = MessageID + "," + AEDTDate +  "," + AEDTTime +  "," + TeamID +  "," + Latitude +  "," + NSIndicator +  "," + Longitud +  "," + EWIndicator +  "," + SystemMode +  "," + UAVStatus +  "*"
	elif task == "2":
		MessageID = "$RXGAT"
		msg = MessageID +  "," + AEDTDate +  "," + AEDTTime +  "," + TeamID + "," + ActEntGate +  "," + ActExtGate +  "*"
	elif task == "3":
		MessageID = "$RXPTH"
		msg = MessageID +  "," + AEDTDate +  "," + AEDTTime +  "," + TeamID + "," + Finished +  "*"
	elif task == "4":
		MessageID = "$RXENC"
		msg = MessageID +  "," + AEDTDate +  "," + AEDTTime +  "," + TeamID + "," + Wildlife1 +  "," + Wildlife2 + "," + Wildlife3 + "*"
	elif task == "5":
		MessageID = "$RXCOD"
		msg = MessageID +  "," + AEDTDate +  "," + AEDTTime +  "," + TeamID + "," + LightPat + "*"
	elif task == "6":
		MessageID = "$RXDOK"
		msg = MessageID +  "," + AEDTDate +  "," + AEDTTime +  "," + TeamID + "," + Color + "," + AMSStatus + "*"
	elif task == "7":
		MessageID = "$RXFLG"
		msg = MessageID +  "," + AEDTDate +  "," + AEDTTime +  "," + TeamID + "," + Color + "," + AMSStatus + "*"
	elif task == "8":
		MessageID = "$RXUAV"
		msg = "Realizando Tarea 10 *"
	elif task == "9":
		MessageID = "$RXUAV"
		msg = "Realizando Tarea 11 *"

	msg_final = msg + str(checksum(msg)).upper()
	print(msg_final)


###########################        Funciones         ###############################
def checksum(string):
    checks=0
    analyze = string[1:(len(string)-1)]
    for letter in analyze:
        checks ^= ord(letter)
    checks = hex(checks)
    final_checks=checks[2:].upper()
    return final_checks


###########################   Definicion Variables   ###############################
# Comunicacion HTTP
host = '192.168.2.102'
port = 5100
url_usv = "http://{}:{}/robots/usvdata".format(host, port)
url_sonda = "http://{}:{}/robots/datasensor".format(host, port)

# Formato JSON
variable_json = {}


# Comunicacion Serial: Product y Vendor ID
# NUcleo H7
Nuc_VID = 1155
Nuc_PID = 14158

# Pixhawk
Pix_VID = 11694
Pix_PID = 4118

# Comunicacion Ethernet: IP
# Ouster Lidar
ous_IP = "10.5.5.55"

# Hyperspectral Camera
hyp_IP = "10.5.5.60"


# Estados de los dispositivos seriales y ethernet
    # 0: Sin conexion
    # 1: Conexion fisica
    # 2: Conexion serial o ethernet
Nuc_EST = 0
Pix_EST = 0
Ous_EST = 0
Hyp_EST = 0


###########################   Messages Format    ###############################
# Common 
MessageID = ""
AEDTDate = ""
AEDTTime =""
TeamID = ""
Checksum = ""

# C3, Task 1
Latitude = ""
NSIndicator = ""
Longitud = ""
EWIndicator = ""
SystemMode = ""
UAVStatus = ""

# C4, Task 2
ActEntGate = ""
ActExtGate = ""

# C5, Task 3
Finished = ""

# C6, Task 4
Wildlife1 = ""
Wildlife2 = ""
Wildlife3 = ""

# C7, Task 5
LightPat = ""

# C8, Task 6
Color = ""
AMSStatus = ""

# C9, Task 7
Color = ""
AMSStatus = ""

###########################      Datos Lidar     ###############################

distancia_max = 12000				#distancia maxima de alcance del lidar
distancia_min = 1000				#distancia minima de alcance del lidar
factor_resize_x_lidar = 70			#factor para agrandar 16
factor_resize_y_lidar = 5			#Factor para agrandar 1024
pixel_x_lidar_recorte_1_camara1 = 4		#pixel a leer en el eje x --> vertical (16 canales)
pixel_y_lidar_recorte_1_camara1 = 338		#pixel a leer en el eje y --> horizontal (1024 canales)
pixel_x_lidar_recorte_2_camara1 = 16		#pixel a leer en el eje x --> vertical (16 canales)
pixel_y_lidar_recorte_2_camara1 = 502		#pixel a leer en el eje y --> horizontal (1024 canales)
pixel_x_lidar_recorte_1_camara2 = 4		#pixel a leer en el eje x --> vertical (16 canales)
pixel_y_lidar_recorte_1_camara2 = 512		#pixel a leer en el eje y --> horizontal (1024 canales)
pixel_x_lidar_recorte_2_camara2 = 16		#pixel a leer en el eje x --> vertical (16 canales)
pixel_y_lidar_recorte_2_camara2 = 676		#pixel a leer en el eje y --> horizontal (1024 canales)

fontColor = (0, 255, 0)
pixel_x_lidar = 4				#pixel a leer en el eje x --> vertical (16 canales)
pixel_y_lidar = 338				#pixel a leer en el eje y --> horizontal (1024 canales)
y_camara1_img = 100				#int(pixel_y_lidar*factor_resize_y_lidar - factor_resize_y_lidar*0.5) #Coordenada de recorte "y1"
x_camara1_img = 100				#int((pixel_x_lidar*factor_resize_x_lidar - factor_resize_x_lidar*0.5)*0.5) #Coordenada de recorte "x1"
y_camara2_img = 100				#int(pixel_y_lidar*factor_resize_y_lidar - factor_resize_y_lidar*0.5) #Coordenada de recorte "y1"
x_camara2_img = 100				#int((pixel_x_lidar*factor_resize_x_lidar - factor_resize_x_lidar*0.5)*0.5) #Coordenada de recorte "x1"
y_lidar1_img = y_camara1_img			#int(pixel_y_lidar*factor_resize_y_lidar - factor_resize_y_lidar*0.5) #Coordenada de recorte "y1"
x_lidar1_img = x_camara1_img			#int((pixel_x_lidar*factor_resize_x_lidar - factor_resize_x_lidar*0.5)*0.5) #Coordenada de recorte "x1"
x_lidar2_img = x_camara2_img - 15		#int(pixel_y_lidar*factor_resize_y_lidar - factor_resize_y_lidar*0.5) #Coordenada de recorte "y1"
y_lidar2_img = y_camara2_img + 50		#int((pixel_x_lidar*factor_resize_x_lidar - factor_resize_x_lidar*0.5)*0.5) #Coordenada de recorte "x1"

factor_resize_x_camara = 0.65			#factor para agrandar 800
factor_resize_y_camara = 0.65			#Factor para agrandar 1280
ancho_camara = 1280				#tamaño por default
altura_camara = 800				#tamaño por default



###########################   Codigo Principal   ###############################

# Busqueda de puertos USB conectados
device_list = list_ports.comports()

for device in device_list:
	print(device.vid)
	print(device.pid)
	if (device.vid != None or device.pid != None):
		if (device.vid == Nuc_VID and device.pid == Nuc_PID):
			Nuc = device.device
			Nuc_EST = 1
		elif (device.vid == Pix_VID and device.pid == Pix_PID):
			Pix = device.device
			Pix_EST = 1

# Establece conexion con dispositivos USB conectados
if Nuc_EST == 1:
	print("Nucleo Detectada")
	nucleo = serial.Serial(Nuc, 115200, 8, 'N', 1)
	if nucleo.is_open:
		print("Nucleo: Conexion Establecida")
		Nuc_EST = 2

if Pix_EST == 1:
	print("Pixhawk Detectada")
	pixhawk = connect(Pix, wait_ready=True, vehicle_class=motionControl.MyVehicle)
	#if pixhawk.is_open:
	print("Pixhawk: Conexion Establecida")
	Pix_EST = 2

# Establece conexion con dispositivos ethernet conectados
res_Ous = os.system("ping -c 1 " + ous_IP)
if res_Ous == 0:
	print("Lidar Ouster Detectado")
	Ous_EST = 2

res_Hyp = os.system("ping -c 1 " + hyp_IP)
if res_Hyp == 0:
	print("Camara Hyperespectral Detectada")
	Hyp_EST = 2


# Ingresar TeamID
print("Ingresar TeamID: ")
TeamID = input()

# Ingresar el task a realizar
print("Ingresar tarea a realizar: ")
task = input()

# Envia mensaje cada 1 segundo dependiendo de la tarea elegida
taskMsg()

# Deteccion de camara
captura_1 = cv2.VideoCapture(0) #a veces el puerto puede ser 0, -1, 1
captura_2 = cv2.VideoCapture(1) #a veces el puerto puede ser 0, -1, 1
lidar_port = 7502
hostname = 'os1-991918000500.local'

###########################    Loop Principal    ###############################

if task == "1":
	print("Realizando tarea 1: Heartbeat Message")
elif task == "2":
	print("Realizando tarea 2: Entrance and Exit Gates")
elif task == "3":
	print("Realizando tarea 3: Follow the Path")
	with closing(client.Scans.stream(hostname, lidar_port, complete = False)) as stream:
		show = True     
		ranges_aol_buffer = []
		while (show and captura_1.isOpened() and captura_1.isOpened()) : #Habilitar imagen
			for scan in stream:

				"""# Lectura

				#Verificar que Nucleo envia data correctamente
				try:
					if(Nuc_EST == 2):
						Nuc_msg = (nucleo.readline()).decode()
						Nuc_split = Nuc_msg.split(",")
						ActEntGate = Nuc_split[5]
						ActEndGate = Nuc_split[6]
						SystemMode = Nuc_split[7]
					
				except:
					print("Error con Nucleo\r\n")


				#Verificar que Pixhawk envia data correctamente
				try:
					if(Pix_EST == 2):
						print("a")		
				
				except:
					print("Error en Pixhawk\r\n")"""


				ret1, frame1 = captura_1.read()
				frame_copia1 = frame1
				ret2, frame2 = captura_2.read()
				frame_copia2 = frame2

				width = int(ancho_camara*factor_resize_y_camara) #se alarga en el eje y 
				height = int(altura_camara*factor_resize_x_camara) #se alarga en el eje x
				dsize = (width, height)
				frame1_reescalado = cv2.resize(frame_copia1, dsize) #Reescalamiento de la imagen
				frame2_reescalado = cv2.resize(frame_copia2, dsize) #Reescalamiento de la imagen
				if (ret1 == True and ret2 == True): #ret indica si se accedio a la camara sin problemas
					cv2.circle(frame1_reescalado,(x_camara1_img,y_camara1_img),5,(0,255,0),-1) 
					cv2.imshow('webCam1', frame1_reescalado)
					cv2.circle(frame2_reescalado,(x_camara2_img,y_camara2_img),5,(0,255,0),-1) 
					cv2.imshow('webCam2', frame2_reescalado)
					if(cv2.waitKey(1) == ord('s')): #si se presiona "s" sale del bucle
						show = False
						break
				else:
					break

				#--------Encontrar posibles boyas en la imagen de escala de grises del lidar-range----------------------
				range = client.destagger(stream.metadata, scan.field(client.ChanField.RANGE)) #matriz que contine las distancias de cada pixel en milimetros
				image_range = Lidar_range_image.Range_Lidar(range,distancia_max,distancia_min,factor_resize_x_lidar,factor_resize_y_lidar)
				y_1_recorte_camara1 = int(pixel_y_lidar_recorte_1_camara1*factor_resize_y_lidar - factor_resize_y_lidar*0.5) #Coordenada de recorte "y1"
				x_1_recorte_camara1 = int((pixel_x_lidar_recorte_1_camara1*factor_resize_x_lidar - factor_resize_x_lidar*0.5)*0.5) #Coordenada de recorte "x1"
				y_2_recorte_camara1 = int(pixel_y_lidar_recorte_2_camara1*factor_resize_y_lidar - factor_resize_y_lidar*0.5) #Coordenada de recorte "y2"
				x_2_recorte_camara1 = int((pixel_x_lidar_recorte_2_camara1*factor_resize_x_lidar - factor_resize_x_lidar*0.5)*0.5) #Coordenada de recorte "x2"
				image_range_recortada_camara1 = image_range[x_1_recorte_camara1:x_2_recorte_camara1, y_1_recorte_camara1:y_2_recorte_camara1]

				y_1_recorte_camara2 = int(pixel_y_lidar_recorte_1_camara2*factor_resize_y_lidar - factor_resize_y_lidar*0.5) #Coordenada de recorte "y1"
				x_1_recorte_camara2 = int((pixel_x_lidar_recorte_1_camara2*factor_resize_x_lidar - factor_resize_x_lidar*0.5)*0.5) #Coordenada de recorte "x1"			
				y_2_recorte_camara2 = int(pixel_y_lidar_recorte_2_camara2*factor_resize_y_lidar - factor_resize_y_lidar*0.5) #Coordenada de recorte "y2"
				x_2_recorte_camara2 = int((pixel_x_lidar_recorte_2_camara2*factor_resize_x_lidar - factor_resize_x_lidar*0.5)*0.5) #Coordenada de recorte "x2"
				image_range_recortada_camara2 = image_range[x_1_recorte_camara2:x_2_recorte_camara2, y_1_recorte_camara2:y_2_recorte_camara2]

				cv2.circle(image_range_recortada_camara1,(x_lidar1_img,y_lidar1_img),5,(0,255,0),-1) #imagen, coordenadas,radio del circulo,Color del circulo, circulo con relleno 
				cv2.imshow('Lidar_Imagen1', image_range_recortada_camara1)
				cv2.circle(image_range_recortada_camara2,(x_lidar2_img,y_lidar2_img),5,(0,255,0),-1)
				cv2.imshow('Lidar_Imagen2', image_range_recortada_camara2)
				
elif task == "4":
	print("Realizando tarea 4: Wildlife Encounter - React and Report")
elif task == "5":
	print("Realizando tarea 5: Scan the Code")
elif task == "6":
	print("Realizando tarea 6: Detect and Dock")
elif task == "7":
	print("Realizando tarea 7: Find and Fling")
elif task == "8":
	print("Realizando tarea 8: UAV Replenishment")
elif task == "9":
	print("Realizando tarea 9: UAV Search and Report")

captura_1.release()
captura_2.release()
cv2.destroyAllWindows()

"""while True:
	try:
		AEDTDate = datetime.today().strftime('%d%m%y')
		AEDTTime = datetime.today().strftime('%H%M%S')

		#Verificar que Nucleo envia data correctamente
		try:
			if(Nuc_EST == 2):
				Nuc_msg = (nucleo.readline()).decode()
				Nuc_split = Nuc_msg.split(",")
				ActEntGate = Nuc_split[5]
				ActEndGate = Nuc_split[6]
				SystemMode = Nuc_split[7]
					
		except:
			print("Error con Nucleo\r\n")


		#Verificar que Pixhawk envia data correctamente
		try:
			if(Pix_EST == 2):
				print("a")		
				
		except:
			print("Error en Pixhawk\r\n")


	except KeyboardInterrupt:
		print("")
		if Pix_EST == 2:
			pixhawk.close()
			print("Pixhawk Desconectada")
		if Nuc_EST == 2:
			nucleo.close()
			print("Nucleo Desconectada")
		break

print("Programa Finalizado")



#print('GPS: ', pixhawk.gps_0)
#print('Global: ', pixhawk.location.global_frame)
#print('Global relative: ', pixhawk.location.global_relative_frame)
#print('Local: ', pixhawk.location.local_frame)
#print('Attitude: ', pixhawk.attitude)
#print('Velocity: ', pixhawk.velocity,'\n')
#pixhawk.arm()"""



#Capturar puntos con lidar

#Calcular posiciones de elementos cercanos

#Determinar posicionamiento de boyas

#Superponer resultados

