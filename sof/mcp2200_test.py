#!/usr/bin/python3
# -*- coding: utf-8 -*-
'''
##
## Test IC MCP2200 USB-HID para PCB 4 reles SSR
##

http://ww1.microchip.com/downloads/en/DeviceDoc/93066A.pdf
https://gist.github.com/toxicantidote/f23b093fb05babea47ffe6c2de52d6ef
https://gist.github.com/toxicantidote/a2feb04fd9fe75cec24da8e37518c6c2
https://github.com/cdtx/mcp2200/blob/master/cdtx/mcp2200/device.py

ver info dispositivos:
udevadm info --name=/dev/ttyACM0 --attribute-walk
Permitir ejecutar sin ROOT:
Agregue la siguiente regla al archivo 
nano /etc/udev/rules.d/51-mcp2200.rules (créarlo si es necesario)
ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="00df", GROUP:="plugdev", MODE="0660"

'''
import usb.core
import usb.util
import time
import sys

MCP2200_VID = 0x04d8
MCP2200_PID = 0x00df
MCP2200_HID_INTERFACE = 2
serial_num = 0
Set_bmap = 11
Clear_bmap = 12

''' Comandos
- SET_CLEAR_OUTPUTS command 0x08
- CONFIGURE         command 0x10
- READ_EE           command 0x20
- WRITE_EE          command 0x40
- READ_ALL          command 0x80
===================================================================
Comando CONFIGURE este comando se utiliza para establecer la configuración
    parámetros que se almacenan en NVRAM, utilizados por el
    MCP2200 después de salir del modo de reinicio
===================================================================
byte 0 command            ->0x10 comando configuración
byte 1-3 sin uso
byte 4 IO_bmap            ->Mapa de bits GPIO para la asignación de pines (1=entrada/0=salida)
byte 5 Config_Alt_Pins    ->Ajustes de pin de configuración alternativos (bit2 TXled, bit3 RXled, bit6 USBCFG y bit7 SSPND)
byte 6 IO_Default_Val_    ->bmap Mapa de bits de valor GPIO de salidas predeterminado
byte 7 Config_Alt_Options ->Opciones de funciones expeciales (bit0 HW_FLOW, bit1 INVERT, bit5 LEDX, bit6 TxTGL y bit7 RxTGL)
byte 8 Baud_H Byte        ->alto de la configuración de velocidad en baudios predeterminada
byte 9 Baud_L Byte        ->bajo de la configuración de velocidad en baudios predeterminada
byte 10-15 sin uso
===================================================================
Comando SET_CLEAR_OUTPUTS (solo son afectados los GPIOs que no tienen asignadas funciones expeciales)
===================================================================
byte 0 command     ->0x08comando salidas GOIOs
byte 1 al 10       ->sin uso
byte 11 Set_bmap   ->los bits corresponden a GPIOs, 1 correspondea GPIO a 1 (Vcc)
byte 12 clear_bmap ->los bits corresponden a GPIOs, 1 correcponde a 0 (GND)
byte 13 al 15      ->sin uso
===================================================================
Comando READ_ALL leer configuracion  NVRAM
===================================================================
 '''
# Definir las estructuras de comando que se están utilizando
cfg_cmd = [0x10] + [0]*15
cfg_cmd[6] = 0xc3 #defecto GPIOS bitmap->11000011
cfg_cmd[9] =0x67 # 115200bps ->103
get_status_cmd = [0x80] + [0]*15
gpio_cmd = [0x08] + [0]*15

## Buscar dispositivo
usbdevs = usb.core.find(find_all=True, idVendor=MCP2200_VID, idProduct=MCP2200_PID)
if usbdevs is None:
        print('Placa NO encontrada')
        exit(1)
else:
        print('Placa encontrada: \n' + str (usbdevs))
print('******************************************************************')
# Recorra e imprima los dispositivos encontrados o seleccione el dispositivo solicitado
for dev in usbdevs:
    print ("dev.iSerialNumber: " + str (dev.iSerialNumber))
    serial_num = usb.util.get_string(dev,dev.iSerialNumber)
    print ("Nº Serie: " + str (serial_num))    
# Comprobamos que encontramos el dispositivo especificado, salir si solo estábamos enumerando
if serial_num == 0:
    print ("No se encotraron dispositivos validos")
    exit(1)
''' 
Reclamar el dispositivo desde el kernel
Obtenga la configuración y separe cada interfaz del kernel
Obtenemos un error 'usb.core.USBError: [Errno 16] Recurso ocupado' de lo contrario
Guarde las interfaces que desconectamos para volver a adjuntarlas más tarde
'''
try:
	if dev.is_kernel_driver_active(2):
		dev.detach_kernel_driver(2)
		usb.util.claim_interface(dev, 2)
		print('Dispositivo solicitado al Kernel --OK--')
## si retorna fallo...
except:
	print('ERROR en solicitud del Kernel')
	usb.util.release_interface(dev, 2)
	if not dev.is_kernel_driver_active(2):
		dev.attach_kernel_driver(2)
	exit(1)
'''
El primer subíndice [0] accede a la primera configuración,
el segundo subíndice [0,0] selecciona la primera interfaz (con la primera configuración alternativa)
y el tercer subíndice [0] selecciona el primer punto final (endpoint).
EndPoint Configuration Table: Describe la forma de comunicar funciones 
del USB con las interfaces. Puede haber más de un endpoint con su 
configuración (como mínimo debe existir el 0).        
'''
## definir el endpoint para enviar(output)
endpoint = dev[0][(MCP2200_HID_INTERFACE,0)][1]
print('Creado endpoint: ' + str (endpoint))
## definir el endpoint para recibir(input)
inipoint = dev[0][(MCP2200_HID_INTERFACE,0)][0]
print('Creado inipoint: ' + str (inipoint))

## Leemos configuración NVRAM
dev.write(endpoint.bEndpointAddress, get_status_cmd)
resp = dev.read(inipoint.bEndpointAddress, 16,1000)

if resp[6] == cfg_cmd[6]:
	print('Configuración NVRAM OK')
else:
	dev.write(endpoint.bEndpointAddress, cfg_cmd)
	dev.write(endpoint.bEndpointAddress, get_status_cmd)
	resp = dev.read(inipoint.bEndpointAddress, 16,1000)
	print('Re-configurando NVRAM...')
	
print('Configuracion  NVRAM: ')
print('[ 0] Comando lectura->' + hex (resp[0]))
print('[ 1] EEP_Addr       ->' + hex (resp[1]))
print('[ 3] EEP_Val        ->' + hex (resp[3]))
print('[ 4] IO_bmap        ->' + hex (resp[4]))
print('[ 5] Opciones GPIOs ->' + hex (resp[5]))
print('[ 6] Defecto GPIOs  ->' + hex (resp[6]))
print('[ 7] Funciones      ->' + hex (resp[7]))
print('[ 8] Baud_H         ->' + hex (resp[8]))
print('[ 9] Baud_L         ->' + hex (resp[9]))
print('[10] BitMaps GPIOs  ->' + hex (resp[10]))
## Ajuste de los parametros (GPIOs como outputs, por defecto off, etc)
print('===TEST salidas GPIOs===')
while True:
	## Reles a y leds off, bits 2, 3, 4 y 5 a 0->reles->GND->OFF, bits 0, 1, 6 y 7 a 1 ->leds->VCC->OFF 
	gpio_cmd[Clear_bmap] = 0x3c # 00111100 
	gpio_cmd[Set_bmap] = 0xc3   # 11000011->GPIOs
	dev.write(endpoint.bEndpointAddress, gpio_cmd)
	print('Todo a OFF  ' + str (gpio_cmd))
	time.sleep(10)

	## recorrer las 4 salidas, encendiéndolas una a una
	# ON relé 1, bit2 a 1->GPIO2->relé 1 ON, bit7 a 0->GPIO7->led1 ON
	gpio_cmd[Clear_bmap] = 0xb8 # led 1-> -1-0111*0*00
	gpio_cmd[Set_bmap] = 0x84# salida 1-> -0-1000*1*11
	dev.write(endpoint.bEndpointAddress, gpio_cmd)
	print('salida 1 ON ' + str (gpio_cmd))
	time.sleep(5)
	
	# ON relé 2, bit5 a 1 ->GPIO5->relé 2 ON, bit6 a 0->GPIO6->led2 ON
	gpio_cmd[Clear_bmap] = 0x5c # led 2->0-1-*0*11100
	gpio_cmd[Set_bmap] = 0xa3# salida 2->1-0-*1*00011
	dev.write(endpoint.bEndpointAddress, gpio_cmd)
	print('salida 2 ON ' + str (gpio_cmd))
	time.sleep(5)
	
	# ON relé 3, bit4 a 1 ->GPIO4->relé 3 ON, bit1 a 0->GPIO1->led3 ON
	gpio_cmd[Clear_bmap] = 0x2e # led 3->001*0*11-1-0
	gpio_cmd[Set_bmap] = 0xd1# salida 3->110*1*00-0-1
	dev.write(endpoint.bEndpointAddress, gpio_cmd)
	print('salida 3 ON ' + str (gpio_cmd))
	time.sleep(5)
	
	# ON relé 4, bit3 a 1 ->GPIO3->relé 4 ON, bit0 a 0->GPIO0->led4 ON
	gpio_cmd[Clear_bmap] = 0x35 # led 4->0011*0*10-1- 
	gpio_cmd[Set_bmap] = 0xca# salida 4->1100*1*01-0-
	dev.write(endpoint.bEndpointAddress, gpio_cmd)
	print('salida 4 ON ' + str (gpio_cmd))
	time.sleep(5)
	
	print('**********************Iniciamos test***********************')

