RDUINO_DIR = /usr/share/arduino
#BOARD_TAG    = nano328
BOARD_TAG = pro5v328
ARDUINO_PORT = /dev/ttyUSB*
ARDUINO_LIBS =

burn:	
	-killall gtkterm
	sleep 1
	make upload
	gtkterm -c arduino &

include /usr/share/arduino/Arduino.mk

