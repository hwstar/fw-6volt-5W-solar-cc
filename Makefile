RDUINO_DIR = /usr/share/arduino
#BOARD_TAG    = nano328
BOARD_TAG = pro5v328
ARDUINO_PORT = /dev/ftdicable
ARDUINO_LIBS =

include /usr/share/arduino/Arduino.mk


burn:	
	-killall gtkterm
	sleep 1
	make upload
	gtkterm -c arduino -p $(ARDUINO_PORT) -s 9600 &


