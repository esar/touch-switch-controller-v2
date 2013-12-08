all:
	~/projects/arduino/arduino-1.0.2/hardware/tools/avr/bin/avr-gcc -mmcu=attiny861 touch_controller.c -o touch_controller
	~/projects/arduino/arduino-1.0.2/hardware/tools/avr/bin/avr-objcopy -O ihex touch_controller touch_controller.hex
	~/projects/arduino/arduino-1.0.2/hardware/tools/avrdude -p t861 -c avrispmkii -C ~/projects/arduino/arduino-1.0.2/hardware/tools/avrdude.conf  -P usb -U flash:w:touch_controller.hex 
