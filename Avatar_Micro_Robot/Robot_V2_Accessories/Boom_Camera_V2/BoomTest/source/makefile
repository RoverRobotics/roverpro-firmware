#
# Makefile for BoomTest usbinterface
#

COMPILE := 1
LIBS += -lusb-1.0 -L../../libusb-1.0.8/libusb/.libs
DEPS += common/assert

default:
	g++ -c BoomTest.cpp -o BoomTest.o
	g++ -c XboxController.cpp -o XboxController.o
	g++ -c firmware/usb_config.c -o firmware/usb_config.o
	g++ -o BoomTest BoomTest.o XboxController.o firmware/usb_config.o `sdl-config --libs` -lusb-1.0

