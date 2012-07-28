#
# Makefile for Robotex usbinterface
# J. Brinton
#

COMPILE := 1
LIBS += -lusb-1.0 -L../../libusb-1.0.8/libusb/.libs
DEPS += common/assert

default:
	g++ -c ControllerTest.cpp -o ControllerTest.o
	g++ -c firmware/usb_config.c -o firmware/usb_config.o
	g++ -o ControllerTest ControllerTest.o firmware/usb_config.o `sdl-config --libs` -lusb-1.0

