#
# Makefile for Robotex usbinterface
# Robotex Inc.
# J. Brinton
#	#g++ -o taylor_test taylor_test.o rassert.o -lusb-1.0

COMPILE := 1
LIBS += -lusb-1.0 -L../../libusb-1.0.8/libusb/.libs
DEPS += common/assert


default:

	g++ -c DispenserTest.cpp -o DispenserTest.o
	g++ -c firmware/usb_config.c -o firmware/usb_config.o	
	g++ -o DispenserTest DispenserTest.o firmware/usb_config.o `sdl-config --libs` -lusb-1.0 -lncurses




