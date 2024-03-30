#
# Makefile for the LEGO WeDo driver (if driver is inside kernel tree).
#

ccflags-y += -I $(srctree)/drivers/lego/include

obj-$(CONFIG_WEDO) += wedo.o

wedo-y := wedo_hub.o wedo_port.o wedo_sensor.o wedo_motor.o wedo_servo.o
