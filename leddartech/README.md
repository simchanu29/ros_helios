Leddartech
==========

In order to be able to connect to the leddar device with the actual Ros package, follow these simple steps :

	1) sudo useradd -G plugdev USERNAME
	2) sudo cp THIS_PACKAGE_DIR/rules/10-leddartech-rules /etc/udev/rules.d/
	3) sudo udevadm trigger

For the modifications, for 3D mapping and fuse with pixhawk imu etc, you may need additionnal packages.
Usually, the missing one is ros_laser_assembler:

    sudo apt-get install ros-kinetic-laser-assembler

If you get an error like LIBUSB_ERROR_CONNECT, it's most likely due to the program not enabled to access the port.

One solution to this problem:
Create a file /etc/udev/rules.d/90-usbpermission.rules and write:

    SUBSYSTEM=="usb",GROUP="users",MODE="0666"

