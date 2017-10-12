# LidarLite_Arduino

### Installation


    1)Follow the instructions for installing rosserial_arduino
    2)Copy the drivers from this_package/lib to the Arduino libraries in <sketchbook>/libraries
    3)Compile the ros_lidalite.ino file with the arduino IDE

### Run

Start a roscore and use:

    rosrun rosserial_python serial_node.py /dev/ttyACM0

(The port can change of course)

Double check the wiring, especially the SDA/SCL pins, if you have errors with the incoming data.
The corresponding pins are described in the documentation of arduino and the documentation from lidarlite.

