# Rosserial on STM32 Discovery board using ChibiOS

This is a rosserial demo for the STM32F407 discovery board.

## Quickstart

From the root of this project, generate the ROS message headers
```bash
rosrun rosserial_client make_libraries src
```

Then you can build and flash the board
```bash
git submodule init
git submodule update

make
make flash
```

Now we can run the node
```bash
roscore &
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1
```
Replace `/dev/ttyACM1` by the port used to communicate with your board.

In a new terminal, you can try out the button
```bash
rostopic echo /button
```
Everytime you push the button, a message is published

Or you can look at the accelerometer values being published
```bash
rostopic echo /accelerometer
```

Finally, you can control the LEDs intensities by publishing a `std_msgs/UInt16` to one of these topics
```bash
rostopic pub /led/red std_msgs/UInt16 --once 1000
rostopic pub /led/blue std_msgs/UInt16 --once 1000
rostopic pub /led/green std_msgs/UInt16 --once 1000
rostopic pub /led/orange std_msgs/UInt16 --once 1000
```
