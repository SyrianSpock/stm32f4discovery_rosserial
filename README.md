# Rosserial on STM32 Discovery board using ChibiOS

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

In a new terminal you can see the board sending messages over ROS
```bash
rostopic echo /chatter
```
