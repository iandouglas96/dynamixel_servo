## Dynamixel Servo Driver

Simple driver for a single Dynamixel Serial servo.  Written from the
ground up to be much simpler than the more complex Dynamixel ROS
driver.  Note: This driver currently assumes that the servo has ID 1.
Additionally, it will NOT work with a USB2Dynamixel dongle.  Instead,
take an FTDI, tie together RX and TX, and pass this in on the data
line to the servo

### Modes

The first, and default, mode is sweep.  The endpoints and speed can be
set in dynamic reconfigure.  The sweep can be started or stopped using
the `sweep_mode` topic.

If not in sweep mode, the servo will hold position.  A desired position
can be sent via the `angle` topic.  If a position is received on this
topic, the servo will automatically exit sweep mode.

### Current Position
The driver will print the current servo position at 100Hz on the 
`current_angle` topic.  The first entry in the position field is the
position.  A ROS timestamp is also provided.  All other fields are not
populated.
