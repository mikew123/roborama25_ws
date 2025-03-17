# DESCRIPTION
This ROS2 code is for the ROBORAMA 2025 Indoor robot competition<br>
The 2024 robot was stripped down and the motors, servos, camera, 3 VL53L5 sensors, IMU were reused as well as the power, wheel and sensor modules<br>
New wheels and a RPLidar C1 and another VL53L5 sensor and a VL53L4 sensor were added<br>
It has a new plywood base which is about 16" diameter (larger than my 3D printer can make)<br>
I am reusing and updating the wheel and sensor ROS2 modules<br>

## Pictures:
Top of robot:<br>
![Robot top](support/RoboRama25_Robot_top_pic.jpg) <br>
Bottom of robot:<br>
![Robot top](support/RoboRama25_Robot_bot_pic.jpg) <br>

# Documentation
## Block diagram of electronics:
The Rasberry Pi does not supply power directly to any of the peripherals<br>
The peripheral power is supplied from the power supply module<br>
The powered USB hub supplies power for to the peripherals connected to it<br>
![Block diagram](support/RoboRama25_Block_Diagram.jpg) <br>

## Schematics:
Power Supply board:<br>
There are 3 voltage regulators. 12V for the wheel motors. A 5V for the Pi and 5V for the peripherals<br>
The Peripheral 5V are bumped up 0.7 volts or so to compensate for the reverse protection diodes<br>
![Power Supply board](support/RoboRama25_Power_Board.jpg) <br>

Front Sensors Controller board:<br>
The 5V input has a reverse protection diode, the 5V input level is expected to be 5.7V or so to compensate for the diod drop. The 3.3V to the sensors is from a regulator since they need more power than the uC internal regulator can safely generate. The 3.3V regulator can be disabled to reset the VL53L4CX sensors before assigning I2C addresses. The RP2040 module 3.3V output is not used.<br>
The four VL53L4CX sensors are on the I2C0 bus and the I2C1 bus is a QT connector which connects to the other sensors.<br>
The wiring of the VL53L4CX sensors is shown on the schematic, this wiring is not on the Front Sensor Controller board.<br>
![Front Sensor Controller](support/RoboRama25_Front_Sensor_Controller_Schematic.jpg) <br>

Wheels Controller board:<br>
The 5V input has a reverse protection diode, the 5V input level is expected to be 5.7V or so to compensate for the diod drop. The RP2040 module 3.3V output powers the pod encoder sensors which do not require a lot of power.<br>
The 12V powers the MP6550 Wheel motor controllers which are connected with a 6 pin cable. The cable also has the motor encoder signals that are powered by 5V and the level shifter connects the signals to the 3.3V GPIO pins of the controller. There is also 4 pin connectors to pod odomtry sensors used on the 2024 robot but not this year.<br>
The 3 pin connectors are for the servos that control the claws operate on 5V and work with the 3.3V GPIO signals. There is a set 2 pin connectors for the limit switches that trip when the can is firmly gripped by the claws, the GPIO pins are configured with internal pull-up resistors so the switches pull the pin levels low when tripped.<br>
![Wheels Controller board](support/RoboRama25_Wheels_Controller.jpg) <br>
