# Flix

this is an esp32 based quadcoptor
The actual code is taken from [flix/okalachev](https://github.com/vishwa-r-03/flix_quadcoptor.git) and refactored to be supported in platform io. 
Ive also added proper gyro calliberation without which the drone was flying irratically.

Visit the original repository to find all the detailed description, documentation, tools and controls. As all of those are the same here as well.
the only changes being this code is refactored for VS code running platformio, a few extra lines of codes for proper gyro caliberation.

## Current status

The drone in its current situation flies somewhat better than before. it is stable and flies okay. Though the responsiveness when controlled from the smartphone is too much and can be hard for beginners to control. but it can be adjusted by changing the max roll and pitch values in the parameter settings inside the ground control unit.

There is no altitude hold and sense of direction as we are using MPU6050. This can be improved greatly if we add magnetometer ( for orientation control ) and a time of flight sensor for altitude adjustments. 


### In Progress:

Currently Im adding an MPU9250 for orientation control to get more accurate yaw control and stabilisation.

```
note: Most breakout boards which we buy online which are labelled "MPU9250/6500" or "MPU 92/XX" are not actual MPU9250 breakout boards, instead they are modified 6500 boards and are mostly faulty or doesnot have a magnetometer in it. so make sure to buy verified sensors from trusted sources. Otherwise it can be a big hassle while debugging and incorporation of the module.
```

### To caliberate the IMU: 

place the drone on a level surface and follow these steps in the correct order.

1. Type and send 'imu' from the mavlink console. This should print out the status & model of the IMU and all the accel and gyro values and their scales and biases.
2. Now calliberate the gyro by sending 'cg' from the console. This should print out the gyro bias values and automatically update them.
3. Now for the accelerometer type 'ca' and follow the instructions one by one which will be printed on the screen.
4. Finally, type 'imu' again to see all these calliberated values being taken into effect.

```
note: After disarming the drone, wait 5 seconds before re arming the drone for the next flight.
```
