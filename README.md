# Flix

this is an esp32 based quadcoptor
The actual code is taken from [flix/okalachev](https://github.com/vishwa-r-03/flix_quadcoptor.git) and refactored to be supported in platform io. 
Ive also added proper gyro calliberation without which the drone was flying irratically.

Visit the original repository to find all the detailed description, documentation, tools and controls. As all of those are the same here as well.
the only changes being this code is refactored for VS code running platformio, a few extra lines of codes for proper gyro caliberation.

### to caliberate the gyro: 

place the drone on a level surface and from the mavlink console type 'cg' and press enter, the gyro bias will be calculated and saved to the existing parameter values.

