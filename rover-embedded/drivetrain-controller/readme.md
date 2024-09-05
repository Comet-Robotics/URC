# Drivetrain Controller
The sole purpose of this controller will be to take movement commands and act on it. It will not make any decisions on when to move what to move etc. We will need to define a protocal for interacting to the controller first over serial than ethernet.


Write now I have it incrementing our throttle then reversing and incrementing reverse very simple.


## Project Setup
Platform.IO IDE on VSCODE

## PWM Specs
Our range for PWM according to these specs [VictorSP](https://link.vex.com/vexpro/docs/VictorSPX-UserGuide)

Frequency: 10hz -> ~350hz
Duty Cycle:
 - 1ms for full reverse
 - 1.5 for 0%
 - 2.0 for full forward


Assuming you take the frequency 50hz
full throttle would be 10%
0% throttle would be 7.5%
full reverse would be 5.0%



### Protocal

 L    R
-100 +100