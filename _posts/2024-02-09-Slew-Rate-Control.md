---
title: Slew Rate Control
author: Josephine
date: 2024-02-09 04:30:00 -0800
tags: [software]
math: true
---

## What is slew rate?
In electronics, slew rate is the maximum rate of change of voltage being output. If it helps, this would be the derivative of voltage with respect to time, or the slope of a voltage over time graph. In autonomous movements, controlling the slew rate of the drive motors can be used to limit the acceleration of the robot.

For example, if your robot is still and you try to accelerate suddenly to full speed, you're trying to change the voltage of the drive motors from 0 to 127 (in PROS's scaled voltage measurements). If you had a slew rate controller with the slew rate set to 4, it would take 31.75 time units for your robot to reach full speed because voltage would only increase by 4 voltage units/time unit.

## Implementation
There are a few ways to implement slew rate control. One is a constant loop running in the background that prevents the voltage of the motors from increasing too quickly. The one shown below is instead a function that takes in the desired voltage, compares it to the last supplied voltage, and returns the voltage that accounts for slew rate, which the user/function can then apply to the motors.
```cpp
int slewControl(int desiredVoltage, int previousVoltage, int slewRate){
    if(desiredVoltage != previousVoltage){
        if (desiredVoltage - previousVoltage > slewRate) {
            return previousVoltage + slewRate;
        }
        if (desiredVoltage - previousVoltage < -slewRate)
        {
            return previousVoltage - slewRate;
        }
    }
    return desiredVoltage;
}
```
You would then be able to add this when trying to set motor voltage:
```cpp
motor1.move(slewControl(desiredVoltage, previousVoltage, slewRate));
```

## Optimizations
### Tracking Previous Voltage
PROS scales the values you use to set voltage to map to the input from the V5 controller (i.e., voltage is set out of 127). This is convenient for writing driver control code but makes it difficult to base output off of how much voltage is currently being sent. For this reason, it can be better to track the previous voltage you've set, rather than trying to use get_voltage() and convert from millivolts.