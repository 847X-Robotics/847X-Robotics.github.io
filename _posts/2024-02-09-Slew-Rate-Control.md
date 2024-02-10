---
title: Slew Rate Control
author: Josephine
date: 2024-02-09 04:30:00 -0800
tags: [software]
math: true
---

## The Problem
The search for "perfect" acceleration is trying to find a balance of getting to the desired speed quickly while also avoiding tipping, wheel slippage, or excessive strain on the motor. As robots are heavy and VEX motors are only so strong, trying to go from rest to full speed immediately will result in jerky movements at least, or tipping and even rolling forwards/backwards on robots with poor COG. Additionally, if motor encoders are being used for any kind of tracking or positioning, accelerating too fast typically leads to wheel slippage, producing inaccurate measurements, not to mention over time wears on the motors. In particular, when relying on motor encoders as tracking wheels, it's essential to reduce wheel slippage as much as possible, risking large inconsistencies if not done. For all these reasons, it might be clear why we often try to find ways to limit the acceleration of our movemenets. 

## What is slew rate?
In electronics, slew rate is the maximum rate of change of voltage being output. If it helps, this would be the derivative of voltage with respect to time, or the slope of a voltage over time graph. In autonomous movements, controlling the slew rate of the drive motors can be used to limit the acceleration of the robot.

For example, if your robot is still and you try to accelerate suddenly to full speed, you're trying to change the voltage of the drive motors from 0 to 127 (in PROS's scaled voltage measurements). If you had a slew rate controller with the slew rate set to 4, it would take 31.75 time units for your robot to reach full speed because voltage would only increase by 4 voltage units/time unit.

## Implementation
There are a few ways to implement slew rate control. One is a constant loop running in the background that prevents the voltage of the motors from increasing too quickly. The one shown below is instead a function that takes in the desired voltage, compares it to the last supplied voltage, and returns the voltage that accounts for slew rate, which the user/function can then apply to the motors. We include a dT or timestep variable to account for the fact that the time between applying the previous voltage and the moment the function's running is greater than one time unit.
```cpp
int slewControl(int desiredVoltage, int previousVoltage, int slewRate, int timestep){
    if(desiredVoltage != previousVoltage){
        if (desiredVoltage - previousVoltage > slewRate * timestep) {
            return previousVoltage + slewRate * timestep;
        }
        if (desiredVoltage - previousVoltage < -slewRate * timestep)
        {
            return previousVoltage - slewRate * timestep;
        }
    }
    return desiredVoltage;
}
```
You would then be able to add this when trying to set motor voltage. In a control loop, you can pass along a timestep value to ensure you've accounted for the cycle time of the loop when comparing the change in voltage to your set slew rate.
```cpp
while(true){
    desiredVoltage = // your voltage calculations;
    leftMotors.move(slewControl(desiredVoltage, previousVoltage, slewRate, millis() - prevTime));
    rightMotors.move(slewControl(desiredVoltage, previousVoltage, slewRate, millis() - prevTime));
    previousVoltage = desiredVoltage;
    prevTime = millis();
    delay(timestep);
}

```

## Optimizations
### Tracking Previous Voltage
PROS scales the values you use to set voltage to map to the input from the V5 controller (i.e., voltage is set out of 127). This is convenient for writing driver control code but makes it difficult to base output off of how much voltage is currently being sent. For this reason, it can be better to track the previous voltage you've set, rather than trying to use get_voltage() and convert from millivolts.