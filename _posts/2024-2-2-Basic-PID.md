---
title: Basic PID
author: Prahas
date: 2024-02-2 08:03:00 -0700
categories: [PID, Introductory]
tags: [software, pid, beginner]     # TAG names should always be lowercase
---
# PID

*How do you control motors quickly and efficiently?*

PID is a method of controlling motors using sensor data. While it's often viewed as a complicated process, the code behind it can be relatively simple. This tutorial explains the math behind PID and outlines how to create a basic PID controller similar to ones used in large libraries like LemLib and JAR.

PID stands for 

> **P**roportional
  **I**ntegral
  **D**erivative
>

***

# Proportional

To begin with, all PID calculations are based off of the `error`. The error is the difference in the target and the current sensor value of the motor. 
`error = target - current`
So for example, if we want the motor to move to 100 degrees, but the motor is currently 20 degrees, the
`error = 100 - 20 = 80`

The first and simplest term in PID is the proportional term. The proprtional term is as the name suggests. It is proportional to the error. `proportional = kP * error` where kP is a constant that needs to be tuned by the user. Too low will make the motor reach the target slowly while too high can cause the motor to go past the target and oscillate. Right now, the output would be only the proportional. This alone is better as opposed to a singular speed for motor control because with a P loop, the motor essentially goes faster when far away from a target and slower when closer to the target

## Writing an Implementation

### Finding Motor Travel

The first step is measuring how much progress the motors have made. This can be done using motor encoders. Finding this value will be different for every use case of PID. For example, for linear flywheel PID, you might use `double pros::Motor::get_actual_velocity ( )` while for lifts, you might use `double pros::Motor::get_position();`

```cpp
#include "main.h"

pros::Motor lift(1, pros::E_MOTOR_GEARSET_18); // motor at port 1, green gearbox
// function to get lift motor to the target angle
void P_loop(float target, float kP) {
    float error = 0;
    float P = 0;
    while (error != 0) {
        error = target - lift.get_position(); // update error
        P = error * kP;                       // update proportional
        lift.move(P);                         // update motor velocity
    }
}

void autonomous() {
    P_loop(90, 0.6); // move lift motor to 90 degrees with kP = 0.6
}
```

This code is mostly ok except for one issue. The loop only runs while the error is not equal to 0. The issue is that the loop will run forever as it isn't possible nor necessary to get the motor to such a precise number. This can be solved by making a range of values when the controller can exit or otherwise known as a`deadband`. We can check if the absolute value of the error is below deadband like so.

```cpp
float deadband = 1;
while (fabs(error) > deadband)){
    // control loop code
}
```

> fabs() is a standard c++ function that returns the absolute value of a `float` datatype! 
>

This will be the same units as the sensor units you use. So for this example, a deadband of 1 is a settle range of 1 degree. 

# Integral 

The next term in PID is the integral term.

The issue with just using the proportional term is that in some cases, the proportional gets too small when close to the target like in this simulation below.
<iframe src="https://www.desmos.com/calculator/dmkizquwlc?embed" width="500" height="500" style="border: 1px solid #ccc" frameborder=0></iframe>

This is called steady-state error and in real life, sometimes the motor halts right before the target.

The integral term can help fix this. The `integral` is the area under a function on a graph like below.
<iframe src="https://www.desmos.com/calculator/ite1iqprlv?embed" width="500" height="500" style="border: 1px solid #ccc" frameborder=0></iframe>

Since we are integrating the error, the integral is basically the sum of the error. Thus, integral can be calculated simply by summing up the error.

```cpp
float integral = 0;

while...// inside the loop
inegral += error;
```

Sometimes the  `integral` term may be too high or too low which is why there is also a constant for the integral-`kI`.

```cpp
float I = integral * kI;
```

## Implementation

```cpp
#include "main.h"
#include  <cmath>  // Include cmath for fabs()
pros::Motor lift(1, pros::E_MOTOR_GEARSET_18); // motor at port 1, green gearbox

// function to get lift motor to the target angle
void PI_loop(float target, float deadband, float kP, float kI) {
    float error = 0;
    float integral = 0;
    float P = 0;
    float I = 0;
    while (fabs(deadband > error)) {
        error = target - lift.get_position();
        integral += error;
        P = error * kP;
        I = integral * kI;
        lift.move(P + I);
    }
}

void autonomous() {
	// move lift motor to 90 degrees with deadband = 1, kP = 0.6, and kI = 0.1
    PI_loop(90, 1, 0.6, 0.1); 
}
```

# Derivative

The derivative is the rate of change of a function. If a function is increasing very quickly, the derivative will be large. Otherwise, if a function is decreasing, the derivative will be negative. 

Because the derivative is the rate of change of the error, when it is added to the output, it will increase the motor power faster when accelerating and will  increace the deacceleration amount.

But it can also be flipped so that it is negative while the motor power is increasing and positive while the motor is deccelerating. This can lead to the velocity of the motor dampening and more accurate/controlled results.

If the constant for the derivative or `kD` is tuned correctly, you can see results like this.
<iframe src="https://www.desmos.com/calculator/yxxnbekjuj?embed" width="500" height="500" style="border: 1px solid #ccc" frameborder=0></iframe>

>Here, the robot reaches the target in a very controlled matter.
>

The derivative in this context can simply be calculated by subtracting the error from the previous error like so.

```cpp
float derivative = previousError - error;
```

The previous error can be found by updating it to the error after the derivative is calculated.

```cpp
while...//control loop

float derivative = previousError - error;
previousError = error;
```

# The Full PID Implementation

```cpp

#include "main.h"
#include  <cmath>  // Include cmath for fabs()
pros::Motor lift(1, pros::E_MOTOR_GEARSET_18); // motor at port 1, green gearbox

// function to get lift motor to the target angle
void PID(float target, float deadband, float kP, 
									   float kI, 
									   float kD) {
    float error = 0;
    float integral = 0;
    float derivative = 0;
    float prevError = 0;
    float P = 0;
    float I = 0;
    float D = 0;
    
    while (fabs(deadband > error)) {
        error = target - lift.get_position();
        integral += error;
        derivative = prevError - error;
        P = error * kP;
        I = integral * kI;
        D = derivative * kD;
        lift.move(P + I + D);
		
		prevError = error;
    }
}

void autonomous() {
    PID(90, 1, 0.6, 0.2, 0.05); 
}

```

# A More Modular PID design

You can write a c++ `class` for the PID.  Doing so has many benefits. With c++ classes, you can create multiple objects of the same PID. So in other words, you write the PID code once but it can get used for any other function.
## Constructor

To begin with, inside the class, there should be a constructor like so.

```cpp
class PID {
    // Constructor
    PID(float kP, float kI, float kD) 
    : kP(kP), 
      kI(kI), 
      kD(kD), 
      integral(0), 
      prevError(0) {}
};
```
By default, classes have their contents as `private:` if not specified. `private:` variables and functions can not be accessed from outside which obviously isn't helpful. This is easily solved using `public:`
```cpp
class PID {
	public:
    // Constructor
    PID(float kP, float kI, float kD) 
    : kP(kP), 
      kI(kI), 
      kD(kD), 
      integral(0), 
      prevError(0) {}
};
```

So what we have above is a constructor. This means that values can be passed into the class upon creation of the object. So normally, you would have to do this:

```cpp
PID pid;
pid.kP = 1;
pid.kI = 1;
pid.kD = 1;
```

But with constructors,

```cpp
PID pid(1,2,3); 
// constructors also get called at the creation of an object!
```

Everything after the `:` are the initial values. `kP`, `kD`, and `kI` initial values are inputted. The `integral` and `prevError`initialize at 0 which also means that they can not be initialized with an input. 

## Member Functions

The next step is to make a function for calulating the output. It can be the same as previously mentioned but instead of actually controlling the motor, it outputs the velocity for the motor. The function should be `public:` so it can be accessed externally. 

```cpp
#include "main.h"

class PID {
    public:
    // Constructor
    PID(float kP, float kI, float kD) 
    : kP(kP), 
      kI(kI), 
      kD(kD) {}

    // Member function to update PID
    // This should be used in a control loop for a motor!
    float update(const float error) { 
        // calculate integral
        integral += error;

        // calculate derivative
        const float derivative = error - prevError;
        prevError = error;

        // calculate output
        return error * kP + integral * kI + derivative * kD;
    }
    
};
```

It is a good idea to makea `reset()` function for the `PID` class. This function should reset just the `integral` and `prevErorr` to `0` because when the loop exits, the `integral` and `prevErorr` will still be based on the previous loop. The `reset()` function can simply be:

```cpp
void reset() {
	integral = 0;
	prevError = 0;
}
```

This can also be `public:` in the public keyword.
## Member Variables
All the variables can be kept within the `protected:` keyword.  This means that the variables will be inaccessable from outside but accessable by the other member functions in future uses.

```cpp
    protected:
    // Member variables
    float kP;
    float kI;
    float kD;
    float integral = 0;
    float prevError = 0;
```
> Note: There is also a `private:` keyword that is similar to `protected:` but the private variables can not be accessed by other member functions in future uses.
>

## Full Implementation

```cpp
using namespace std;
#include <cmath> // Include cmath for fabs()

class PID {
    public:
    // Constructor
    PID(float kP, float kI, float kD) 
    : kP(kP), 
      kI(kI), 
      kD(kD) {}

    // Member function to update PID
    float update(const float error) {
        // calculate integral
        integral += error;

        // calculate derivative
        const float derivative = error - prevError;
        prevError = error;

        // calculate output
        return error * kP + integral * kI + derivative * kD;
    }

    // Member function to reset PID
    void reset() {
        integral = 0;
        prevError = 0;
    }

    protected:
    // Member variables
    float kP;
    float kI;
    float kD;
    float integral = 0;
    float prevError = 0;

    // Member functions can also be added here!
    
};
```

This now lets you create a `PID`object initialized with `kP`,`kD`, and `kI`. You can make however many you want and use it for whatever you want.

> The settling conditions can be created outside since the settling conditions vary from use to use. 

## Examples

Lift example:

```cpp
pros::Motor lift(10, pros::E_MOTOR_GEARSET_18); // port 10, 200 RPM

void autonomous() {
	// construct liftPID off of PID class
    PID liftPID(1, 0.2, 2.5); 
    float deadband = 1; // deadband from earlier
	// settled bool to check if PID should be exited
    bool settled = false; 
    float target = 45;
    float motorPower;
    while (!settled) {
        float error = target - lift.get_position();
        motorPower = liftPID.update(error);
		// if error is less than deadband...
        if (fabs(error) < deadband) { 
            settled = true; // exit
        }
    }
    liftPID.reset();
    
}
```

Flywheel example

```cpp
pros::Motor flywheel(10, pros::E_MOTOR_GEARSET_06); // port 10, 600 RPM

void autonomous() {
    PID fwPID(1, 0.2, 2.5); // construst liftPID off of PID class
    float target = 450;
    float motorPower;
    while (true) {
        float error = target - flywheel.get_actual_velocity();
        motorPower = fwPID.update(error);
    }
    fwPID.reset();
}
```

> Flywheel control loops are often only manually exited
