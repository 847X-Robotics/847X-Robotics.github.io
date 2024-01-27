---
title: Basic Odometry
author: Tropix
date: 2024-01-24 10:12:00 -0700
categories: [Odometry, Introductory]
tags: [software,odometry]     # TAG names should always be lowercase
image:
  path: https://i.postimg.cc/grFzjG4w/Tropical-pfp.png
  width: 640 #correct image size
  height: 480
---

# Odometry

*How does a robot know where it is?*

Odometry is a method of tracking the absolute position of a robot on a field using sensor data. While it's often viewed as a complicated process, the code behind it can be relatively simple. This tutorial explains the math behind odometry and outlines how to create a basic position tracking function that works similarly to the one used in taolib.

## The basics

The general idea behind tracking the robot's position is as follows:
* Measure the total distance that the robot has traveled using wheel encoders.
* Measure the absolute heading of the robot using either encoders or a gyroscope (we'll be using the [V5 Inertial Sensor](https://www.vexrobotics.com/276-4855.html)).
* Create a loop that measures the distance that our wheel encoders have traveled from the last loop run, then imagine that distance as the hypotenuse of a right triangle. The robot's current heading at the time of the loop is the base angle of the triangle.
* By forming this triangle, some basic trigonometry can be used to find the length of the triangle's opposite and ajacent sides. The length of these sides are how far the robot has traveled in the X and Y direction.
* Repeat this process using our loop at a very fast rate (ideally once every 10 milliseconds).

To visualize the last two steps, let's use an image:

![Odometry Math](https://i.imgur.com/4Edofqi.png)

To find the change in X and Y, we'll use the following formulas.

```cpp
change_in_x = change_in_distance * cos(heading)
change_in_y = change_in_distance * sin(heading)
```

## Let's write an implementation

### Finding Wheel Travel

The first step is measuring how far the wheels have traveled forward or backward. This can be done using motor encoders.

We can get the degrees of travel from each encoder using `Motor.position(vex::degrees)` (this also works for motor groups). For the most accuracy, we'll take the average of all motors in the drivetrain and use that as our total distance value.

```cpp
double average_encoder_position = (LeftMotors.position(vex::degrees) + RightMotors.position(vex::degrees)) / 2;
```

Now we have data on how far the encoders have turned in degrees, but how do we convert those degrees to a distance unit like inches?

Every time a motor encoder spins a full 360 degrees (1 "revolution"), the drivetrain will have traveled the circumference of the wheel being driven. We aren't quite done yet, though. If the drivetrain has a [gear ratio](https://wiki.purduesigbots.com/hardware/design-fundamentals/gear-ratios), then we need to factor that into the distance equation. The final formula for degrees to distace is `(average_encoder_position / 360) * wheel_circumference * external_gear_ratio)`.

```cpp
#include <cmath>

constexpr double gear_ratio = ((double)84/60); // Using an 84/60 gear ratio for this example.
constexpr double wheel_radius = 2;
constexpr double wheel_circumference = 2 * M_PI * wheel_radius;

double average_encoder_position = (LeftMotors.position(vex::degrees) + RightMotors.position(vex::degrees)) / 2;
double distance_traveled = (average_encoder_position / 360) * wheel_circumference * gear_ratio;
```

### Finding Heading

The next thing we need is to determine where the robot is facing. The easiest way is to use the gyroscope of the [V5 Inertial Sensor](https://www.vexrobotics.com/276-4855.html), so we'll go over that first.

>  *WARNING:* Be sure to [calibrate your Inertial Sensor](https://www.vexforum.com/t/how-do-i-calibrate-inertial-sensor-in-pre-auton/74040) at the start of `pre_auton`, or else the headings that it reports will be wrong!

You can find the absolute heading of the drivetrain in degrees using the `Inertial.heading(vex::degrees)` function. There are a few things wrong with this value, though. First, the sensor reports heading as increasing in the clockwise direction, meaning that heading will *increase* as the robot turns *right*. For the purposes of position tracking, this will alter how our robot's coordinates are reported, so this value will need to be converted to a counterclockwise angle.

```cpp
double heading = 360 - Inertial.heading(vex::degrees);
```

The next problem is that the sensor treats "0 degrees" as the robot's starting angle. This is generally a bad idea for position tracking, because we want "forward" to be up on the y-axis for easy visualization. **In most cases, you want to imagine the robot's starting angle as 90 degrees.**

![Starting at 0 degrees vs starting at 90 degrees](https://i.imgur.com/tZE6Mxy.png)

To make this adjustment, we can simply add `90` to the heading value after converting it to coutnerclockwise degrees. We'll also use the `std::fmod` function of the standard library to wrap the heading back to 0 once we reach 360 degrees after offsetting.

```cpp
#include <cmath>

double start_heading = 90;

// Using std::fmod to preserve the "wraparound effect" of 0-360 degrees when considering the offset of start heading.
double heading = std::fmod((360 - Inertial.heading(vex::degrees)) + start_heading, 360);
```

### Finding Heading (without an inertial sensor)

If you don't have access to an inertial sensor, then absolute heading *can* be estimated using wheel encoders only. Keep in mind that this will be less accurate than if you had an appropriate sensor.

The formula for finding heading with encoders is as follows:

```
heading_in_radians = (right_distance - left_distance) / track_width
```

Right and left distances can be found similarly to how `distance_traveled` is found, but using only one side of the drivetrain.

```cpp
double left_distance = (LeftMotors.position(vex::degrees) / 360) * wheel_circumference * external_gear_ratio;
double right_distance = (LeftMotors.position(vex::degrees) / 360) * wheel_circumference * external_gear_ratio;
```

Next we need to find track width. Track width is a measurement of the distance between the center of your left and right wheels. This measurement will be unique to different drivetrains, and should be measured in the same units that `wheel_radius` is measured with.

![Track width measurement](https://kb.vex.com/hc/article_attachments/360085801912/track_width.jpg)

For the purpose of example, let's say our track width is 13.75 inches. We can now find our absolute heading in radians.

```cpp
constexpr double track_width = 13.75;

double left_distance = (LeftMotors.position(vex::degrees) / 360) * wheel_circumference * external_gear_ratio;
double right_distance = (LeftMotors.position(vex::degrees) / 360) * wheel_circumference * external_gear_ratio;

double heading_in_radians = (right_distance - left_distance) / track_width;
``` 

The next step is to convert the heading to degrees and restrict it to 0 <= x < 360 (if the heading exceeds 360 degrees it will be reset to 0). This will mimick the heading reported by an inertial sensor's gyro.

We'll also add a `start_heading` of 90 degrees (explained in the previous section).

```cpp
#include <cmath>

constexpr double track_width = 13.75;
constexpr double start_heading = 90;

double left_distance = (LeftMotors.position(vex::degrees) / 360) * wheel_circumference * external_gear_ratio;
double right_distance = (LeftMotors.position(vex::degrees) / 360) * wheel_circumference * external_gear_ratio;

double heading_in_radians = (right_distance - left_distance) / track_width;

double heading = std::fmod((360 - (heading_in_radians * (180 / M_PI))) + start_heading, 360);
```


## Making a loop

Now that we can measure the correct heading and distance values, let's put these together into a loop that can track the change in X and Y position.

We can create a basic loop inside of a function like so:

```cpp
#include "vex.h"

double x = 0;
double y = 0;

void odometry() {
    while (true) {
        vex::this_thread::sleep_for(10);
    }
}
```

This creates a `while`-loop that will run forever and wait 10 milliseconds in between each cycle. We store global `x` and `y` variables for keeping track the robot's absolute x and y position.

> We wait 10 milliseconds for two reasons. First, an infinite loop on the V5 Brain without a wait condition will hog resources until the program freezes. Second, motor encoders can only report their values once every 10 milliseconds, meaning it would be wasteful to run these calculations any faster.

We can now introduce our sensor readings to the loop.

```cpp
#include <cmath>
#include "vex.h"

constexpr double gear_ratio = ((double)84/60); // Using an 84/60 gear ratio for this example.
constexpr double wheel_radius = 2;
constexpr double wheel_circumference = 2 * M_PI * wheel_radius;
constexpr double start_heading = 90;

double x = 0;
double y = 0;

void odometry() {
    LeftMotors.resetPosition();
    RightMotors.resetPosition();

    while (true) {
        double heading = std::fmod((360 - Inertial.heading(vex::degrees)) + start_heading, 360);

        double average_encoder_position = (LeftMotors.position(vex::degrees) + RightMotors.position(vex::degrees)) / 2;
        double distance_traveled = (average_encoder_position / 360) * wheel_circumference * gear_ratio;

        vex::this_thread::sleep_for(10);
    }
}
```

Every time the loop runs we'll find the new heading and travel distance. These are the numbers that will eventually be used for the odometry formula.

### Putting it together

Let's perform the odometry calculations to find the change in x and y position. The calculations use our absolute heading and the distance that the robot has traveled since the *last loop cycle* (Δdistance).

We can find this change distance by storing the distance that the robot traveled at the start of the last loop cycle, and subtracting it from the current distance reading in the next cycle.

```cpp
void odometry() {
    LeftMotors.resetPosition();
    RightMotors.resetPosition();

    double previous_distance_traveled = 0;

    while (true) {
        ...
        double distance_traveled = (average_encoder_position / 360) * wheel_circumference * gear_ratio;
        double change_in_distance = distance_traveled - previous_distance_traveled;

        ...

        // At the end of the loop, set previous_distance_traveled for the next loop iteration
        previous_distance_traveled = distance_traveled;

        vex::this_thread::sleep_for(10);
    }
}
```

This effectively tells us how far the robot has traveled in the 10 milliseconds it took to calculate the new positions. Now that we have this reading, we can use it to find our change in x and y.

> Keep in mind that C++'s trig functions operate on radians, so when providing heading we should multiply `heading` by `M_PI / 180` to convert degrees to radians.

```cpp
x += change_in_distance * std::cos(heading * (M_PI / 180));
y += change_in_distance * std::sin(heading * (M_PI / 180));
```

Adding everything together, your loop should now look like this:

```cpp
#include <cmath>
#include "vex.h"

constexpr double gear_ratio = ((double)84/60); // Using an 84/60 gear ratio for this example.
constexpr double wheel_radius = 2;
constexpr double wheel_circumference = 2 * M_PI * wheel_radius;
constexpr double start_heading = 90;

double x = 0;
double y = 0;

void odometry() {
    LeftMotors.resetPosition();
    RightMotors.resetPosition();

    double previous_distance_traveled = 0;

    while (true) {
        double heading = std::fmod((360 - Inertial.heading(vex::degrees)) + start_heading, 360);

        double average_encoder_position = (LeftMotors.position(vex::degrees) + RightMotors.position(vex::degrees)) / 2;
        double distance_traveled = (average_encoder_position / 360) * wheel_circumference * gear_ratio;

        double change_in_distance = distance_traveled - previous_distance_traveled;
        
        x += change_in_distance * std::cos(heading * (M_PI / 180));
        y += change_in_distance * std::sin(heading * (M_PI / 180));

        // At the end of the loop, set previous_distance_traveled for the next loop iteration
        previous_distance_traveled = distance_traveled;

        vex::this_thread::sleep_for(10);
    }
}
```

We now have a loop that will track the robot's absolute coordinates. But what next? How should we run it?

### Starting odometry as a thread

To use the coordinates found in our loop, we want to run the `odometry` function in the background. We do this using a technique called [threading](https://en.wikipedia.org/wiki/Multithreading_(computer_architecture)). Using a separate thread means we can get position updates while moving the robot around or running an autonomous routine.

We'll start our odometry thread at the beginning of the autonomous period by creating a `vex::thread` instance, and passing in our `odometry` function:

```cpp
// Autonomous function found in the VEX competition template
// If you aren't using the competition template, feel free to start this in main().
void autonomous() {
    // Start our odometry thread.
    // The odometry loop will run in the background while we move.
    vex::thread odometry_thread(odometry);

    // Spin our drivetrain forward for 2 seconds.
    LeftMotors.spin(vex::forward);
    RightMotors.spin(vex::forward);

    vex::wait(2, vex::seconds);

    LeftMotors.stop();
    RightMotors.stop();

    // Print where we ended up on the coordinate plane onto the brain screen.
    Brain.Screen.print("(%f, %f)", x, y);
}
```

### Tracking Wheels

Some teams may opt to use tracking wheels rather than integrated motor encoders. These are unpowered wheels attached to the bottom of the drivetrain that are hooked to external encoders. Using deadwheels has the advantage of preventing wheel slipping from messing up the readings for finding wheel travel distance.

![Tracking wheels](https://www.vexforum.com/uploads/default/original/2X/a/a90abb6db94c9fe8bddc0c284bc9f69bda97bd6e.png)

The math for finding distance traveled using tracking wheels is almost exactly the same as with motor encoders, except tracking wheels always have an implied 1:1 gear ratio:

```cpp
constexpr double tracking_wheel_radius = 2;
constexpr double tracking_wheel_circumference = 2 * M_PI * tracking_wheel_radius;

double average_encoder_position = (LeftEncoder.position(vex::degrees) + RightEncoder.position(vex::degrees)) / 2;
double distance_traveled = (average_encoder_position / 360) * tracking_wheel_circumference;
```
