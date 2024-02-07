---
title: Basic Motion Profiling
author: Josephine
date: 2024-02-07
categories: [Introductory]
tags: [software, beginner]
---

# Basic Motion Profiling
A smoother, open-loop alternative to PID for linear movement is motion profiling. The benefit to using motion profiles over just PID is an easy way to control acceleration. Motion profiles allow control over the maximum acceleration and deceleration of a robot and pre-generates a set of velocities for the robot to follow to travel under those constraints.


## Trapezoidal Motion Profiles

A trapezoidal motion profile is the most basic kind of profile and takes into account only velocity and acceleration. More advanced profiles like an S-curve profile can account for jerk in addition to these.

![Position and velocity v. time graph](https://i.imgur.com/lN8peLv.png)

### Math
A motion profile is broken into three parts: acceleration, cruising, and deceleration. These can be represented using the same graph from above:
![Acceleration, cruising, and deceleration separated](https://i.imgur.com/QMlZ64h.png)
To calculate the time and distances for these, we can go back to our basic kinematics equations:
$$x(t) = x_{0} + v_{0}t + \frac{1}{2}t^2\\
v(t) = v_{0} + at$$

### Math Implementation
We can put the equations form above into code to set up a very basic motion profile:
```cpp
// We first find the amount of time it takes to accelerate to max velocity
accelTime  =  maxVel  /  maxAccel;

// Using the acceleration time, we can find the distance it would take to reach max velocity
accelDist  =  maxVel  * accelTime;

// Since acceleration time = deceleration time in a trapezoidal profile, we can set these equal
decelTime  =  accelTime;
decelDist = accelDist;

// Then calculate the cruising distance based on the distance left
cruiseDist  =  dist  -  accelDist  -  decelDist;
cruiseTime  =  cruiseDist  /  maxVel; // Divide by the velocity to get time  
```

You might notice one issue with the above code: what happens when the distance it takes to accelerate to your max velocity is more than half the distance to be travelled? This wouldn't leave enough distance to decelerate to a full stop. We add the following code to account for this, recalculating max velocity as well:
```cpp
if (accelDist  >  dist  /  2)
{
	accelTime  =  sqrt((dist  /  2) / (maxAccel  /  2));
	accelDist  =  0.5  *  maxAccel  *  pow(accelTime, 2);
	maxVel  =  maxAccel  *  accelTime;
}
``` 

### Controlling the Robot
Since we know the velocity of the robot, we can control the robot using either distance or time. Below is a way of getting the desired velocity based on the amount of time the robot has been travelling for:
``` cpp
if (time  <  accelTime) { // if we're accelerating
	return ((maxVel  /  accelTime) *  time);
}

else  if (time  <  decelStart) {
	return  maxVel;
}

else { // if none of the above, we must be in deceleration
	return (maxVel  - ((maxVel  /  accelTime) * (time  - decelStart)));
}
```
After this, it is up to you to find a way to set the robot's velocity to the ideal one. One way to do this is by setting the drive motors' velocity directly; in PROS, this looks like ```motor.move_velocity(velocity).``` You could also create your own velocity PID loop to accomplish this.

### Adding PID
The one major flaw in motion profiling is that it's open loop, meaning that the robot doesn't take in feedback as it drives. To get the benefits of motion profile movement without sacrificing the ability to correct after interference, a common implementation of linear motion profiling will involve a PID loop at the end to make up for any remaining error.