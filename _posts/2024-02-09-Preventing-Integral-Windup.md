---
title: Preventing Integral Windup
author: Josephine
date: 2024-02-09 04:15:00 -0800
categories: [PID]
tags: [software, pid]
math: true
---

In a traditional PID loop, the I or integral term is mostly used to maintain the state of the system. When creating a PID loop that exits, such as for moving forwards or turning to a heading, we don't need this holding functionality from the integral term. However, many will still choose to use I because it can help reduce steady state error. Unfortunately, in this unconventional kind of PID loop, one major downside of using integral is how easily it introduces integral windup.

Integral windup occurs when the integral term becomes excessively large and is typically a result of error accumulating faster than the system can handle it. This isn't ideal as it can oversaturate the controller and lead to overshooting the target. A few common methods of preventing integral windup are outlined in this post.

*These methods are listed separately but can/should be combined for better results.*

## Conditional integration looking at the error value
We can prevent a large accumulation of error from the start of the loop by limiting when the integral term can be added to. The best way to do this is by looking at how large the error value is and only integrating when the robot is near the setpoint. Implementing this is as simple as restricting changes to the integral variable to a certain range of error:

```cpp
if(fabs(error) < range){
    integral += error;
}
```

This allows the benefits of the integral term—mainly, reducing steady state error—to still be applied when needed, but it reduces the risk for an excessively large I value that causes overshooting.

## Capping the integral value
Alternatively, you can simply cap the integral value by only allowing it to increase up to a certain amount. With tuning, this can make sure the output isn't oversaturated and keeps the integral value from building up infinitely (though it should be noted that without combining this with conditions for integration, the integral value may max out way before getting near the setpoint and have little effect in the last portion of the loop).

The snippet below includes ensuring the integral value is less than an assigned maximum value before it adds the current error.

```cpp
if(integral < integralCap>){
    integral += error;
}
```

## Resetting integral at crossing
Consider what happens when you've been accumulating error since the start of your movement but have overshot your setpoint by just a little bit. At this point, the proportional term would be trying to move the robot back towards the setpoint but the integral term would still be contributing towards the original direction of movement. The proportional and integral terms would essentially be working against each other, increasing your overshoot and generally negating the desired effects of the integral term. As the title may imply, this method resets the integral value every time the setpoint is crossed to ensure the integral term is working in the right "direction".

Below is a simple implementation of this method, where sign(value) returns 1 if the value is positive and -1 if the value is negative.

```cpp
if(sign(error) != sign(prevError)){
    integral = 0;
    integral += error;
}
```