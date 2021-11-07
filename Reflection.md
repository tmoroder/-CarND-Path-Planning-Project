# Reflection Code

The majority of code was taken directly from the [Q&A video](https://classroom.udacity.com/nanodegrees/nd013/parts/b9040951-b43f-4dd3-8b16-76e7b52f4d9d/modules/85ece059-1351-4599-bb2c-0095d6534c8c/lessons/407a2efa-3383-480f-9266-5981440b09b3/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d). Without this material the project would have been very hard, because I could not see any starting point. In particular the interaction and data provided from/to the simulator was not clear to me. Additionally, in my opinion, the course material was very far off from the parts required for this project.

The code consists of two main parts that I describe in more detail:

- Behavior
- Path generation



# Behavior

In the behavior part we determine the intended lane ``lane`` and velocity ``rel_vel`` of our car in the near future.

This is achieved by checking each lane for cars in the front and in the back of our car. But here we should consider not the actual timestep but the time at the end of our current planned path to follow. Thus one first needs to estimate the position of the other cars in a few timesteps into the future. This is done by assuming that each car continues follwing the street with its current measured velocity. This motion model is most directly expressed in terms of the Frenet s coordinates which propagates to: ``s(t) = s(t=0) + velocity * t`` and a constant Frenet d coordinate. This prediction is again something that has been explained in detail in the Q&A video. With this it is straightforward to check each portion of the road, and we store this in the Boolean vectors  ``is_lane_free_front`` and ``is_lane_free_back`` (see Predict lane status in the code, current line 123).

After one has checked each lane we have implemented the following rule based logic, expressed in verbal terms:

- If the current lane is blocked, then do a lane change to the left or right lane if front and back of that lane are free of cars, otherwise reduce the speed.
- If the lane is free of cars in front, then optionally increase speed if not at the maximum allowed speed limit yet and optionally change to middle lane again. 

We have added the rule to optionally change back to the middle lane because this gives two options to overtake a slower car in the future. This logic can be implemented directly using an if-else control flows:
```c++
bool car_in_front = !is_lane_free_front[lane];
if (car_in_front) {
  // Car in front is too close
  if ((lane > 0) && is_lane_free_front[lane - 1] && is_lane_free_back[lane - 1]) {
    // Left lane change
    lane = lane - 1;              
  } else if ((lane < 2) && is_lane_free_front[lane + 1] && is_lane_free_back[lane + 1]) {
    // Right lane change
    lane = lane + 1;
  } else {
    // Reduce speed
    ref_vel -= 0.224;
  }
} else {
  // No car in front
  if (ref_vel < max_speed) {
    // Increase speed if not max yet
    ref_vel += 0.224;
  } 
  if (lane != 1) {
    // Change to middle lane if possible
    if ((lane > 0) && is_lane_free_front[lane - 1] && is_lane_free_back[lane - 1]) {
      lane = lane - 1;              
    } else if ((lane < 2) && is_lane_free_front[lane + 1] && is_lane_free_back[lane + 1]) {
      lane = lane + 1;
    }
  }
}
```


# Path generation

The path generation was fully explained in the above mentioned material. A path consists of a sequence of (x, y) points that the car is supposed to navigate. In the simulator the car advances to the first given point in ``0.2ms`` and then asks the server for the next sequence.

In order for a smooth transition one appends new path points to end of the previously planned path. In those the newest values of the intended ``lane`` and velocity ``ref_vel`` are taken into account. However in order to achieve a smooth transition to those last point (meaning not exceeding velocity, acceleration and jerk constraints) one applies some curve or function fitting technique. We followed the suggestion of the Q&A and employed the readily available ``spline.h`` file, available at the following [website](https://kluge.in-chemnitz.de/opensource/spline/), and carried out a regression SPLINE. In this fitting process one needs to explicitly provide anchor points or knots for which we used:

- The last two points from the previous path (or computed form the initial heading).
- Additional 3 planned targets for chosen lane at 30, 60 and 90 meters.

Note that this is a function fitting process and hence we should ensure that a given input is not mapped to two different outputs; otherwise we get problems in expressing it as ``y=y(x)``. One ensures this by using a local coordinate system at the last point from the previous path. In total this anchor point preparation is done in lines 191--243.

After one has fitted the spline we build up the complete full prediction path. This is done in lines 256--285, and is effectively the logic of the Q&A.


