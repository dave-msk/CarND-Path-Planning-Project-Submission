Path Planning Project Write-up Report
=====================================

This report covers the methodology of the path planning project solution in this submission. The simulation video recorded on my machine is uploaded [here](https://www.youtube.com/watch?v=znvG6yde32M).

Abbreviation
------------

- `M`: Main car
- `FL`, `FF`, `LC`: Decision states of M - "Follow Lane", "Follow Front" and "Lane Change" respectively

Solution Walkthrough
====================

Overview
--------

The path generation process can be summarized into the following high-level pseudo-code:

```
1. Get the vehicles at the closest front and back of M, for each lane.
2. Generate path `s_free` that accelerates freely to the target speed.
3. If following `s_free` does not exceed a boundary defined by the front vehicle, then follow `s_free`.
4. Else:
5.     If left lane change is possible, generate and follow path for left lane change.
6.     Else if right lane change is possible, generate and follow path for right lane change.
7.     Else: 
8.         Follow the vehicle at the front.
```

In the following, I will first focus on the descriptions of each stage described above, then I will talk about how the Frenet to Cartesian coordinates conversion is done, and finally potential problems of my solution.

Background Settings
-------------------

Before going through the logic, it would be beneficial to know about the background setting of the solution.

### Code Structure

The code is devided into three main components.

`utils.h` and `utils.cpp` contain helper functions that does not need state tracking, such as performing differentiation given a list of polynomial coefficients (lines 93-99), polynomial function evaluation (lines 101-106), solve for quintic or quartic polynomial given boundary points (lines 19-40, 42-60), etc. 

`constants.h` contains constants for global configurations, such as target speed, limits (speed, acceleration, jerk), time search configurations, etc.

`ptg.h` and `ptg.cpp` contain the class `PTG` which is responsible for state-tracking and path generation.

### PTG Class

The PTG class keeps track of `M`'s state, namely location, speed, and acceleration in the s-dimension, and its location in the d-dimension as well. To archive a smooth transition from the original and the newly generated path, the calculation of the new path is based on the state of `M` at a certain amount of time ahead rather than the current one. Therefore, the physical state, `(s, sd, sdd, d)`, needed is not the current one provided by the simulator, which gives the necessity for tracking.

Other than the physical state, PTG also keeps track of the decision state of `M`. There are three possible values:
- `FL`: Follow lane, and accelerate to the target speed
- `FF`: See if lane change is possible. If not, follow front vehicle, and keep distance from it
- `LC`: Perform lane change

---

Get Closest Vehicles
--------------------

Most of the time only the vehicles that are located directly in front of and behind `M` are needed for decision making. Therefore, given the sensor fusion data, I first extracted the indices of vehicles that are closest, in terms of s-coordinate, to `M` for each of the three lanes (line 94, 197-224 in `ptg.cpp`), for further processing.

Freely Accelerating S-Path Generation
-------------------------------------

The code then generates a curve that allows `M` to accelerate to the target speed in the s-dimension (line 96 in `ptg.cpp`). In the curve generation (lines 62-78 in `utils.cpp`), the starting (`{start_s, start_sd, start_sdd}`) and end configuration (`{TARGET_SPEED, 0}`) are given, the code tries to increase the time parameter (by a pre-defined search resolution, line 18 in `constants.h`) and return the first feasible (i.e. with speed, acceleration and jerk under some pre-defined limit, see lines 10-12 in `constants.h`) curve. This ensures that the returned curve suggests a path that does not violate the limits while taking the least amount of time (among the search values) to achieve the target speed.

This curve, `s_free`, is the s-dimension curve that will be used in both "free" lane-following and lane changing.

"Free" Lane Following
---------------------

After generating `s_free`, the code first checks whether it is possible, or safe, for `M` to follow this curve directly on the current lane if `M` is in the `FL` mode. A prediction `pred_s` (line 99 in `ptg.cpp`), which is the s-coordinate `M` would be at 1.5s (line 15 in `constants.h`) ahead of time, is first computed. If there is no vehicle at the front, or `pred_s` is behind the current position of the front vehicle, then `M` is considered safe to follow `s_free` (as there is a 1.5s time buffer for `M` even if the front vehicle does not move).

Otherwise, `M` will switch into the `FF` mode.

Lane Change Check
-----------------

In the `FF` mode, the code will see if it is possible to perform lane change.

The left lane goes first (lines 113-139 in `ptg.cpp`). The code first determines whether there is a lane on the left for it to change to (line 113). Then it check where there is any vehicle on it. If there is none, then it is free to perform left lane change (lines 115-119). If there are vehicles, `M` can perform lane change if there is a 1.5s time buffer from the front vehicle (lines 124-125), and 1s time buffer from the back vehicle on the left lane (line 133).

Otherwise, it is not considered safe to change to left lane. The exactly same procedure is perform to check whether a right lane change is possible on lines 142-168 in `ptg.cpp`.

The code for lane change path generation is at lines 259-291 in `ptg.cpp`. The lane change curve comprises the `s_free` curve from above, and a quintic (for jerk minimization purpose) `d_curve` computed using `(start_d, 0, 0)` as starting and `(target_d, 0, 0)` for ending configuration, with a time interval of 2.0s (line 262). The whole lane change action will be performed without path re-computation as done in `FL` and `FF` state. Therefore, the state will be switch into `LC` to guaranteed that path re-computation is done before the lane change is completed (lines 78-83 in `ptg.cpp`).

Follow Front
------------

If no lane change is possible, `M` will follow the front vehicle. To maintain a safe distance from the front vehicle, I have included three distance zones for the `M` to stay in different speeds (lines 177-179 in `ptg.cpp`).


|distance (m) | percentage |
|:-----------:|:----------:|
| `ds` < 10 |    60%    |
| 10 <= `ds` < 15 | 80% |
| 15 <= `ds` < 20 | 95% |
| `ds` >= 20 | 100% |

The above table illustrates the relations between `ds` (the distance between `M` and the front vehicle) and the percentage of the speed of the front vehicle for `M` to keep. For instance, if `ds` is between 10 to 15 meters, then `M` will keep its speed to be 80% of the speed of the front vehicle.

If for some reason the front vehicle is not blocking anymore (maybe due to lane change), PTG will resume the decision state to `FL` and allow "free" lane following again.

The s-curve for following is generated by the same method (`forwardSSearch`) as `s_free` is generated, with the target speed changed to be related to the speed of front vehicle.

Frenet to Cartesian Converion
-----------------------------

The given map data consists of information of 181 check points. The most naive way to convert from Frenet to Cartesian coordinate system is by linear approximation. However, linear approximation introduces sharp turns at each check point, which is not desired. To "reconstruct" the smooth curve from the check points, I have used a cubic spline to join every check points, with `x`, `y`, `dx`, and `dy` parametrized by `s` (lines 19-47 in `ptg.cpp`).

When we need to convert `(s,d)` to `(x,y)`, the base point `(x0,y0)` corresponding to `(s,0)`, and the d-unit vector `(dx(s), dy(s))` at such s are computed using spline interpolation. Then, the resulting `(x,y)` is given by `(x0, y0) + d*(dx(s), dy(s))`.

The spline library used is provided by [http://kluge.in-chemnitz.de/opensource/spline/](http://kluge.in-chemnitz.de/opensource/spline/).

---

Potential Problems
==================

One of the greatest problem of this solution is that the path re-computation is disable when lane change is in execution. This provides no protection against any expected scenarios during lane changes. One possible situation is that the front vehicle is going to change to the same lane `M` is targeting. This will likely cause a collision.

Another problem is that if a vehicle changes lane so that it becomes directly ahead of `M`, `M` will not decelerate until that vehicle actually enters our current lane, which would probably be too late to avoid collision.