---
layout: default
title: Coordinate frames
---

# Coordinate frames

For general information on ROS coordinate frame conventions, see [REP 105](https://www.ros.org/reps/rep-0105.html).

## `base_link`

This is the origin of the robot. All sensors and mechanisms rigidly mounted to the robot can be described as static transforms relative to `base_link`.
We define `base_link` as the front of the robot, laterally centered, on the ground.

![Base link, top](/assets/images/warthog-top-bl.png)

![Base link, side](/assets/images/warthog-side-bl.png)

<small>Above: The base link frame origin (red).</small>

## `map`

The origin of this frame depends on the current map. The map's origin should be described in its `info.yaml` file.
