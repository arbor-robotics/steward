---
layout: default
title: Waypoint following
---

# Waypoint following

Action: `/follow_waypoints`

Server: `/waypoint_follower`

Definition:

```
nav2_msgs/action/FollowWaypoints
-------
#goal definition
geometry_msgs/PoseStamped[] poses
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
---
#result definition
int32[] missed_waypoints
---
#feedback definition
uint32 current_waypoint
```
