---
layout: default
title: Nav2
---

# Nav2

## 1. What are behavior trees?

_From Chapter 1 of ["Behavior Trees in Robotics and AI: An Introduction"](https://arxiv.org/abs/1709.00084)_ (2022)

### 1.1. Short history and motivation

A **behavior tree (BT)** represents the transitions between tasks in an autonomous agent. BTs are efficient ways of creating _complex, modular,_ and _reactive_ systems. They are an increasingly popular alternative to finite state machines (FSMs). The BT was first popularized in computer game development, but it's since been applied to robotics and AI.

### 1.2. What's wrong with FSMs?

Robots should be both _reactive_ and _modular_. By reactive, we mean that they should quickly and efficiently react to changes. By modular, we mean the degree to which a system's components can be separated and rearranged.

FSMs have the same drawback as GOTO statements used in early programming languages: they rely on _one-way control transfers_. For a system to be reactive, there needs to be many transitions between components, which adds complexity and harms modularity. If one component is removed, every transition to that component needs to be revised.

### 1.3. Classical formulation of BTs

A BT is a directed, rooted tree. Its internal nodes are called **control flow nodes** and its leaf nodes are called **execution nodes**.

Execution starts at the root node. The root generates **ticks** at a given frequency. A node is executed _iff_ it receives ticks. Any node that receives a tick immediately returns either **Running, Success**, or **Failure** to its parent.

Control flow nodes are divided into **Sequence, Fallback, Parallel** and **Decorator**, while execution nodes are divided into **Action** and **Condition**.

#### Sequence (→)

The Sequence node routes ticks to its children from left to right until it finds a child that returns either _Failure_ or _Running_. It then accordingly returns _Failure_ or _Running_ to its parent. It returns _Success_ _iff_ all children return _Success_. Note that when a Sequence node finds a child that returns Failure or Running, the Sequence node will not route ticks to the next child (if one exists).

#### Fallback (?)

The Fallback node routes ticks to its children from left to right until it finds a child that returns either _Success_ or _Running_. It then accordingly returns _Success_ or _Running_ to its parent. It returns _Failure_ _iff_ all children return _Failure_. Note that when a Sequence node finds a child that returns Success or Running, the Sequence node will not route ticks to the next child (if one exists).

#### Parallel (⇒)

The Parallel node routes ticks to _all_ children. It returns _Success_ if M children return Success, _Failure_ if $$N-M+1$$ children return _Failure_, and _Running_ otherwise, where N is the number of children and $$M\le N$$ is a user-defined threshold.

#### Actions

When an Action node receives a tick, it executes a command and returns either _Success_, _Failure_, or _Running_. These are symbolized by text blocks with square corners.

#### Conditions

When a Condition receives a tick, it checks a proposition and returns either _Success_ or _Failure_ (but never _Running_). These are symbolized by text blocks with rounded corners.

#### Decorators (♦)

A Decorator manipulates the return status of a single child according to some user-defined rule. The _invert_ decorator, for example, inverts the success/failure of the child. The _max-N-tries_ decorator only lets its child fail _N_ times, after which it always returns _Failure_ without ticking the child.

![Pac-Man BT example](/assets/images/pacman-bt.png)

<small>Above: An example BT for Pac-Man.</small>

## 2. The navigation servers

[[docs](https://navigation.ros.org/concepts/index.html#navigation-servers)]

Nav2 includes four action servers: the planner, behavior, smoother, and controller servers.

The planner, smoother, and controller servers can be changed to implement a distinct algorithm (e.g. control algorithms). These servers expose action interfaces to their tasks. When the BT ticks to the corresponding BT node, it will call the action server to process its task.

### 2.1. Planners

The **planner** computes a path to complete some objective. A path is sometimes called a route. In the context of Steward, a path describes a local, granular curve for the robot to follow, while a route is more global and less detailed.

### 2.2. Controllers

Controllers follow a path by generating feasible control efforts.

### 2.3. Behaviors

**Recovery behaviors**, or simply recoveries, are intended to autonomously handle unknown or failure conditions. Examples include backing up, spinning in place, or calling a human operator for help.

### 2.4. Smoothers

Smoothers refine a route. Separating smoothers from planners allow them to work more flexibly with a range of planners or controllers. In general, smoother receive a path and return an improved version.

### 2.5. Footprints

When calculating paths using a cost map, we set a robot's footprint either as a circle of radius `robot_radius` or as a polygon of points `footprint`. The footprint can be dynamically adjusted using the costmap's `~/footprint` topic.

## 3. Setting up Nav2

### 3.1. Transforms

Nav2 requires the following transforms, which all follow [REP 105](https://www.ros.org/reps/rep-0105.html):

- `map -> odom`
- `odom -> base_link`
- `base_link -> [sensor frame]`

As a reminder, the first transform is meant to be globally accurate yet not necessarily smooth, while the second transform is meant to be smooth yet not necessarily globally accurate. The second TF typically uses wheel encoders or other methods to dead reckon from the first transform. The first transform favors global accuracy while the second favors local accuracy. See REP 105 for more information.

While the first two transforms are dynamic, the third is nearly always static, since the sensors rarely change position relative to the robot's origin.

### 3.2. The URDF

This is already provided in `steward/config/steward.urdf.xacro`, which is based off of `warthog.urdf.xacro`.

### 3.3. Odometry

Nav2 requires both an `odom -> base_link` transform _and_ `nav_msgs/Odometry` messages. The latter supplies velocity data.
