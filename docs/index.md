---
title: Home
layout: home
nav_order: 1
last_modified_date: 2024-08-30 0:00:00 +0000
---
# Steward: Your <br/> Reforestation Companion
{: .fs-10 }

# Introduction

Steward is a robot that plants trees on marginal pastureland. This documentation describes Steward's software components. We separate Steward's software into three suites:

- **Steward OS**, the core code, which is organized as a ROS2 workspace
- [**EcoSim**](https://arbor-robotics.github.io/ecosim), a simulation tool built with the Unity game engine
- **Canopy**, a web-based user interface

This documentation is specifically for Steward OS.

# Overview of Steward OS

Steward OS is a collection of ROS2 nodes, where each node is a program written to perform a specific task. Nodes are organized into the following groups:

- **Interfaces**: Interfaces to sensors, the simulator, Canopy, etc.
- **Perception**: Classifiers, detectors, and state estimators
- **Planning**: Path planners, costmap generators, and behavior managers
- **Control**: Nodes to turn high-level goals into low-level actuation commands
- **Actuation**: Nodes for moving the robot

[![Architecture flowchart](/assets/images/sw-architecture-24-08-30.svg)](/assets/images/sw-architecture-24-08-30.svg)
