---
title: Health
layout: default
---

## How Steward OS monitors health

### 1. Components self-report their health

Every essential component, and most non-essential components regularly publish [DiagnosticStatus](https://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticStatus.html) messages. These statuses include the name of the component, its current health level, and a brief description.

### 2. The Health Monitor monitors all statuses

The Health Monitor, which is itself a ROS2 node, combines all statuses into a complete list. It inspects each status in the list with the help of a predefined list of [checks](#checks). These are like unit tests, but they happen constantly during operation.

### 3. The Health Monitor makes a diagnosis and takes action

The Monitor has two jobs. First, it **provides clear information to the user** about Steward's complete health. Think of this as the diagnostic panel on your car: Your car may display may warn you about low tire pressure or a faulty engine, but it doesn't actually prevent you from trying to drive. It's up to you, the user, to either operate the car or not.

Second, the Monitor **disables operation** if necessary. This makes it *different* from your car's diagnostics. In serious failure conditions, the Health Monitor will actually block Steward from entering `TELEOP` or `AUTO` modes. See [system-wide status levels](#system-wide-status-levels)

{: .example-title}
> Example: Disabling `TELEOP` and `AUTO`
>
> If the Monitor detects a motor failure, then it will disable both `TELEOP` and `AUTO`, as it is not safe to move the Warthog either autonomously or manually.

{: .example-title}
> Example: Disabling `AUTO` but not `TELEOP`
>
> If instead the Monitor detects a Map Manager error, then it will disable `AUTO` but not `TELEOP`, as it is still safe for a human to control the robot.

## System-wide status levels

### 🟢 Healthy

All systems are operating normally.

### 🟡 Warn

Steward can operate autonomously, but some features may be disabled or limited. For example, ground speed may be limited.

### 🟠 Teleop Only

Steward's autonomy is unavailable due to a significant fault in the planning or perception systems, in a sensor interface, or some other critical component.

### 🔴 Out of Service

Steward OS is completely unavailable.


## Checks

### Interfaces

#### Is Steward OS connected to the Warthog or EcoSim?

| **Code**     | `BRIDGE_FAILURE`  |
| **Inspects** |  `bridge/steward` or `bridge/warthog`  |
| **Period** |  0.2s (5 Hz)  |
| **Triggers** | 🔴 Out of Service  |
| **Message**  | Steward is unable to connect to the Warthog or the simulator and cannot operate.  |
