---
title: Home
layout: home
---

# About Stu

The **Digital Steward**, or Stu, is a robotic tree planting companion built by grad students in Carnegie Mellon's Robotics Institute.

Stu's custom code is entirely open-source. That means that it's free to use and completely transparent. Our code documentation lives here. It's for nerds, so if you're not a nerd, may we suggest reading our main project site instead?

# Architecture

<div style="width: 640px; height: 480px; margin: 10px; position: relative;"><iframe allowfullscreen frameborder="0" style="width:640px; height:480px" src="https://lucid.app/documents/embedded/b676de26-bdc8-4fa0-baab-b34b08dfedc9" id="zyVbuUjpbA7-"></iframe></div>

# Dependencies

- Ubuntu 22.04 LTS
- ROS2 Humble (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [pre-commit](https://pre-commit.com/#installation)
  - `$ pip install pre-commit && pre-commit --version`

# Installation

```bash
$ git clone https://github.com/arbor-robotics/steward.git
$ cd steward
$ colcon build
$ . install/setup.bash # Must run in every new terminal.

```
