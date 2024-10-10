---
title: Planning
layout: default
---

When designing movement plans for Steward, we use a framework built around **cost maps**, which are basically grayscale images that each represent something that the Steward should either avoid or be attracted to.

## Egocentric cost maps

![Dimensions of the egocentric cost map](/assets/images/costmap.png)
<small>*Above*: Dimensions of the egocentric cost map. Each grid cell here is actually 10x10 cells/pixels.</small>

Our cost maps have a resolution of 0.2 meters per pixel. That mean that there are 25 pixels per square meter.
