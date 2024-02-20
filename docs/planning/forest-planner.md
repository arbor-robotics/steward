---
layout: default
title: Forest Planner
---

# Forest Planner

#### Subscribes to:

- `/map/height` (`nav_msgs/OccupancyGrid`)
- `/map/planting_bounds` (`nav_msgs/OccupancyGrid`)

#### Publishes:

- `/planning/forest_plan` (`steward_msgs/ForestPlan`)

#### ROS Params:

None so far.

## Planting bounds
The Forest Planner subscribes to `/map/planting_bounds`, which is of type `nav_msgs/OccupancyGrid`. This grid should have the same physical bounds as the height map (`/map/height`), and its cell resolution should be compatible such that one cell in the planting bounds grid corresponds to one, four, 16, etc cells in the height map.

Cells in the planting bound grid may take the following values:

| Value (int) | Description                                          |
| ----- | ---------------------------------------------------- |
| 0     | Keep out                                             |
| 1     | Plant here                                           |
| 2     | May enter, but do not plant here                     |

![Example of a planting bounds grid](/assets/images/planting-bounds-grid-example.png)
<small>Example of a planting bounds grid, where 0=black, 1=white, 2=gray.</small>


## Considerations

The number of seedlings that should be planted per acre of land depends on the priorities of the landowner. Higher planting densities mean more timber production but less favorable habitat for wildlife. According to the [Georgia Foresty Commission](https://gatrees.org/wp-content/uploads/2020/03/HowManyTreesShouldIPlantJuly2011.pdf), "A compromise initial spacing for timber and wildlife is between **500 and 600 seedlings per acre.**" Of course, this metric really depends on the species being planted, climatic conditions of the planting sight, and so on. However, we will use the 500-600 spa measurement as a design target. We will design our algorithms to support a maximum planting density of 800 spa.

{: .extra }
When foresters calculate planting density, they also consider _basal area_. In the United States, basal area is the cross-sectional area of a tree's trunk at breast height (4.5 feet, 1.5 m) in square feet. This includes the bark. Basal area can also be expressed in acres, which is the sum of the basal area of each tree within a given acre. See this video by the [Alabama Extension](https://www.youtube.com/watch?v=8EmbJe4tVPQ).
