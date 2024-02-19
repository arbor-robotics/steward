---
layout: default
title: Route Generator
---

# Route Generator

Computes an efficient route that connects all the planting points within a forest plan. This is a form of the [traveling salesman problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem#Exact_algorithms).

#### Subscribes to:

- `/planning/forest_plan` (`steward_msgs/ForestPlan`)

#### Publishes:

- `/planning/full_route` (`steward_msgs/Route`)
- `/planning/remaining_route` (`steward_msgs/Route`)

#### ROS Params:

- `use_fasttsp` (Bool): Use Fast-TCP instead of Ant Colony Optimization. Defaults to True.

## Overview

By default, we use [Fast-TSP](https://fast-tsp.readthedocs.io/en/latest/index.html), a stochastic local search (SRS) solver. It's fast and produces great results.

The Route Generator alternatively supports [Ant Colony Optimization](https://github.com/Akavall/AntColonyOptimization). However, this method is slower and the resultant route is not as optimal.

![Result of Fast-TSP](/assets/images/fast-tsp-550pts.png)
<small>_Result of Fast-TSP. The green dot is our starting and ending point. Note that the loop is not closed in the plot, but the start point and end point can be easily connected to achieve a loop. Further, the start/end point is arbitrary-- any point in the graph can be selected without affecting the path length._</small>

![Runtime results of Fast-TSP](/assets/images/fast-tsp-runtimes.png)
<small>_Runtime of Fast-TSP for various sapling counts. The far end of 10k saplings takes 9 seconds. Note that at a typical planting density of 550 saplings per acre, that's over 18 acres in nine seconds!_</small>
