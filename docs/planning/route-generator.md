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
