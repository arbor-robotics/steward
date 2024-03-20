---
layout: default
title: Forest Planner
---

# Forest Planner

#### Subscribes to:

- `/map/height` (`nav_msgs/OccupancyGrid`)
- `/map/planting_bounds` (`nav_msgs/OccupancyGrid`)

#### Publishes:

- `/planning/forest_plan` (`nav_msgs/OccupancyGrid`)

#### ROS Params:

- `grid_resolution` (float): The size of the Forest Plan's cells. This should be compatible with both the height map and planting bounds map such that $$res_{forest plan} = (res_{input})^n, n \in \mathbb{Z}$$

## Planting bounds

The Forest Planner subscribes to `/map/planting_bounds`, which is of type `nav_msgs/OccupancyGrid`. This grid should have the same physical bounds as the height map (`/map/height`), and its cell resolution should be compatible such that one cell in the planting bounds grid corresponds to one, four, 16, etc cells in the height map.

Cells in the planting bound grid may take the following values:

| Value (int) | Description                      |
| ----------- | -------------------------------- |
| 100         | Keep out                         |
| 0           | Plant here                       |
| 50          | May enter, but do not plant here |

![Left to right, down: Planting bounds map, forest plan, height map, satellite imagery](/assets/images/forest-planner-subplots.png)
<small>Forest plan data for Flagstaff Hill (Schenley Park). In the planting bounds, yellow="enter but don't plant" and teal="plant here." The Forest Plan is the node's output; everything else is an input.</small>

## Format of forest plans

Forest plans should be grids, where each cell takes on a value corresponding to its species.

Forest plans can be divided into tiles, such as one tile per acre, to reduce memory and computational cost.

A good resolution for a forest plan is 0.2 meters. That is, each cell in the forest plan has dimensions of 0.2m x 0.2m. Each acre of land could be represented at this resolution as a 318x318 px image (318.07468, more precisely). That's a tiny image!

### Why grids? Why not graphs/networks?

Grids (matrices) be quickly operated on using any linear algebra library, such as numpy. Matrices can also be treated as images, which can be easily visualized without downloading specialized tools. They can be processed through CNNs. GPUs are optimized to process matrices, not graphs. Matrices can be converted to graph form if needed.

## Considerations

### Planting density

Planting density, which we will measure in trees per acre, varies widely based on the land owner's priorities, the local climate, tree species, even seedling cost. [[1][1]] According to the city of Portland, forests have 100-200 trees per acre. [[2][2]] The Georgia Forestry Commission recommends about 550 trees per acre for a wildlife-hardwood tradeoff. [[3][3]] The Miyawaki Method, popular with naturalists, recommends about 12,000 trees per acre, but this can be as high as 28,000 trees in mangrove forests. [[4][4]] So we’re dealing with a range of 100 tpa to 28,000 tpa. Since we are not targetting mangrove reforestation, we will support a maximum planting density of **12,000 tpa**.

<details markdown="block">
<summary> Images of forests at various planting densities</summary>

![308 tpa forest](/assets/images/forest-planner-308tpa.png)

<small>A 308 tpa forest, according to the [Washington Farm Forestry Assosciation](https://www.wafarmforestry.com/sites/default/files/pdfs/Education/SFLO101/5-Planting.pdf).</small>

![550 tpa forest](/assets/images/forest-planner-550tpa.png)

<small>A 550 tpa forest, according to the [Washington Farm Forestry Assosciation](https://www.wafarmforestry.com/sites/default/files/pdfs/Education/SFLO101/5-Planting.pdf).</small>

![A Miyawaki forest, planted at approx. 12,000 tpa](/assets/images/forest-planner-miyawaki.jpg)

<small>A Miyawaki forest, planted at approx. 12,000 tpa.</small>

![A mangrove forest, which can reach 28,000 tpa. Credit: [JSTOR Daily](https://daily.jstor.org/the-magnificent-maligned-mangrove/)](/assets/images/forest-planner-mangrove.webp)

<small>A mangrove forest, which can reach 28,000 tpa.</small>

</details>

{: .extra }
When foresters calculate planting density, they also consider _basal area_. In the United States, basal area is the cross-sectional area of a tree's trunk at breast height (4.5 feet, 1.5 m) in square feet. This includes the bark. Basal area can also be expressed in acres, which is the sum of the basal area of each tree within a given acre. See this video by the [Alabama Extension](https://www.youtube.com/watch?v=8EmbJe4tVPQ).

### Species

Long-term, we'd like to support hundreds, perhaps thousands, of tree species. For now, though, we're targeting the specific climate of Pittsburgh, where [Appalachian oak forest](https://envirothonpa.org/documents/2-1_ForestTypesPA_001.pdf) is dominant. We will therefore start with seven supported species:

| Scientific name           | Common name         | Forest layer |
| ------------------------- | ------------------- | ------------ |
| _Amelanchier arborea_     | common serviceberry | Understory   |
| _Nyssa sylvatica_         | black tupelo        | Understory   |
| _Acer rubrum_             | red maple           | Canopy       |
| _Carya ovata_             | shagbark hickory    | Canopy       |
| _Quercus montana_         | chestnut oak        | Canopy       |
| _Quercus rubra_           | northern red oak    | Canopy       |
| _Liriodendron tulipifera_ | Tulip tree          | Emergent     |

[1]: https://thundersaidenergy.com/downloads/reforestation-what-planting-density-for-seedlings/
[2]: https://www.oregonmetro.gov/news/power-trees
[3]: https://gatrees.org/wp-content/uploads/2020/03/HowManyTreesShouldIPlantJuly2011.pdf
[4]: https://thewaterchannel.tv/thewaterblog/the-key-steps-of-the-miyawaki-method-to-plant-a-mini-forest/
