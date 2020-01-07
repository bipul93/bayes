# bayes
Grid Localization using Bayes Filter

Grid Localization is a variant of discrete Bayes Localization. In this method, the map is an occupancy grid. At each time step, the algorithm finds out the probabilities of the robot’s presence at every grid cell.T he grid cells with maximum probabilities at each step, characterize the robot’s trajectory. Grid Localization using Bayes filter runs in two iterative steps that utilize the movementdata (prediction step) and observation data (update step).

Visualization in RViz
