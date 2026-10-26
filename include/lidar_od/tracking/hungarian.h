#pragma once

#include <vector>

namespace lidar_od {

// Minimum-cost assignment (Kuhn-Munkres with potentials, O(n^2 m)).
// cost is rows x cols, any shape. Returns for every row the assigned column,
// or -1 when there are more rows than columns and the row is left out.
std::vector<int> solveAssignment(const std::vector<std::vector<double>>& cost);

}  // namespace lidar_od
