// Optimal rectangular assignment (Kuhn-Munkres / Hungarian, O(n^2 m) with potentials).
#pragma once

#include <cstddef>
#include <vector>

namespace helios_obstacle_detection
{

/// `cost` is row-major rows x cols. Entries >= `infeasible` are forbidden pairs.
/// Returns, for every row, the assigned column or -1. Minimises the total cost of the
/// feasible pairs; rows that can only be matched through forbidden pairs stay unassigned.
std::vector<int> solveAssignment(
  const std::vector<double> & cost, std::size_t rows, std::size_t cols, double infeasible);

}  // namespace helios_obstacle_detection
