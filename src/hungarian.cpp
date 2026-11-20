#include "helios_obstacle_detection/hungarian.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace helios_obstacle_detection
{

std::vector<int> solveAssignment(
  const std::vector<double> & cost, std::size_t rows, std::size_t cols, double infeasible)
{
  if (cost.size() != rows * cols) {
    throw std::invalid_argument("solveAssignment: cost size mismatch");
  }
  std::vector<int> result(rows, -1);
  if (rows == 0 || cols == 0) {
    return result;
  }
  // Work on an n x m problem with n <= m (transpose if needed). Forbidden entries get a big
  // finite penalty so the algorithm stays well defined; such pairs are dropped afterwards.
  const bool transposed = rows > cols;
  const std::size_t n = transposed ? cols : rows;
  const std::size_t m = transposed ? rows : cols;
  double max_feasible = 0.0;
  for (const double c : cost) {
    if (c < infeasible) {
      max_feasible = std::max(max_feasible, std::abs(c));
    }
  }
  const double big = (max_feasible + 1.0) * static_cast<double>(n + 1) * 4.0;
  auto at = [&](std::size_t i, std::size_t j) {   // 0-based in the (n x m) problem
      const double c = transposed ? cost[j * cols + i] : cost[i * cols + j];
      return c >= infeasible ? big : c;
    };

  constexpr double kInf = std::numeric_limits<double>::infinity();
  std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0), minv(m + 1);
  std::vector<std::size_t> p(m + 1, 0), way(m + 1, 0);
  std::vector<char> used(m + 1);
  for (std::size_t i = 1; i <= n; ++i) {
    p[0] = i;
    std::size_t j0 = 0;
    std::fill(minv.begin(), minv.end(), kInf);
    std::fill(used.begin(), used.end(), 0);
    do {
      used[j0] = 1;
      const std::size_t i0 = p[j0];
      double delta = kInf;
      std::size_t j1 = 0;
      for (std::size_t j = 1; j <= m; ++j) {
        if (!used[j]) {
          const double cur = at(i0 - 1, j - 1) - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      }
      for (std::size_t j = 0; j <= m; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const std::size_t j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  for (std::size_t j = 1; j <= m; ++j) {
    if (p[j] == 0) {
      continue;
    }
    const std::size_t i = p[j] - 1;  // row in (n x m)
    const std::size_t col = j - 1;
    const std::size_t r = transposed ? col : i;
    const std::size_t c = transposed ? i : col;
    if (cost[r * cols + c] < infeasible) {
      result[r] = static_cast<int>(c);
    }
  }
  return result;
}

}  // namespace helios_obstacle_detection
