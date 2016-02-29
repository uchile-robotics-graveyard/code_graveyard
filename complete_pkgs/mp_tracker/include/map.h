#ifndef _SOCIAL_MAP
#define _SOCIAL_MAP
#include <vector>
#include <queue>
#include <cmath>
#include <stdint.h>
#include <iostream>

namespace icl {

struct cell {
  size_t i,j;
  size_t source_i, source_j;
};

struct map {
  double origin_x, origin_y;
  size_t size_x, size_y;
  double resolution;
  std::vector<int8_t> map;
  std::vector<double> obs_dist;
  bool initialised;

  // Convert from world coords to map coords
  inline int get_cell_x(double real_x) {return std::floor((real_x - this->origin_x) / this->resolution + 0.5) + this->size_x / 2;}
  inline int get_cell_y(double real_y) {return std::floor((real_y - this->origin_y) / this->resolution + 0.5) + this->size_y / 2;}

  // Convert from map index to world coords
  inline double get_real_x(size_t cell_x) {return this->origin_x + (cell_x - this->size_x / 2) * this->resolution;}
  inline double get_real_y(size_t cell_y) {return this->origin_y + (cell_y - this->size_y / 2) * this->resolution;}

  inline bool is_valid(size_t cell_x, size_t cell_y) {return (cell_x < this->size_x) && (cell_y < this->size_y);}
  inline int8_t at(size_t cell_x, size_t cell_y) {return map.at(cell_x + cell_y * this->size_x);}
  inline double dist_at(double real_x, double real_y) {
    int indx = get_cell_x(real_x) + get_cell_y(real_y) * this->size_x;
    if(indx < 0 || indx >= (int)obs_dist.size())
      return 0.0;
    return obs_dist.at(indx);
  }

  double calc_range(double ox, double oy, double oa, double max_range);

  void calc_obstacle_distances();
  void check_and_queue(cell &, std::queue<cell>&, const double &);
};
}
#endif
