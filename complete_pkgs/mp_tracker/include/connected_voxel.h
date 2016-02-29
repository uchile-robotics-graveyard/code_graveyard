#ifndef _CONNECTED_VOXEL_GRID_H
#define _CONNECTED_VOXEL_GRID_H
#include <boost/dynamic_bitset.hpp>

struct ConnectedVoxelGrid {
  // Structures to hold the data
  boost::dynamic_bitset<> grid;
  boost::dynamic_bitset<> connections;
  const size_t size_x, size_y, size_z, num_connection;

  // Possible connections
  enum connection {
    UP=0, DOWN, FRONT, BACK, RIGTH, LEFT;
  };

  // Interface
  ConnectedVoxelGrid(size_t size_x_, size_t size_y_, size_t size_z_);
  void mark(size_t x, size_t y, size_t z);
  void connect(size_t x, size_t y, size_t z, connection c);
  void clear(size_t x, size_t y, size_t z);
  const bool is_marked(size_t x, size_t y, size_t z) const;
};

#endif//_CONNECTED_VOXEL_GRID_H
