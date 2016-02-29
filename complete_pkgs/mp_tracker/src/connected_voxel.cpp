#include "connected_voxel.h"

// Useful definitions
#define indx_grid(x,y,z) x*(size_z*size_y) + y*size_z + z
#define indx_connection(x,y,z,c) x*(size_z*size_y*num_connection) \
                                 + y*(size_z*num_connection)\
                                 + z*num_connection + (size_t)c

ConnectedVoxelGrid::ConnectedVoxelGrid(size_t size_x_, size_t size_y_, size_t size_z_):
  grid(size_x_*size_y_*size_z_),
  connections(size_x_*size_y_*size_z_*6),
  size_x(size_x_), size_y(size_y_), size_z(size_z_), num_connection(6) {}

void ConnectedVoxelGrid::mark(size_t x, size_t y, size_t z) {
  grid[indx_grid(x,y,z)] = true;
}

void ConnectedVoxelGrid::connect(size_t x, size_t y, size_t z, connection c) {
  if(c == UP && z == size_z-1)
    return;
  if(c == DOWN && z == 0)
    return;
  if(c == FRONT && y == size_y-1)
    return;
  if(c == BACK && y == 0)
    return;
  if(c == RIGTH && x == size_x-1)
    return;
  if(c == LEFT && x == 0)
    return;
  connections[indx_connection(x,y,z,c)] = true;
}

void ConnectedVoxelGrid::clear(size_t x, size_t y, size_t z) {
  connections[indx_grid(x,y,z)] = false;
  if(connections[indx_connection(x,y,z,UP)]) {
    connections[indx_connection(x,y,z+1,DOWN)] = false;
    clear(x,y,z+1);
  }
  if(connections[indx_connection(x,y,z,DOWN)]) {
    connections[indx_connection(x,y,z-1,UP)] = false;
    clear(x,y,z-1);
  }
  if(connections[indx_connection(x,y,z,FRONT)]) {
    connections[indx_connection(x,y+1,z,BACK)] = false;
    clear(x,y+1,z);
  }
  if(connections[indx_connection(x,y,z,BACK)]) {
    connections[indx_connection(x,y+1,z,FRONT)] = false;
    clear(x,y-1,z);
  }
}

const bool ConnectedVoxelGrid::is_marked(size_t x, size_t y, size_t z) const {
  return grid[indx_grid(x,y,z)];
}

