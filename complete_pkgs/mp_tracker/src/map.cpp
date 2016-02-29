/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Range routines
 * Author: Andrew Howard
 * Date: 18 Jan 2003
 * CVS: $Id: map_range.c 1347 2003-05-05 06:24:33Z inspectorg $
**************************************************************************/

#include <cmath>
#include <limits>
#include "map.h"
#include <iostream>

// Extract a single range reading from the map.  Unknown cells and/or
// out-of-bound cells are treated as occupied, which makes it easy to
// use Stage bitmap files.
double icl::map::calc_range(double ox, double oy, double oa, double max_range)
{
  if(!initialised)
    return max_range;

  // Bresenham raytracing
  int x0,x1,y0,y1;
  int x,y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = get_cell_x(ox);
  y0 = get_cell_y(oy);
  
  x1 = get_cell_x(ox + max_range * cos(oa));
  y1 = get_cell_y(oy + max_range * sin(oa));

  if(std::abs(y1-y0) > std::abs(x1-x0))
    steep = 1;
  else
    steep = 0;

  if(steep)
  {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = std::abs(x1-x0);
  deltay = std::abs(y1-y0);
  error = 0;
  deltaerr = deltay;

  x = x0;
  y = y0;

  if(x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if(y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if(steep)
  {
    if(!is_valid(y,x) || at(y,x) > 0)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * this->resolution;
  }
  else
  {
    if(!is_valid(x,y) || at(x,y) > 0)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * this->resolution;
  }

  while(x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;
    if(2*error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if(steep)
    {
      if(!is_valid(y,x) || at(y,x) > 0)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * this->resolution;
    }
    else
    {
      if(!is_valid(x,y) || at(x,y) > 0)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * this->resolution;
    }
  }
  return max_range;
}

#define OCC_VAL 80
#define NEAR 2.0
void icl::map::check_and_queue(icl::cell &c, std::queue<cell>& queue, const double &non_visited_dist) {
  double di = (int)c.source_i - (int)c.i;
  double dj = (int)c.source_j - (int)c.j;
  double dist = std::sqrt(di*di + dj*dj) * resolution;
  if(dist < obs_dist.at(c.i + c.j * size_x)) {
    obs_dist.at(c.i + c.j * size_x) = dist;
    queue.push(c);
  }
}

void icl::map::calc_obstacle_distances(){
  if(!initialised)
    return;
  // Allocate space for the distances
  obs_dist.resize(map.size());
  const double max_dist = std::sqrt(size_x*size_x + size_y*size_y) * resolution;
  std::fill(obs_dist.begin(), obs_dist.end(), max_dist);
  std::queue<icl::cell> points_queue;
  for(size_t i = 0; i < size_x; i++) {
    for(size_t j = 0; j < size_y; j++) {
      if(at(i,j) >= OCC_VAL) {
        obs_dist.at(i + j * size_x) = 0.0;
        icl::cell c;
        c.i = i;
        c.j = j;
        c.source_i = i;
        c.source_j = j;
        points_queue.push(c);
      }
    }
  }
  while(!points_queue.empty()) {
    const icl::cell &c = points_queue.front();
    points_queue.pop();
    if(c.i > 0) {
      icl::cell cn;
      cn.i = c.i-1;
      cn.j = c.j;
      cn.source_i = c.source_i;
      cn.source_j = c.source_j;
      check_and_queue(cn, points_queue, max_dist);
    }
    if(c.j > 0) {
      icl::cell cn;
      cn.i = c.i;
      cn.j = c.j-1;
      cn.source_i = c.source_i;
      cn.source_j = c.source_j;
      check_and_queue(cn, points_queue, max_dist);
    }
    if(c.i < size_x -1) {
      icl::cell cn;
      cn.i = c.i+1;
      cn.j = c.j;
      cn.source_i = c.source_i;
      cn.source_j = c.source_j;
      check_and_queue(cn, points_queue, max_dist);
    }
    if(c.j < size_y -1) {
      icl::cell cn;
      cn.i = c.i;
      cn.j = c.j+1;
      cn.source_i = c.source_i;
      cn.source_j = c.source_j;
      check_and_queue(cn, points_queue, max_dist);
    }
  }
/*
std::cout << "map=[" << std::endl;
  for(size_t i = 0; i < size_x; i++) {
std::cout << "[";
    for(size_t j = 0; j < size_y; j++) {
std::cout << "[" <<  (int)at(i,j) << ", " << obs_dist.at(i + j * size_x) << "], ";
    }
std::cout << "]," << std::endl;
  }
std::cout << "]" << std::endl;
*/
}
