#ifndef _MP_TRACKER_PERSON
#define _MP_TRACKER_PERSON
#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>

namespace icl {
// Number of bns of the histograms
#define NUM_BINS 20
struct person_state {
  typedef boost::shared_ptr<person_state> ptr;

  person_state() {}

  person_state(const person_state &s) {
    weight = s.weight;
    posvelheight = s.posvelheight;
    posvelheight_cov = s.posvelheight_cov;
  }
  double weight;
  // The basic data of the state
  Eigen::Matrix<double,5,1> posvelheight;
/*
  double fatness;
  Eigen::Matrix<double,NUM_BINS,1> histogram_head;
  Eigen::Matrix<double,NUM_BINS,1> histogram_body;
  Eigen::Matrix<double,NUM_BINS,1> histogram_legs;
*/
  // The uncertainty
  Eigen::Matrix<double,5,5> posvelheight_cov;
  // No uncertainty on the histogram?

  bool operator< (const person_state &ps) const {
    return this->weight < ps.weight;
  }

  friend std::ostream& operator << (std::ostream &out, const person_state &p) {
    return out << p.weight << std::endl << p.posvelheight.transpose() << std::endl << p.posvelheight_cov << std::endl;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline bool operator<(const person_state::ptr &p1, const person_state::ptr &p2) {
  return (*p1) < (*p2);
}

inline void printall(const std::vector<person_state::ptr> &states) {
  for(size_t i = 0; i < states.size(); i++) {
    std::cout << "person " << *states.at(i) << std::endl;
//    std::cout << *states.at(i);
  }
  std::cout << std::endl;
}

}
#endif
