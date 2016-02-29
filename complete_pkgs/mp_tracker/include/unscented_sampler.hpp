#ifndef UNSCENTED_SAMPLER
#define UNSCENTED_SAMPLER
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <cmath>

namespace icl {
using namespace Eigen;
using namespace std;

#ifdef EIGENDECOMP
#undef EIGENDECOMP
#endif
//#define EIGENDECOMP
//#define DEBUG
template<int dim=Eigen::Dynamic, typename scalar=double>
struct unscented_sampler {
  // Spherical simplex
  typedef Eigen::Matrix<scalar, dim, 1> mean_type;
  typedef Eigen::Matrix<scalar, dim, dim> cov_type;
  typedef pair<size_t, size_t> sigma_hashmap_key;
  typedef std::map<sigma_hashmap_key, Eigen::VectorXd> sigma_hashmap;
  typedef typename sigma_hashmap::iterator sigma_hashmap_iterator;
  typedef typename sigma_hashmap::const_iterator sigma_hashmap_const_iterator;
  sigma_hashmap sigma_points_simplex;

  template<typename V>
  void get_points(const mean_type &mean_, const cov_type &cov_, scalar w0_, scalar alpha_, V &points, vector<scalar> &weights) {
#ifdef DEBUG
std::cout << "taking sigma points for: " << mean_.transpose() << " with cov: " << cov_ << std::endl;
#endif
    size_t dim_point = mean_.rows();
    size_t num_sigma = dim_point + 1;
    points.resize(num_sigma+1);
    weights.resize(num_sigma+1);
    // Fill the sigma weights
    scalar w1 = (1.0 - w0_) / (scalar)num_sigma;
    weights[0] = w0_;
    fill(weights.begin()+1, weights.end(), w1);
#ifdef EIGENDECOMP
    SelfAdjointEigenSolver<cov_type> solver(cov_);
    cov_type eigvals = solver.eigenvalues().array().sqrt().matrix().asDiagonal();
    cov_type eigvect = solver.eigenvectors();
    cov_type sqS = eigvect * eigvals;
#else
    cov_type sqS = cov_.llt().matrixL();
#endif
    for(size_t i = 0; i < num_sigma + 1; i++) {
      mean_type sigma;
      calc_sigma_point_simplex(i, dim_point, weights, sigma);
      points[i] = mean_ + sqS*sigma;
#ifdef DEBUG
std::cout << i << "-th sigma point: " << sigma.transpose() << " rotated with: " << sqS << std::endl;
#endif
    }
  }

  template<typename D>
  void calc_sigma_point_simplex(const size_t &i, const size_t &j, const vector<scalar> &weights, Eigen::MatrixBase<D> &output) {
    sigma_hashmap_const_iterator elem = sigma_points_simplex.find(make_pair(i, j));
    if(elem != sigma_points_simplex.end()) {
      output = elem->second;
      return;
    }
    output = Eigen::VectorXd::Zero(j);
    if(j == 1) {
      if(i == 0) {
        output(0) = 0.0;
      } else if(i == 1) {
        output(0) = -1.0 / sqrt(2.0 * weights[1]);
      } else if(i == 2) {
        output(0) = 1.0 / sqrt(2.0 * weights[1]);
      }
    } else {
      Eigen::VectorXd part = Eigen::VectorXd::Zero(j-1);
      if(i == 0) {
        calc_sigma_point_simplex(0, j-1, weights, part);
        output.block(0,0,j-1,1) = part;
        output[j-1] = 0.0;
      } else if (i <= j) {
        calc_sigma_point_simplex(i, j-1, weights, part);
        output.block(0,0,j-1,1) = part;
        output[j-1] = - 1.0 / sqrt(j*(j+1)*weights[j]);
      } else {
        output.block(0,0,j-1,1) = Eigen::VectorXd::Zero(j-1);
        output[j-1] = j / sqrt(j*(j+1)*weights[j]);
      }
    }
    sigma_points_simplex[make_pair(i, j)] = output;
  }
};

}
#endif
