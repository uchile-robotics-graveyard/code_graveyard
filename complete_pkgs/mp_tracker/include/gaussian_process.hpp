#ifndef _GAUSSIAN_PROCESSPH
#define _GAUSSIAN_PROCESSPH
#include <cmath>
#include <Eigen/Dense>
#include <boost/function.hpp>

namespace gp {
using namespace Eigen;
using namespace std;

struct squared_exp_kernel {
  double sigma_;
  squared_exp_kernel(double sigma = 1.0): sigma_(sigma) {}

  double operator()(const VectorXd &x1, const VectorXd &x2) const {
    VectorXd dx = x1-x2;
    return exp(-dx.squaredNorm() / sigma_);
  }
};


struct GaussianProcess {
  boost::function<double (const VectorXd &, const VectorXd &)> kernel_;
  double noise_;
  MatrixXd data_, L_;
  VectorXd alpha_; 
  size_t n_;

  GaussianProcess(const boost::function<double (const VectorXd &, const VectorXd &)> &kernel,
                  const double &noise):
    kernel_(kernel),
    noise_(noise),
    n_(0) {};

  void train(const MatrixXd &data, const VectorXd &response) {
    // Same number of data points and response points
    assert(data.cols() == response.rows());
    n_ = data.cols();
    data_ = MatrixXd(data.rows(), data.cols());

    // Compute matrix K
    MatrixXd K = MatrixXd::Zero(n_, n_);
    for(size_t i = 0; i < n_; i++) {
      for(size_t j = i; j < n_; j++) {
        const double k = kernel_(data.col(i), data.col(j));
        K(i, j) = k;
        K(j, i) = k;
        if(i == j)
          K(i, j) += noise_;
      }
      // Make a copy of the data
      data_.col(i) = data.col(i);
    }

    // Compute L
    L_ = K.llt().matrixL();
    alpha_ = L_.triangularView<Upper>().transpose().solve(L_.triangularView<Upper>().solve(response));
  }

  double operator()(const VectorXd &x) const {
    // Check for consistency
    assert(n_ > 0);
    assert(x.rows() == n_);

    // Vector ks
    VectorXd ks(n_);
    for(size_t j = 0; j < n_; j++) {
      ks(j) = kernel_(data_.col(j), x);
    }

    return ks.dot(alpha_);
  }
};
}
#endif
