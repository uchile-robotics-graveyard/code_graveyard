#include <map>
#include <stdexcept>
#include "d_tracker.h"
#include <cstdio>

std::map<std::pair<double,double>, double> bhatta_cache;
typedef std::map<std::pair<double,double>, double>::iterator cache_iterator;

int icl::optimise_beta_params(const gsl_vector *x, void *params_, gsl_vector *f) {
  double *params = (double*)params_;
  const double weight_sum_a = params[0];
  const double weight_sum_b = params[1];
  
  const double a = gsl_vector_get(x, 0);
  const double b = gsl_vector_get(x, 1);

  gsl_sf_result psi_ab, psi_a, psi_b;
  int status;
  status = gsl_sf_psi_e(a+b, &psi_ab);
  if(status != GSL_SUCCESS)
    return status;
  status = gsl_sf_psi_e(a, &psi_a);
  if(status != GSL_SUCCESS)
    return status;
  status = gsl_sf_psi_e(b, &psi_b);
  if(status != GSL_SUCCESS)
    return status;

  const double part_a = psi_ab.val - psi_a.val - weight_sum_a;
  const double part_b = psi_ab.val - psi_b.val - weight_sum_b;

  gsl_vector_set(f, 0, part_a);
  gsl_vector_set(f, 1, part_b);

  return GSL_SUCCESS;
}

int icl::optimise_beta_params_df(const gsl_vector *x, void *params_, gsl_matrix *J) {
  const double a = gsl_vector_get(x, 0);
  const double b = gsl_vector_get(x, 1);

  gsl_sf_result psi_ab, psi_a, psi_b;
  int status;
  status = gsl_sf_psi_1_e(a+b, &psi_ab);
  if(status != GSL_SUCCESS)
    return status;
  status = gsl_sf_psi_1_e(a, &psi_a);
  if(status != GSL_SUCCESS)
    return status;
  status = gsl_sf_psi_1_e(b, &psi_b);
  if(status != GSL_SUCCESS)
    return status;

  gsl_matrix_set(J, 0, 0, psi_ab.val - psi_a.val);
  gsl_matrix_set(J, 0, 1, psi_ab.val);
  gsl_matrix_set(J, 1, 0, psi_ab.val);
  gsl_matrix_set(J, 1, 1, psi_ab.val - psi_b.val);

  return GSL_SUCCESS;
}

int icl::optimise_beta_params_fdf(const gsl_vector *x, void *params_, gsl_vector *f, gsl_matrix *J) {
  int status;
  status = optimise_beta_params(x, params_, f);
  if(status != GSL_SUCCESS)
    return status;
  status = optimise_beta_params_df(x, params_, J);
  if(status != GSL_SUCCESS)
    return status;
  return GSL_SUCCESS;
}

// Helper function
double kld(const double &a_1, const double &b_1, const double &a_2, const double &b_2) {
  gsl_sf_result lnbeta_2, lnbeta_1, psi_a1, psi_b1, psi_ab1;
  int status;
  status = gsl_sf_lnbeta_e(a_2, b_2, &lnbeta_2);
  if(status != GSL_SUCCESS) {
    char buf[200];
    sprintf(buf, "Problem calculating ln(beta(a_2=%f, b_2=%f))", a_2, b_2);
    throw std::runtime_error(std::string(buf));
  }
  status = gsl_sf_lnbeta_e(a_1, b_1, &lnbeta_1);
  if(status != GSL_SUCCESS) {
    char buf[200];
    sprintf(buf, "Problem calculating ln(beta(a_1=%f, b_1=%f))", a_1, b_1);
    throw std::runtime_error(std::string(buf));
  }
  status = gsl_sf_psi_e(a_1, &psi_a1);
  if(status != GSL_SUCCESS) {
    char buf[200];
    sprintf(buf, "Problem calculating psi(a_1=%f)", a_1);
    throw std::runtime_error(std::string(buf));
  }
  status = gsl_sf_psi_e(b_1, &psi_b1);
  if(status != GSL_SUCCESS) {
    char buf[200];
    sprintf(buf, "Problem calculating psi(b_1=%f)", b_1);
    throw std::runtime_error(std::string(buf));
  }
  status = gsl_sf_psi_e(a_1+b_1, &psi_ab1);
  if(status != GSL_SUCCESS) {
    char buf[200];
    sprintf(buf, "Problem calculating psi(a_1+b_1=%f)", a_1 + b_1);
    throw std::runtime_error(std::string(buf));
  }
  return lnbeta_2.val - lnbeta_1.val
          + (a_1 - a_2) * psi_a1.val
          + (b_1 - b_2) * psi_b1.val
          + (a_2 - a_1 + b_2 * b_1) * psi_ab1.val;
}

double icl::kl_diff_beta(const double &a_1, const double &b_1, const double &a_2, const double &b_2) {
  return kld(a_1, b_1, a_2, b_2) + kld(a_2, b_2, a_1, b_1);
}

double icl::bhatta_dist_cat(const double &a_1, const double &b_1, const double &a_2, const double &b_2) {
  // Cache some values to make it run faster
  return 0.5 * (std::log(a_1) + std::log(b_1) + std::log(a_2) + std::log(b_2))
         + std::log(a_1 + b_1) + std::log(a_2 + b_2);
}

double search_and_add(const double &a, const double b) {
  cache_iterator it = bhatta_cache.find(std::make_pair(a, b));
  double retval;
  if(it != bhatta_cache.end())
    retval = it->second;
  else {
    gsl_sf_result lnbeta;
    int status = gsl_sf_lnbeta_e(a, b, &lnbeta);
    if(status != GSL_SUCCESS) {
      char buf[200];
      sprintf(buf, "Problem calculating ln(beta((a=%f, b=%f))", a, b);
      throw std::runtime_error(std::string(buf));
    }
    retval = lnbeta.val;
    bhatta_cache[std::make_pair(a, b)] = retval;
  }
  return retval;
}

double icl::bhatta_dist_beta(const double &a_1, const double &b_1, const double &a_2, const double &b_2) {
  // Cache some values to make it run faster
  double ln_beta_1, ln_beta_2;
  ln_beta_1 = search_and_add(a_1, b_1);
  ln_beta_2 = search_and_add(a_2, b_2);

  gsl_sf_result lnbeta;
  int status = gsl_sf_lnbeta_e(0.5* (a_1 + a_2), 0.5 * (b_1 + b_2), &lnbeta);
  if(status != GSL_SUCCESS) {
    char buf[200];
    sprintf(buf, "Problem calculating ln(beta((a=%f, b=%f))", 0.5* (a_1 + a_2), 0.5 * (b_1 + b_2));
    throw std::runtime_error(std::string(buf));
  }
  return 0.5 * (ln_beta_1 + ln_beta_2) - lnbeta.val;
}

void icl::clear_bhatta_cache() {
  bhatta_cache.clear();
}

