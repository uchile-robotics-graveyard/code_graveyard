#include "mp_tracker.h"
#include "util.h"
#include <boost/math/special_functions/fpclassify.hpp>
#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/StdVector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "munkres.h"

#define HEAD_PROPORTION 6.0 / 7.0
#define SIM_OBS_SIZE 360
#define USE_DIM 5
#ifdef SAMPLE_FROM_MIXTURE
#include <opencv2/core/eigen.hpp>
#include <opencv2/ml/ml.hpp>
#endif

void icl::MPTracker::observation_2d_fullbody(const person_state::ptr &person, const Eigen::Vector3d &error, measurement_2d_fullbody &out, const Eigen::Matrix<double,3,4> &K) {
  Eigen::Vector4d middle, feet, head;
  feet   << person->posvelheight(0), person->posvelheight(1), 0.0, 1.0;
  middle << person->posvelheight(0), person->posvelheight(1), person->posvelheight(4) / 2.0, 1.0;
  head   << person->posvelheight(0), person->posvelheight(1), person->posvelheight(4), 1.0;
  Eigen::Vector3d pixel = K * middle;
  out.u = pixel(0) / pixel(2) + error(0);
  out.v = pixel(1) / pixel(2) + error(1);
  Eigen::Vector3d pixel_feet = K * feet;
  Eigen::Vector3d pixel_head = K * head;
  out.h = pixel_feet(1) / pixel_feet(2) - pixel_head(1) / pixel_head(2) + error(2);
  //  out.h = (out.h < 96 ? 96 : out.h);
}

double icl::MPTracker::laser_reading_likelihood_simple(const Eigen::VectorXd &person, const pcl::KdTree<pcl::PointXYZ>::Ptr &tree) {
  // The expected distance of a measured point to the map is 10cm = 0.1m
  const int K = 30;
  std::vector<int> indices(K);
  std::vector<float> sq_dists(K);
  pcl::PointXYZ query;
  query.x = person(0);
  query.y = person(1);
  query.z = 0.0;
  int found = tree->radiusSearch(query, 0.5, indices, sq_dists, K);
  double likelihood = 1.0;
  int valid_points = 0;
  double sigma2_ = sigma*sigma;
  for(int j=0; j < found; j++) {
    likelihood *= std::exp(-sq_dists[j] / sigma2_ );
    valid_points++;
  }
  if(valid_points)
    return std::pow(likelihood, 1.0 / valid_points);
  else return 0;
}

double icl::MPTracker::laser_reading_likelihood(const Eigen::VectorXd &person, const Eigen::Matrix4d &T, const std::vector<float> &ranges, const double &start_angle, const double &angle_step) {
  Eigen::Vector4d person_l = T * (Eigen::Vector4d() << person(0), person(1), 0.0, 1.0).finished();
  double angle = std::atan2(person_l(1), person_l(0));
  double range = person_l.block(0,0,3,1).norm();
  int angle_bin = (angle - start_angle) / angle_step;
  const double sigma_2 = sigma*sigma;
  double likelihood = 1.0;
  size_t valid = 0;
#ifdef DEBUG
  std::cout << "# point: " << person_l.transpose() << " with angle " << angle << " and range " << range << " got bin " << angle_bin << std::endl;
#endif
  for(int bin = angle_bin - 1; bin <= angle_bin + 1; bin++) {
    if(bin < 0 || bin >= ranges.size())
      continue;
    double dr = ranges[bin] - range;
    if(dr > 0.0) {
      likelihood *= std::exp(-dr*dr / sigma_2);
    } else {
      likelihood *= 0.9 * std::exp(-dr*dr / sigma_2) + 0.1;
    }
    valid ++;
  }
  if(valid > 0)
    return std::pow(likelihood, 1.0 / valid);
  return 0.0;
}

icl::MPTracker::MPTracker(): states_t(0), states_tp1(0), num_targets(0)
#ifdef MANAGE_TRACKS
			     , start_running(0), last_update_time(0), last_path(0)
#endif
{
  F = Eigen::Matrix<double,5,5>::Identity();
  F(0, 2) = 1.0;
  F(1, 3) = 1.0;
  num_targets = 0;
  // Likelihood field parameters
  r_max = 30;
  sigma = 0.4;
  sigma_2 = 2.0*sigma*sigma;
  p_const = 1.0 / (2.0 * M_PI * sigma);
  angle_epsilon = 3.0 / 180.0 * M_PI;

  static_map.initialised = false;

  p_fullbody_detect = 0.75;
  clutter_fullbody_rfs = 1E-5;
  truncate_threshold = 1E-12;
  merge_threshold = 1E-4;

  visual_max_dist_detect = 10.0;
  visual_min_dist_detect = 2.0;

  cluster_proximity = 0.10;

  pixel_error = 100;

  birth_prior_weight = 1E-6;
}

void icl::MPTracker::predict(const double &dt, const Eigen::Vector3d &robot_pos, const double &time) {
#ifdef MANAGE_TRACKS
  last_update_time = time;
#endif
  Eigen::Matrix<double,5,3> w;
  // The expected change in height of a person is really low
  w << dt*dt / 2.0, 0.0, 0.0,
    0.0, dt*dt / 2.0, 0.0,
    dt, 0.0, 0.0,
    0.0, dt, 0.0,
    0.0, 0.0, 0.01;
  Q = w * w.transpose();
  F(0, 2) = dt;
  F(1, 3) = dt;
  // The height is constant
  F(4, 4) = 1.0;

  states_tp1.resize(states_t.size());

  for(size_t i = 0; i < states_t.size(); i++) {
    states_t.at(i)->weight *= p_survival;
    states_t.at(i)->posvelheight = F * states_t.at(i)->posvelheight;
    states_t.at(i)->posvelheight_cov = F * states_t.at(i)->posvelheight_cov * F.transpose() + Q;
    states_tp1[i] = states_t.at(i);
  }
}

void icl::MPTracker::correct_laser_mc(const std::vector<float> &ranges, double min_angle, double angle_step, const Eigen::Matrix4d &T) {
  if(states_tp1.size() == 0 || !static_map.initialised) {
    return;
  }
#ifdef SAMPLE_FROM_MIXTURE
  // Sample from the mixture of gaussians
  std::sort(states_tp1.begin(), states_tp1.end());
  std::reverse(states_tp1.begin(), states_tp1.end());
  double total_weight = 0;
  for(size_t i = 0; i < states_tp1.size(); i++) total_weight += states_tp1.at(i)->weight;
  // 30 particles per dimention per number of people
  size_t num_samples = std::max((int)(30*5*total_weight), 200);
  std::vector<particle> particles(num_samples);
  sample_from_mixture(states_tp1, num_samples, particles);

  // Create a pointcloud from the laser reading
  std::vector<int> segments;
  std::map<int, double> segment_lengths;
  std::map<int, int> segment_count;
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pointcloud_from_laser_2(ranges, min_angle, angle_step, 30.0, T, laser_pc, cluster_proximity, segments, segment_lengths, segment_count);

  // Create a KD-Treee for fast KNN searches
  pcl::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  tree->setEpsilon(0.1);
  tree->setInputCloud(laser_pc);

  boost::function<double (const Eigen::VectorXd&)> likelihood;
  if(use_map_likelihood)
    likelihood = boost::bind(&MPTracker::laser_reading_likelihood, this, _1, T.inverse(), ranges, min_angle, angle_step);
  else
    likelihood = boost::bind(&MPTracker::laser_reading_likelihood_simple, this, _1, tree);
#ifdef DEBUG
  std::cout << "particles=[" << std::endl;
#endif
  double weight = 0.0;
  size_t valid_likelihoods = 0;
  // Evaluate the likelihood function for each sample
  for(size_t i = 0; i < num_samples; i++) {
    double particle_likelihood = likelihood(particles[i].state);
    if(particle_likelihood > 0) valid_likelihoods++;
#ifdef DEBUG
    std::cout << "[" << particles[i].state(0) << ", "
      << particles[i].state(1) << ", "
      << particles[i].state(2) << ", " 
      << particles[i].state(3) << ", " 
      << particles[i].weight << ", "
      << particle_likelihood << "]," << std::endl;
#endif
    particles.at(i).weight = particles.at(i).weight * particle_likelihood;
    weight += particles.at(i).weight;
  }
#ifdef DEBUG
  std::cout << "]" << std::endl;
#endif
  if(weight == 0 || valid_likelihoods < 5) {
    states_t.resize(0);
    return;
  }
  // Resample particles to get uniform weights
  std::sort(particles.begin(), particles.end());
  std::reverse(particles.begin(), particles.end());
  std::vector<particle> resampled(num_samples);
  sample_from_samples(particles, num_samples, resampled);

#ifdef DEBUG
  std::cout << "resampled_particles=[" << std::endl;
  for(size_t k = 0; k < resampled.size(); k++) {
    std::cout << "[" << resampled[k].state(0) << ", "
      << resampled[k].state(1) << ", "
      << resampled[k].state(2) << ", " 
      << resampled[k].state(3) << ", " 
      << resampled[k].weight << "]," << std::endl;
  }
  std::cout << "]" << std::endl;
#endif

  // Use EM-GMM to recreate the gaussian mixture
  size_t num_components = (total_weight < 0.5 ? 1.0 : std::floor(total_weight + 0.5));
  size_t num_models = 5;
#if 1
  std::vector<cv::EM> models(num_models);
  double best_complexity = std::numeric_limits<double>::max();
  size_t final_mixture = 0;
  size_t model_indx = 0;
  cv::Mat samples(num_samples,5,  CV_32FC1);
  for(size_t i = 0; i < num_samples; i++) {
    for(size_t j = 0; j < 5; j++)
      samples.at<float>(i,j) = resampled[i].state(j);
  }
  for(size_t i = 0; i < num_models; i++) {
    int num_mixtures = num_components - std::floor(0.5*num_models + 0.5) + i;
    if(num_mixtures <= 0)
      continue;
    models[i] = cv::EM(num_mixtures, cv::EM::COV_MAT_GENERIC);
    cv::Mat samples_loglike(num_samples, 1, CV_64FC1);
    try {
      models[i].train(samples, samples_loglike);
    } catch(...) {++i; continue;}

    double loglike = cv::sum(samples_loglike)[0];
    // Number of parameters: one weight per component minus one, as the sum up to 1
    //                       + the mean for each component and half the covariance
    //                       matrix (as it is symetric)
    /*
       const int dim = 5;
       const double num_vars = num_mixtures - 1.0
       + num_mixtures * dim
       + num_mixtures * (dim * dim + dim) / 2.0;
     */
    const double num_vars = num_mixtures - 1.0 + num_mixtures * 20.0;
    // BIC
    double complexity = -2.0 * loglike  + num_vars * std::log(num_samples);
    // AICc
    //    double complexity = 2.0 * num_vars - 2.0 * loglike + 2.0 * num_vars * (num_vars + 1.0) / (num_samples - num_vars - 1);
#ifdef DEBUG
    std::cout << "# checking " << num_mixtures << " components with Complexity: " << complexity << std::endl;
#endif
    if(complexity < best_complexity) {
      best_complexity = complexity;
      model_indx = i;
      final_mixture = num_mixtures;
    }
  }
  const cv::Mat means(models[model_indx].get<cv::Mat>("means"));
  const cv::Mat weights(models[model_indx].get<cv::Mat>("weights"));

  // Copy back to the states_t
  states_t.resize(final_mixture);
  num_targets = 0;
  for(size_t i = 0; i < final_mixture; i++) {
    states_t.at(i).reset(new person_state);
    const cv::Mat cov(models[model_indx].get<std::vector<cv::Mat> >("covs")[i]);
    states_t.at(i)->weight = total_weight * weights.at<double>(i);
    num_targets += states_t.at(i)->weight;
    cv::cv2eigen(means.row(i), states_t.at(i)->posvelheight);
    cv::cv2eigen(cov, states_t.at(i)->posvelheight_cov);
  }
#else
  std::vector<CvEM> models(num_models);
  double best_complexity = std::numeric_limits<double>::max();
  size_t final_mixture = 0;
  CvEM *damodel = NULL;
  cv::Mat samples(num_samples,5,  CV_32FC1);
  for(size_t i = 0; i < num_samples; i++) {
    for(size_t j = 0; j < 5; j++)
      samples.at<float>(i,j) = resampled[i].state(j);
  }
  CvMat stupid = samples;
  for(size_t i = 0; i < num_models; i++) {
    int num_mixtures = num_components - std::floor(0.5*num_models + 0.5) + i;
    if(num_mixtures <= 0)
      continue;
    CvEMParams params;
    params.covs = NULL;
    params.means = NULL;
    params.weights = NULL;
    params.probs = NULL;
    params.nclusters = num_mixtures;
    params.cov_mat_type = CvEM::COV_MAT_GENERIC;
    params.start_step = CvEM::START_AUTO_STEP;
    params.term_crit.max_iter = 70;
    params.term_crit.epsilon = 0.005;
    params.term_crit.type = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;

    try {
      models[i].train(&stupid, 0, params);
    } catch(...) { i++;continue; }

    double loglike = models[i].get_log_likelihood();
    // If there's only one mixture EM does not estimate the log likelihood
    if(num_mixtures == 1) {
      loglike = 0.0;
      Eigen::Matrix<double,5,1> mean;
      Eigen::Matrix<double,5,5> cov;
      cv::Mat tmp_mean(models[i].get_means());
      cv::Mat tmp_cov(models[i].get_covs()[0]);
      cv::cv2eigen(tmp_mean.row(0).t(), mean);
      cv::cv2eigen(tmp_cov, cov);
      for(size_t j = 0; j < num_samples; j++) {
	loglike += log_normal_eval(resampled[j].state, mean, cov);
      }
    }
    // Number of parameters: one weight per component minus one, as the sum up to 1
    //                       + the mean for each component and half the covariance
    //                       matrix (as it is symetric)
    /*
       const int dim = 5;
       const double num_vars = num_mixtures - 1.0
       + num_mixtures * dim
       + num_mixtures * (dim * dim + dim) / 2.0;
     */
    const double num_vars = num_mixtures - 1.0 + num_mixtures * 20.0;
    // BIC
    double complexity = -2.0 * loglike  + num_vars * std::log(num_samples);
    // AICc
    //    double complexity = 2.0 * num_vars - 2.0 * loglike + 2.0 * num_vars * (num_vars + 1.0) / (num_samples - num_vars - 1);
#ifdef DEBUG
    std::cout << "# checking " << num_mixtures << " components with Complexity: " << complexity << std::endl;
#endif
    if(complexity < best_complexity) {
      best_complexity = complexity;
      damodel = &models[i];
      final_mixture = num_mixtures;
    }
  }
  const cv::Mat means(damodel->get_means());
  const cv::Mat weights(damodel->get_weights());

  // Copy back to the states_t
  states_t.resize(final_mixture);
  num_targets = 0;
  for(size_t i = 0; i < final_mixture; i++) {
    states_t.at(i).reset(new person_state);
    const cv::Mat cov(damodel->get_covs()[i]);
    states_t.at(i)->weight = total_weight * weights.at<double>(i);
    num_targets += states_t.at(i)->weight;
    cv::cv2eigen(means.row(i), states_t.at(i)->posvelheight);
    cv::cv2eigen(cov, states_t.at(i)->posvelheight_cov);
  }
#endif
#ifdef DEBUG
  std::cout << "means_laser=[]" << std::endl << "stds_laser=[]" << std::endl << "weights_laser=[]" << std::endl;
  for(size_t i = 0; i < states_t.size(); i++) {
    std::cout << "means_laser.append([";
    for(size_t j = 0; j < 5; j++) {
      std::cout << states_t[i]->posvelheight(j) << ", ";
    }
    std::cout << "])" << std::endl;
    std::cout << "stds_laser.append([";
    for(size_t j = 0; j < 5; j++) {
      std::cout << "[";
      for(size_t k = 0; k < 5; k++) {
	std::cout << states_t[i]->posvelheight_cov(j, k) << ", ";
      }
      std::cout << "], " << std::endl;
    }
    std::cout << "])" << std::endl;
    std::cout << "weights_laser.append(" << states_t[i]->weight << ")" << std::endl;
  }
#endif
#else
#ifdef DEBUG
  std::cout << "means_smpl=[]" << std::endl << "stds_smpl=[]" << std::endl << "weights_smpl=[]" << std::endl;
  for(size_t i = 0; i < states_tp1.size(); i++) {
    std::cout << "means_smpl.append([";
    for(size_t j = 0; j < 5; j++) {
      std::cout << states_tp1[i]->posvelheight(j) << ", ";
    }
    std::cout << "])" << std::endl;
    std::cout << "stds_smpl.append([";
    for(size_t j = 0; j < 5; j++) {
      std::cout << "[";
      for(size_t k = 0; k < 5; k++) {
	std::cout << states_tp1[i]->posvelheight_cov(j, k) << ", ";
      }
      std::cout << "], " << std::endl;
    }
    std::cout << "])" << std::endl;
    std::cout << "weights_smpl.append(" << states_tp1[i]->weight << ")" << std::endl;
  }
#endif
  // Consider the agregated state as one and sample from it
  const size_t num_samples = 150;
  std::vector<particle> particles(num_samples);

  // Create a pointcloud from the laser reading
  std::vector<int> segments;
  std::map<int, double> segment_lengths;
  std::map<int, int> segment_count;
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pointcloud_from_laser(ranges, min_angle, angle_step, 30.0, T, laser_pc, cluster_proximity, segments, segment_lengths, segment_count);

  // Create a KD-Treee for fast KNN searches
  pcl::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  tree->setEpsilon(0.1);
  tree->setInputCloud(laser_pc);

#ifdef DEBUG
  std::cout << "particles=[]" << std::endl;
#endif
  boost::function<double (const Eigen::VectorXd&)> likelihood;
  if(use_map_likelihood)
    likelihood = boost::bind(&MPTracker::laser_reading_likelihood, this, _1, T, ranges, min_angle, angle_step);
  else
    likelihood = boost::bind(&MPTracker::laser_reading_likelihood_simple, this, _1, tree);

  for(size_t k = 0; k < states_tp1.size(); k++) {
    double weight = 0.0;
    const Eigen::MatrixXd &S = states_tp1.at(k)->posvelheight_cov.block(0, 0, USE_DIM, USE_DIM);
    const Eigen::VectorXd &u = states_tp1.at(k)->posvelheight.block(0, 0, USE_DIM, 1);
    // Sample from the k-th gaussian
    // Generate a N(0,I)
    Eigen::MatrixXd Sp = S;
    std::vector<Eigen::VectorXd> samples;
    sample_normal_uS(u, Sp, num_samples, samples);
    for(size_t i = 0; i < num_samples; i++) {
      particles.at(i).state = samples[i];
      const double local_weight = 1.0 / num_samples;
      particles.at(i).weight = local_weight;
    }
    // Calculate likelihood of the samples
#ifdef DEBUG
    std::cout << "particles.append([" << std::endl;
#endif
    size_t valid_likelihoods = 0;
    for(size_t i = 0; i < num_samples; i++) {
      double particle_likelihood = likelihood(particles[i].state);
      if(particle_likelihood > 0) valid_likelihoods++;
#ifdef DEBUG
      std::cout << "[" << particles[i].state(0) << ", "
	<< particles[i].state(1) << ", "
	<< particles[i].state(2) << ", " 
	<< particles[i].state(3) << ", " 
	<< particles.at(i).weight << ", "
	<< particle_likelihood << "]," << std::endl;
#endif
      particles.at(i).weight = particles.at(i).weight * particle_likelihood;
      weight += particles.at(i).weight;
    }
#ifdef DEBUG
    std::cout << "])" << std::endl;
#endif
    if(weight == 0 || valid_likelihoods < 5) {
#ifdef DEBUG
      std::cout << "# total weights = 0 for component " << k << std::endl;
#endif
      states_tp1.erase(states_tp1.begin() + k);
      k--;
      continue;
    }
    // re-generate state vector
    Eigen::VectorXd new_mean = Eigen::VectorXd::Zero(USE_DIM);
    for(size_t i = 0; i < num_samples; i++) {
      particles.at(i).weight /= weight;
      new_mean += particles.at(i).weight * particles.at(i).state;
    }
    // re-generate the covariance
    Eigen::MatrixXd new_cov = Eigen::MatrixXd::Zero(USE_DIM, USE_DIM);
    double sum_weights_sq = 0.0;
    for(size_t i = 0; i < num_samples; i++) {
      const double &local_weight = particles.at(i).weight;
      sum_weights_sq += local_weight * local_weight;
      Eigen::VectorXd dx = particles.at(i).state - new_mean;
      new_cov += local_weight * (dx * dx.transpose());
    }
    new_cov *= 1.0 / (1.0 - sum_weights_sq);
#ifdef DEBUG
    if(boost::math::isnan(new_mean(0)) || boost::math::isnan(new_cov(0,0))) {
      std::cout << "# problem with component " << k << std::endl;
    }
#endif
#define HACK_0
    // Hack to avoid getting too narrow
#ifdef HACK_0
    new_cov += 0.01*Eigen::MatrixXd::Identity(USE_DIM, USE_DIM);
#elif defined HACK_1
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(new_cov);
    Eigen::VectorXd eigval = solver.eigenvalues();
    Eigen::MatrixXd Q = solver.eigenvectors();
    Eigen::MatrixXd A = (eigval.array() < 0.0025).select(0.0025, eigval).asDiagonal();
    new_cov = Q * A * Q.inverse();
#endif
    // Copy back to the state.
    states_tp1.at(k)->posvelheight.block(0, 0, USE_DIM, 1) = new_mean;
    states_tp1.at(k)->posvelheight_cov.block(0, 0, USE_DIM, USE_DIM) = new_cov;
    // Reestimate the weights?
  }
  states_t.resize(states_tp1.size());
  std::copy(states_tp1.begin(), states_tp1.end(), states_t.begin());
  std::sort(states_t.begin(), states_t.end());
  std::reverse(states_t.begin(), states_t.end());
#ifdef DEBUG
  std::cout << "means=[]" << std::endl << "stds=[]" << std::endl << "weights=[]" << std::endl;
  for(size_t i = 0; i < states_t.size(); i++) {
    std::cout << "means.append([";
    for(size_t j = 0; j < 5; j++) {
      std::cout << states_t[i]->posvelheight(j) << ", ";
    }
    std::cout << "])" << std::endl;
    std::cout << "stds.append([";
    for(size_t j = 0; j < 5; j++) {
      std::cout << "[";
      for(size_t k = 0; k < 5; k++) {
	std::cout << states_t[i]->posvelheight_cov(j, k) << ", ";
      }
      std::cout << "], " << std::endl;
    }
    std::cout << "])" << std::endl;
    std::cout << "weights.append(" << states_t[i]->weight << ")" << std::endl;
  }
#endif
#endif
}

void icl::MPTracker::correct_feature_2d_fullbody_ut(const std::vector<measurement_2d_fullbody> &meas, const Eigen::Matrix<double,3,4> &PK, const Eigen::Matrix<double,4,4> &T, const std::vector<person_state::ptr> &birth_prior) {
#ifdef DEBUG
  std::cout << "measurements_fb=[" << std::endl;
  for(size_t i = 0; i < meas.size(); i++) {
    std::cout << "[" << meas[i].u << ", " << meas[i].v << ", " << meas[i].h << ", " << meas[i].weight << "]," << std::endl;
  }
  std::cout << "]" << std::endl;
#endif
  size_t num_mixtures = states_tp1.size();
  size_t num_observation = meas.size();

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>  > n(num_mixtures);
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>  > S(num_mixtures);
  std::vector<Eigen::Matrix<double,5,3>, Eigen::aligned_allocator<Eigen::Matrix<double,5,3> >  > K(num_mixtures);
  std::vector<Eigen::Matrix<double,5,3>, Eigen::aligned_allocator<Eigen::Matrix<double,5,3> >  > G(num_mixtures);
  std::vector<Eigen::Matrix<double,5,5>, Eigen::aligned_allocator<Eigen::Matrix<double,5,3> >  > P(num_mixtures);
  std::vector<double> prob_detect(num_mixtures);

  person_state::ptr person_aux(new person_state);
  measurement_2d_fullbody meas_aux;
  for(size_t i = 0; i < num_mixtures; i++) {
#ifdef DEBUG
    cout << "# analysing mixture: " << i << std::endl;
#endif
    std::vector<Eigen::VectorXd> sigma_points;
    std::vector<double> sigma_weights;
    Eigen::VectorXd point = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Sigma = pixel_error * Eigen::MatrixXd::Identity(8,8);
    // The error is assumed to be N(0,1) for now
    point.block(0,0,5,1) = states_tp1.at(i)->posvelheight;
    Sigma.block(0,0,5,5) = states_tp1.at(i)->posvelheight_cov;
    Sigma(7, 7) = height_error;
    // Create the sigma points
    unsctd_sampler.get_points(point, Sigma, 0.3, 1.0, sigma_points, sigma_weights);
    // Assumes error dh/dx ~ 1
    n[i] = Eigen::Matrix<double,3,1>::Zero();
    S[i] = Eigen::Matrix<double,3,3>::Zero();
    K[i] = Eigen::Matrix<double,5,3>::Zero();
    G[i] = Eigen::Matrix<double,5,3>::Zero();
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>  > z(sigma_points.size());

    for(size_t j = 0; j < sigma_points.size(); j++) {
      const Eigen::Matrix<double,5,1> &x_j = sigma_points.at(j).block(0,0,5,1);
      const Eigen::Vector3d error = sigma_points.at(j).block(5,0,3,1);
      person_aux->posvelheight = x_j;
      observation_2d_fullbody(person_aux, error, meas_aux, PK);
      z[j] << meas_aux.u, meas_aux.v, meas_aux.h;
#ifdef DEBUG
      cout << "# observation: " << j << " for sigma point: " << x_j.transpose() << ", " << error.transpose() << " is " << z[j].transpose() << std::endl;
#endif
      n[i] += sigma_weights.at(j) * z[j];
    }
    for(size_t j = 0; j < sigma_points.size(); j++) {
      const Eigen::Matrix<double,5,1> &x_j = sigma_points.at(j).block(0,0,5,1);
      const double &weight_j = sigma_weights.at(j);
      const Eigen::Vector3d dz = z[j] - n[i];
      S[i] += weight_j * dz * dz.transpose();
      G[i] += weight_j * (x_j - states_tp1.at(i)->posvelheight) * dz.transpose();
    }
    K[i] = G[i] * S[i].inverse();
    P[i] = states_tp1.at(i)->posvelheight_cov -  K[i] * G[i].transpose();

    // Estimate probability of detection
    // I think this is wrong
    /*
       const Eigen::Vector2d robot_pos = T.block(0,3,2,1);
       unscented_sampler<> smplr;
       std::vector<double> weights2d;
       std::vector<Eigen::VectorXd> points2d;
       smplr.get_points(point.block(0,0,2,1), Sigma.block(0,0,2,2), 0.3, 1.0, points2d, weights2d);
       prob_detect[i] = 0.0;
       for(size_t j = 0; j < points2d.size(); j++) {
       const Eigen::Vector2d &x_j = points2d.at(j).block(0,0,2,1);
       const double dist_ = (x_j-robot_pos).norm();
       std::cout << "#sigma weight: " << weights2d.at(j) << ", point: " << points2d.at(j).transpose() << ", dist to robot: " << dist_ << std::endl;
       if(dist_ < visual_max_dist_detect && dist_ > visual_min_dist_detect) {
       prob_detect[i] += weights2d.at(j) * p_fullbody_detect;
       } else {
       prob_detect[i] += weights2d.at(j) * 0.01;
       }
       }
     */
    prob_detect[i] = p_fullbody_detect;
  }

  // Generate the final array
  size_t total_components = num_mixtures +  num_observation * (num_mixtures + 1);
  states_t.resize(total_components);
  // Components not observed
  for(size_t i = 0; i < num_mixtures; i++) {
    states_t[i].reset(new person_state(*states_tp1.at(i)));
#ifdef DEBUG
    std::cout << "# component " << i << ", before: " << states_t[i]->weight << ", after " << states_t[i]->weight * (1 - prob_detect[i])<< std::endl;
#endif
    states_t[i]->weight *= (1 - prob_detect[i]);
  }
  // Observed components
  for(size_t l = 0; l < num_observation; l++) {
    double sum_weight = 0.0;
    Eigen::Vector3d obs;
    obs << meas[l].u, meas[l].v, meas[l].h;
    //clutter_fullbody_rfs = 0.5 * (1.0 - erf(0.1 * meas[l].weight + 3.3));
    for(size_t j = 0; j < num_mixtures; j++) {
      unsigned int indx = num_mixtures + l*(num_mixtures + 1)+j;
      person_state::ptr person(new person_state);
      person->weight = prob_detect[j] * states_tp1[j]->weight * normal_eval(obs, n[j], S[j]);
#ifdef DEBUG
      std::cout << "# components of the " << indx << " weights : " << prob_detect[j] << " * " << states_tp1[j]->weight << " * " << normal_eval(obs, n[j], S[j]) << " = " << person->weight << "(n[" << j << "] = " << n[j].transpose() << ")" << std::endl;
#endif
      sum_weight += person->weight;
      person->posvelheight = states_tp1[j]->posvelheight + K[j] * (obs - n[j]);
      person->posvelheight_cov = P[j];
      states_t[indx] = person;
    }
    unsigned int birth_prior_indx = num_mixtures + l*(num_mixtures + 1) + num_mixtures;
    if(l < birth_prior.size()) {
      sum_weight += birth_prior_weight;
      states_t[birth_prior_indx] = birth_prior[l];
      states_t[birth_prior_indx]->weight = birth_prior_weight / (clutter_fullbody_rfs + sum_weight);
    } else {
      // Dummy state with weight 0
      states_t[birth_prior_indx].reset(new person_state());
      states_t[birth_prior_indx]->weight = 0.0;
    }
    for(size_t j = 0; j < num_mixtures; j++) {
      unsigned int indx = num_mixtures + l*(num_mixtures + 1)+j;
#ifdef DEBUG
      std::cout << "# normalisation of component " << indx << " original : " << states_t[indx]->weight << ", normalised:  " <<  states_t[indx]->weight / (clutter_fullbody_rfs + sum_weight) << ". Clutter: " << clutter_fullbody_rfs << std::endl;
#endif
      states_t[indx]->weight /= (clutter_fullbody_rfs + sum_weight);
    }
  }
  // Append the birth prior

  std::sort(states_t.begin(), states_t.end());
  std::reverse(states_t.begin(), states_t.end());
#ifdef DEBUG
  std::cout << "means_fb=[]" << std::endl << "stds_fb=[]" << std::endl << "weights_fb=[]" << std::endl;
  for(size_t i = 0; i < states_t.size(); i++) {
    std::cout << "means_fb.append([";
    for(size_t j = 0; j < 5; j++) {
      std::cout << states_t[i]->posvelheight(j) << ", ";
    }
    std::cout << "])" << std::endl;
    std::cout << "stds_fb.append([";
    for(size_t j = 0; j < 5; j++) {
      std::cout << "[";
      for(size_t k = 0; k < 5; k++) {
	std::cout << states_t[i]->posvelheight_cov(j, k) << ", ";
      }
      std::cout << "], " << std::endl;
    }
    std::cout << "])" << std::endl;
    std::cout << "weights_fb.append(" << states_t[i]->weight << ")" << std::endl;
  }
#endif
}

void icl::MPTracker::prune() {
  for(std::vector<person_state::ptr>::iterator it = states_t.begin(); it != states_t.end(); it++) {
    // Check for this mixture to be consistend
    person_state::ptr person = *it;
    if(boost::math::isnan(person->posvelheight) || boost::math::isnan(person->weight)) {
      it = states_t.erase(it);
      it--;
    }
  }
  std::vector<unsigned int> I;
  std::vector<person_state::ptr> final_mixture;
  num_targets = 0;
  std::vector<Eigen::Matrix<double,5,5>, Eigen::aligned_allocator<Eigen::Matrix<double,5,5> > > inverses;
  // Get the initial values for I
  for(unsigned int i = 0; i < states_t.size(); i++) {
    inverses.push_back(states_t[i]->posvelheight_cov.inverse());
    if(!boost::math::isnan(states_t[i]->weight) && states_t[i]->weight > truncate_threshold) {
      I.push_back(i);
    }
  }
  if(I.size() == 0) {
    states_t.resize(0);
    return;
  }
  do {
    // Find the components with the highest weight in I
    double max_weight = -1;
    unsigned int max_indx = 0;
    size_t remv_indx = 0;
    for(unsigned int i = 0; i < I.size(); i++) {
      const double &weight = states_t[I[i]]->weight;
      if(weight > max_weight) {
	max_indx = I[i];
	max_weight = weight;
	remv_indx = i;
      }
    }
    if(max_weight < 0)
      throw std::runtime_error("Negatives weights in components");
    // Remove this component from I
    I.erase(I.begin() + remv_indx);
    // Find the components near the the component found previously and merge
    std::vector<unsigned int> indices;
    Eigen::Matrix<double,5,1> average_state = states_t[max_indx]->weight * states_t[max_indx]->posvelheight;
    double total_weight = states_t[max_indx]->weight;
    const Eigen::Matrix<double,5,1> &max_state = states_t[max_indx]->posvelheight;
    for(unsigned int i = 0; i < I.size(); i++) {
      Eigen::Matrix<double,5,1> dx = states_t[I[i]]->posvelheight - max_state;
      double dist = dx.transpose()*inverses[I[i]]*dx;
      if(dist < merge_threshold) {
	indices.push_back(I[i]);
	total_weight += states_t[I[i]]->weight;
	average_state += states_t[I[i]]->weight * states_t[I[i]]->posvelheight;
      }
    }
    // Generate the new component
    person_state::ptr  merged_item(new person_state);
    merged_item->weight = total_weight;
    merged_item->posvelheight = average_state / total_weight;
    Eigen::Matrix<double,5,1> dx = states_t[max_indx]->posvelheight - merged_item->posvelheight;
    merged_item->posvelheight_cov = states_t[max_indx]->weight * (states_t[max_indx]->posvelheight_cov + dx*dx.transpose());
    for(unsigned int i = 0; i < indices.size(); i++) {
      const person_state::ptr &current_state = states_t[indices[i]];
      dx = current_state->posvelheight - merged_item->posvelheight;
      merged_item->posvelheight_cov += current_state->weight * (current_state->posvelheight_cov + dx*dx.transpose());
      // Remove the element from I
      std::vector<unsigned int>::iterator it = std::find(I.begin(), I.end(), indices[i]);
      if(it != I.end())
	I.erase(it);
    }
    merged_item->posvelheight_cov = merged_item->posvelheight_cov / total_weight;
    // Check if the weights are big enough
    if(final_mixture.size() < max_components) {
      // Add to the final mixture
      final_mixture.push_back(merged_item);
      // Sort the elements in decreasing order
      std::sort(final_mixture.begin(), final_mixture.end());
      num_targets += merged_item->weight;
    } else if(final_mixture[final_mixture.size()-1]->weight < merged_item->weight) {
      // Replace to with the final item
      final_mixture[final_mixture.size()-1] = merged_item;
      // Sort the elements in decreasing order
      std::sort(final_mixture.begin(), final_mixture.end());
      num_targets += merged_item->weight;
    }
  } while (I.size() > 0);
  // Assign the final mixture
  states_t.resize(final_mixture.size());
  std::copy(final_mixture.begin(), final_mixture.end(), states_t.begin());
  // Sort them by bigger to smaller weights
  std::sort(states_t.begin(), states_t.end());
  std::reverse(states_t.begin(), states_t.end());
#ifdef DEBUG
  std::cout << "means_prune=[]" << std::endl << "stds_prune=[]" << std::endl << "weights_prune=[]" << std::endl;
  for(size_t i = 0; i < states_t.size(); i++) {
    std::cout << "means_prune.append([";
    for(size_t j = 0; j < 5; j++) {
      std::cout << states_t[i]->posvelheight(j) << ", ";
    }
    std::cout << "])" << std::endl;
    std::cout << "stds_prune.append([";
    for(size_t j = 0; j < 5; j++) {
      std::cout << "[";
      for(size_t k = 0; k < 5; k++) {
	std::cout << states_t[i]->posvelheight_cov(j, k) << ", ";
      }
      std::cout << "], " << std::endl;
    }
    std::cout << "])" << std::endl;
    std::cout << "weights_prune.append(" << states_t[i]->weight << ")" << std::endl;
  }
#endif
}


