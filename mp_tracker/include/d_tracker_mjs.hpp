/*
template<int dim_state, int dim_measurement, int num_modes>
DTrackerMJS<dim_state,dim_measurement,num_modes>::DTracker(bool use_beta, bool cap_max_weights_):
    use_beta_mixture(use_beta),
    cap_max_weights(cap_max_weights_),
    weights_t(0),
    means_t(0),
    covs_t(0),
    weights_tp1(0),
    means_tp1(0),
    covs_tp1(0),
    num_targets(0),
    python_loaded(false) {
  num_targets = 0;
  // Just in case, some default parameters just in case
  p_detect = 0.75;
  clutter_rfs = 1E-5;
  truncate_threshold = 1E-12;
  merge_gauss_threshold = 1E-4;
  merge_beta_threshold = 0.15;
  birth_prior_weight = 1E-6;
}

template<int dim_state, int dim_measurement, int num_modes>
void DTrackerMJS<dim_state, dim_measurement, num_modes>::load_debug_plugin(const std::string &python_file) {
  // Load the debugging module
  try {
    Py_Initialize();
    python_module = python::import("__main__");
    python_namespace = python_module.attr("__dict__");
    python::exec_file(python_file.c_str(), python_namespace, python_namespace);
    python_plotter = python_namespace["debug_plot"];
    python_loaded = true;
    // Release python's GIL. This allows for threads in the python code to keep running
    gil_save = PyEval_SaveThread();
  } catch(...) {
    PyErr_Print();
    PyErr_Clear();
    Py_Finalize();
  }
}

template<int dim_state, int dim_measurement, int num_modes>
void DTrackerMJS<dim_state, dim_measurement, num_modes>::debug_plot(bool t, const pcl::PointCloud<pcl::PointXYZ>::Ptr &laser_pc, const measurements &meas) {
  static size_t frame_num = 0;

  if(!python_loaded)
    return;

  // Aquire python's GIL
  PyEval_RestoreThread(gil_save);

  python::list weights, as, bs, means, covs, laser, obs;
  // Fill the arrays
  if(t) { // do time state_t
    for(size_t i = 0; i < weights_t.size(); i++) {
      // Create the a mean list and append to the means list
      python::list mean;
      for(size_t j = 0; j < dim_state; j++) {
        mean.append(means_t[i](j));
      }
      means.append(mean);
      // Create a covariance double list and append to the covariances list
      python::list cov;
      for(size_t j = 0; j < dim_state; j++) {
        python::list row;
        for(size_t k = 0; k < dim_state; k++) {
          row.append(covs_t[i](j, k));
        }
        cov.append(row);
      }
      covs.append(cov);
      // Append the weights
      weights.append(weights_t[i]);
      if(use_beta_mixture) {
        as.append(a_t[i]);
        bs.append(b_t[i]);
      }
    }
  } else { // do time state_t+1
    for(size_t i = 0; i < weights_tp1.size(); i++) {
      python::list mean;
      for(size_t j = 0; j < dim_state; j++) {
        mean.append(means_tp1[i](j));
      }
      means.append(mean);
      python::list cov;
      for(size_t j = 0; j < dim_state; j++) {
        python::list row;
        for(size_t k = 0; k < dim_state; k++) {
          row.append(covs_tp1[i](j, k));
        }
        cov.append(row);
      }
      covs.append(cov);
      weights.append(weights_tp1[i]);
      if(use_beta_mixture) {
        as.append(a_tp1[i]);
        bs.append(b_tp1[i]);
      }
    }
  }

  // Create the laser list
  for(size_t i = 0; i < laser_pc->size(); i++) {
    python::list point;
    point.append(laser_pc->points[i].x);
    point.append(laser_pc->points[i].y);
    point.append(laser_pc->points[i].z);
    laser.append(point);
  }

  for(size_t i = 0; i < meas.size(); i++) {
    python::list point;
    for(size_t j = 0; j < dim_measurement; j++) {
      point.append(meas[i](j));
    }
    obs.append(point);
  }
  // Call the plotting function
  try {
    if(use_beta_mixture) {
      python_plotter(frame_num++, weights, means, covs, laser, obs, as, bs);
    } else {
      python_plotter(frame_num++, weights, means, covs, laser, obs, python::object(), python::object());
    }
  } catch(...) {
    PyErr_Print();
    PyErr_Clear();
  }
  // Relesae GIL
  gil_save = PyEval_SaveThread();
}

template<int dim_state, int dim_measurement, int num_modes>
DTrackerMJS<dim_state, dim_measurement, num_modes>::~DTracker() {
  if(python_loaded)
    // Aquire python's GIL
    PyEval_RestoreThread(gil_save);
    // Finalise python iterpreter
    Py_Finalize();
}
*/

template<int dim_state, int dim_measurement, int num_modes>
void DTrackerMJS<dim_state, dim_measurement, num_modes>::predict() {
  size_t num_components;
  for(size_t s = 0; s < num_modes; s++) {
    num_components += means_t[s].size();
  }

  size_t indx = 0;
  for(size_t s = 0; s < num_modes; s++) {
    means_tp1[s].resize(num_components);
    covs_tp1[s].resize(num_components);
    weights_tp1[s].resize(num_components);
    for(size_t sp = 0; sp < num_modes; sp++) {
      for(size_t i = 0; i < num_components; i++) {
        weights_tp1[s][indx] = p_survival * weights_t[sp][i] * transition(s,sp);
        means_tp1[s][indx] = F[s] * means_t[sp][i];
        covs_tp1[s][indx] = F[s] * covs_t[sp][i] * F[s].transpose() + Q[s];
        indx++;
      }
    }
  }
}

template<int dim_state, int dim_measurement, int num_modes>
void DTrackerMJS<dim_state, dim_measurement, num_modes>::correct(const measurements &observations) {
  size_t num_observation = observations.size();
  size_t num_mixtures = means_t.size();
  // Construct update components
  measurements h(num_mixtures);
  measurement_mats S(num_mixtures);
  state_measurement_mats K(num_mixtures);
  state_mats P(num_mixtures);

  // Get the transformation between frames
  for(size_t i = 0; i < num_mixtures; i++) {
    // Predict the observation
    h[i] = H * means_tp1[i];
    S[i] = R + H * covs_tp1[i] * H.transpose();
    //K[i] = covs_tp1[i] * H.transpose() * S[i].inverse();
    K[i] = S[i].transpose().ldlt().solve(H * covs_tp1[i].transpose()).transpose();
    P[i] = (state_mat::Identity() - K[i] * H) * covs_tp1[i];
  }
  // Update
  size_t total_components = num_mixtures +  num_observation * (num_mixtures + 1);
  means_t.resize(total_components);
  covs_t.resize(total_components);
  weights_t.resize(total_components);
  // First, object not detected
  for(size_t i = 0; i < num_mixtures; i++) {
    weights_t[i] = (1.0 - p_detect)*weights_tp1[i];
    means_t[i] = means_tp1[i];
    covs_t[i] = covs_tp1[i];
  }
  // Second, detection of objects
  for(size_t i = 0; i < num_observation; i++) {
    // Get the measurements
    measurement obs = observations[i];
    double sum_weight = 0.0;
    for(size_t j = 0; j < num_mixtures; j++) {
      unsigned int indx = (i+1)*num_mixtures+j;
      // Update weight
      weights_t[indx] = p_detect * weights_tp1[j] * normal_eval(obs, h[j], S[j]);
      sum_weight += weights_t[indx];
      // Correct in global coordinates
      means_t[indx] = means_tp1[j] + K[j] *(obs - h[j]);
      // New covariance
      covs_t[indx] = P[j];
    }
    unsigned int birth_prior_indx = num_mixtures + i*(num_mixtures + 1) + num_mixtures;
    if(i < birth_prior_means.size()) {
      sum_weight += birth_prior_weight;
      means_t[birth_prior_indx] = birth_prior_means[i];
      covs_t[birth_prior_indx] = birth_prior_covs[i];
      weights_t[birth_prior_indx] = birth_prior_weight / (clutter_rfs + sum_weight);
    } else {
      // Dummy state with weight 0, just to pre-size the arrays
      means_t[birth_prior_indx] = state::Zero();
      covs_t[birth_prior_indx] = state_mat::Identity();
      weights_t[birth_prior_indx] = 0.0;
    }
    for(size_t j = 0; j < num_mixtures; j++) {
      weights_t[(i+1)*num_mixtures+j] /= (clutter_rfs + sum_weight);
    }
  }
  // Clear birth priors
  clear_birth_prior();
}

template<int dim_state, int dim_measurement, int num_modes>
void DTrackerMJS<dim_state, dim_measurement, num_modes>::prune() {
  for(size_t i = 0; i < weights_t.size(); i++) {
    // Check for this mixture to be consistend
    if(boost::math::isnan(means_t[i]) || boost::math::isnan(weights_t[i])) {
      weights_t.erase(weights_t.begin() + i);
      means_t.erase(means_t.begin() + i);
      covs_t.erase(covs_t.begin() + i);
      i--;
    }
  }
  std::vector<unsigned int> I;
  std::vector<double> final_weights;
  states final_means;
  state_mats final_covs;
  // Only used with mixtures BetaGaussian
  std::vector<double> final_a, final_b;
  num_targets = 0;
  // This is horrible, but when I templated DTracker it stoped working properly
  std::vector<typename Eigen::LLT<state_mat>::Traits::MatrixL,
              Eigen::aligned_allocator<typename Eigen::LLT<state_mat>::Traits::MatrixL> > els_matrices;
  // Get the initial values for I
  for(unsigned int i = 0; i < weights_t.size(); i++) {
    els_matrices.push_back(covs_t[i].llt().matrixL());
    if(!boost::math::isnan(weights_t[i]) && weights_t[i] > truncate_threshold) {
      I.push_back(i);
    }
  }
  if(I.size() == 0) {
    means_tp1.resize(0);
    covs_tp1.resize(0);
    weights_tp1.resize(0);
    return;
  }
  do {
    // Find the components with the highest weight in I
    double max_weight = -1;
    unsigned int max_indx = 0;
    size_t remv_indx = 0;
    for(unsigned int i = 0; i < I.size(); i++) {
      const double &weight = weights_t[I[i]];
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
    state average_state = weights_t[max_indx] * means_t[max_indx];
    double total_weight = weights_t[max_indx];
    const state &max_state = means_t[max_indx];
    for(unsigned int i = 0; i < I.size(); i++) {
      state dx = means_t[I[i]] - max_state;
      state v = els_matrices[I[i]].solve(dx);
      const double sq_dist =  v.squaredNorm();
      if(sq_dist < merge_gauss_threshold) {
        indices.push_back(I[i]);
        total_weight += weights_t[I[i]];
        average_state += weights_t[I[i]] * means_t[I[i]];
      }
    }
    // Generate the new component
    double merged_weight = total_weight;
    state merged_mean = average_state / total_weight;
    state dx = means_t[max_indx] - merged_mean;
    state_mat merged_cov = weights_t[max_indx] * (covs_t[max_indx] + dx*dx.transpose());
    for(unsigned int i = 0; i < indices.size(); i++) {
      dx = means_t[indices[i]] - merged_mean;
      merged_cov += weights_t[indices[i]] * (covs_t[indices[i]] + dx*dx.transpose());
      // Remove the element from I
      std::vector<unsigned int>::iterator it = std::find(I.begin(), I.end(), indices[i]);
      if(it != I.end())
        I.erase(it);
    }
    merged_cov = merged_cov / total_weight;
    //
    // Ugly fix to avoid getting too string components
    if(cap_max_weights && total_weight > 1.0)
      total_weight = 1.0;

    // Check if the weights are big enough
    if(final_weights.size() < max_components) {
      // Add to the final mixture
      final_weights.push_back(merged_weight);
      final_means.push_back(merged_mean);
      final_covs.push_back(merged_cov);
      num_targets += merged_weight;
    } else if(final_weights[final_weights.size()-1] < merged_weight) {
      // Replace to with the final item
      const size_t final_indx = final_weights.size()-1;
      final_weights[final_indx] = merged_weight;
      final_means[final_indx] = merged_mean;
      final_covs[final_indx] = merged_cov;
      num_targets += merged_weight;
    }
  } while (I.size() > 0);
  // Assign the final mixture
  size_t final_size = final_weights.size();
  weights_t.resize(final_size);
  means_t.resize(final_size);
  covs_t.resize(final_size);
  std::copy(final_weights.begin(), final_weights.end(), weights_t.begin());
  std::copy(final_means.begin(), final_means.end(), means_t.begin());
  std::copy(final_covs.begin(), final_covs.end(), covs_t.begin());
}

template<int dim_state, int dim_measurement, int num_modes>
template<typename Container>
void DTrackerMJS<dim_state, dim_measurement, num_modes>::get_state(Container &output, int mode, bool one_per_gaussian) {
  size_t num_targets = std::floor(this->num_targets + 0.5);
  resize(output, num_targets);
  size_t index = 0;
  size_t i = 0;
  resize(output, weights_t.size());
  // The states are the peaks of the GMM
  for(; i < weights_t.size(); i++) {
    if(weights_t[i] > 0.5) {
      if(!one_per_gaussian) {
        // Extract one for each expected number of people in this peak
        int num_copies = std::floor(weights_t[i] + 0.5);
        if(index + num_copies > size(output)) {
          resize(output, index + num_copies);
        }
        for(int j = 0; j < num_copies; j++) {
          assign(output, index, means_t[i]);
          index++;
        }
      } else {
        // Or extract for each peak
        assign(output, index, means_t[i]);
        index++;
      }
    }
  }
  resize(output,index);
/*
  if(index < num_targets && weights_t.size() > 0) {
    ROS_WARN_STREAM("Expected " << num_targets << " targets, but only found " << index << ". Filling the gaps");
    do {
      assign(output, index, means_t[i % weights_t.size()]);
      index++;i++;
    } while(index < num_targets);
  }
*/
}

