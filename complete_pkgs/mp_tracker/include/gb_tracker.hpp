template<int dim_state, int dim_measurement>
GBTracker<dim_state,dim_measurement>::GBTracker():
    components_t(0),
    components_tp1(0),
    probability_var_increase(1.1),
    is_confident_value(0.5),
    is_not_confident_value(0.1),
    gating_threshold(1.0),
    num_frames_missdetected(5),
    min_frames_creation(1),
    is_person_probability(0.6),
    probability_var_extraction(0.011),
    last_id(1),
    use_beta(true),
    use_colour(true) {
  // Don't use the default error handler
  gsl_set_error_handler_off();
}

template<int dim_state, int dim_measurement>
void GBTracker<dim_state, dim_measurement>::predict() {
  size_t num_components = components_t.size();
  observation_likelihood = 0.0;
  components_tp1.resize(num_components);

  for(size_t i = 0; i < num_components; i++) {
    components_tp1[i].id = components_t[i].id;
    components_tp1[i].obs_count = components_t[i].obs_count;
    components_tp1[i].miss_detections_count = components_t[i].miss_detections_count;
    components_tp1[i].mean = F * components_t[i].mean;
    components_tp1[i].cov = F * components_t[i].cov * F.transpose() + Q;
    // Update colour compoents
    if(use_colour)
      update_colour(i);
    // Update the beta components
    if(use_beta)
      update_beta(i);
  }
}

template<int dim_state, int dim_measurement>
void GBTracker<dim_state, dim_measurement>::predict_ut(size_t dim_noise, const update_function &update) {
  size_t num_components = components_t.size();
  observation_likelihood = 0.0;
  components_tp1.resize(num_components);
  const size_t dim_sigma = dim_state + dim_noise;
  for(size_t i = 0; i < num_components; i++) {
    // Setup the sigma points
    std::vector<Eigen::VectorXd> sigma_points;
    std::vector<double> sigma_weights;
    Eigen::VectorXd point = Eigen::VectorXd::Zero(dim_sigma);
    Eigen::MatrixXd Sigma = sigma_f * Eigen::MatrixXd::Identity(dim_sigma, dim_sigma);
    // The error is assumed to be N(0,1) for now
    point.block(0,0,dim_state,1) = components_t[i].mean;
    Sigma.block(0,0,dim_state,dim_state) = components_t[i].cov;
    // Create the sigma points
    unsctd_sampler.get_points(point, Sigma, 0.3, 1.0, sigma_points, sigma_weights);
    // Rebuild mean and covariance of this component
    components_tp1[i].mean = state::Zero();
    components_tp1[i].cov = state_mat::Zero();
    states x_js;
    x_js.resize(sigma_points.size());
    for(size_t j = 0; j < sigma_points.size(); j++) {
      const state &x_j = sigma_points[j].block(0,0,dim_state,1);
      state error = sigma_points[j].block(dim_state,0,dim_noise,1);
      // Run the non-linear update function
      x_js[j] = update(x_j, error);
      components_tp1[i].mean += sigma_weights[j] * x_js[j];
    }
    for(size_t j = 0; j < sigma_points.size(); j++) {
      const state &x_j = x_js[j];
      state dx = components_tp1[i].mean - x_j;
      components_tp1[i].cov += sigma_weights[j] * dx * dx.transpose();
    }
    components_tp1[i].id = components_t[i].id;
    // Update colour compoents
    if(use_colour)
      update_colour(i);
    // Update the beta components
    if(use_beta)
      update_beta(i);
  }
}

template<int dim_state, int dim_measurement>
void GBTracker<dim_state, dim_measurement>::update_beta(size_t i) {
  double var_beta = components_t[i].a * components_t[i].b
                    / (std::pow(components_t[i].a + components_t[i].b, 2)*(components_t[i].a + components_t[i].b + 1.0));
  double exp_beta = components_t[i].a / (components_t[i].a + components_t[i].b);
  components_tp1[i].a = (exp_beta * (1.0 - exp_beta)) / (probability_var_increase * var_beta) * exp_beta;
  components_tp1[i].b = (exp_beta * (1.0 - exp_beta)) / (probability_var_increase * var_beta) * (1.0 - exp_beta);
  if(components_tp1[i].a <= 0.0 || components_tp1[i].b <= 0.0) {
#ifndef NOROS
    ROS_WARN_STREAM("a or b components set to 0, a:" << components_tp1[i].a
                     << " and b:" << components_tp1[i].b
                     << " original a: " << components_t[i].a
                     << " and original b:" <<  components_t[i].b);
#endif
    components_tp1[i].a = components_t[i].a / (components_t[i].a + components_t[i].b) + 0.001;
    components_tp1[i].b = components_t[i].b / (components_t[i].a + components_t[i].b) + 0.001;
  }
/*
  components_tp1[i].a = components_t[i].a; // / (components_t[i].a + components_t[i].b);
  components_tp1[i].b = components_t[i].b + 1; // / (components_t[i].a + components_t[i].b);
*/
}

template<int dim_state, int dim_measurement>
void GBTracker<dim_state, dim_measurement>::update_colour(size_t i) {
  // The colour components are treated as independet KF/Gaussian distributions
  components_tp1[i].colour_projection_mean = components_t[i].colour_projection_mean;
  components_tp1[i].colour_projection_std = components_t[i].colour_projection_std
                                              + colour_std_change * colour_projection_type::Ones(num_colour_components);
}

template<int dim_state, int dim_measurement>
void GBTracker<dim_state, dim_measurement>::correct(
      const measurements &observations,
      const std::vector<double> &confidences,
      const colour_projections_type &colour_projections) {
  const size_t num_observation = observations.size();
  const size_t num_components = components_tp1.size();
  // Clear this variable to star accumilating the log likelihood
  log_observation_likelihood = 0.0;

  // This variable to hold precalculated expected values
  measurements h(num_components);
  measurement_mats S(num_components);
  state_measurement_mats K(num_components);
  // Cost matrix
  Eigen::MatrixXd distances(num_observation, num_components);
  Eigen::MatrixXd pos_like(num_observation, num_components);
//  Eigen::MatrixXd hog_like(num_observation, num_components);
  Eigen::MatrixXd col_like(num_observation, num_components);
  // Get the transformation between frames
  const state zero = state::Zero();
  for(size_t i = 0; i < num_components; i++) {
    // Predict the observation
    h[i] = H * components_tp1[i].mean;
    S[i] = R + H * components_tp1[i].cov * H.transpose();
    K[i] = S[i].transpose().ldlt().solve(H * components_tp1[i].cov.transpose()).transpose();
    for(size_t j = 0; j < num_observation; j++) {
      // The cost of assigning this j-th observation to the
      // i-th filter
      measurement dz = observations[j]-h[i];
//      const double &p = confidences[j];
      const colour_projection_type colour = (use_colour ? colour_projections[j] : colour_projection_type::Zero(1));
      distances(j,i) = dz.norm();
      if(distances(j,i) > gating_threshold  * (components_tp1[i].miss_detections_count+1)) {
        distances(j,i) = 1000000.0;
        pos_like(j,i) = 1000000.0;
//        hog_like(j,i) = 1000000.0;
        col_like(j,i) = 1000000.0;
      }
      else {
        pos_like(j,i) = -log_normal_eval(K[i]*dz, zero, components_tp1[i].cov);
//        hog_like(j,i) = -log_beta_eval(p, components_tp1[i].a, components_tp1[i].b);
        if(use_colour)
          col_like(j,i) = -log_normal_eval(colour, components_tp1[i].colour_projection_mean, components_tp1[i].colour_projection_std);
      }
    }
  }

  Eigen::MatrixXd assignments = pos_like;
  if(use_colour)
    assignments += col_like / num_colour_components;// + hog_like;
  munkres::Munkres<Eigen::MatrixXd, Eigen::MatrixXi> hungarian;
  hungarian.solve(assignments);

  // Update
  components_t.resize(0);
  components_t.reserve(std::max(num_components, num_observation));

  // Vector with the updated filters
  std::vector<size_t> updated_filters;

  // Iterate every observation
  for(size_t i = 0; i < num_observation; i++) {
    // Get the measurements
    const measurement obs = observations[i];
    const double confidence = (use_beta ? confidences[i] : 1);
    const colour_projection_type colour = (use_colour ? colour_projections[i] : colour_projection_type::Zero(1));

    // Search for the asigned filter to the observation
    const size_t invalid = num_components + 1;
    size_t asigned_filter = invalid;
    for(size_t j = 0; j < num_components; j++) {
      if(assignments(i, j) == 0 && distances(i, j) <= gating_threshold * (components_tp1[j].miss_detections_count+1)) {
        asigned_filter = j;
        break;
      }
    }

    // If a measurement if correctly assigned correct the corresponding filter
    if(asigned_filter < invalid) {
      tracker_component new_component;
      // Keep id
      new_component.id = components_tp1[asigned_filter].id;
      // State correction
      new_component.mean = components_tp1[asigned_filter].mean
                           + K[asigned_filter] *(obs - h[asigned_filter]);
      // New covariance
      new_component.cov = (state_mat::Identity() - K[asigned_filter] * H) * components_tp1[asigned_filter].cov;

#if 0
      // Correct the colour projection component
      colour_projection_type Kc;
      Kc = components_tp1[asigned_filter].colour_projection_std
           * (components_tp1[asigned_filter].colour_projection_std + colour_observation_std).inverse();
      colour_projection_type delta_colour =  colour - components_tp1[asigned_filter].colour_projection_mean;
      // New colour mean
      new_component.colour_projection_mean = components_tp1[asigned_filter].colour_projection_mean + Kc * delta_colour; 
      // New colour std
      new_component.colour_projection_std = (1.0 - Kc) * components_tp1[asigned_filter].colour_projection_std;
#else
      if(use_colour) {
        colour_projection_type normalisation;
        normalisation = (components_tp1[asigned_filter].colour_projection_std + colour_observation_std).inverse();
        new_component.colour_projection_mean = colour_observation_std * normalisation * components_tp1[asigned_filter].colour_projection_mean + components_tp1[asigned_filter].colour_projection_std * normalisation * colour;
        new_component.colour_projection_std = (components_tp1[asigned_filter].colour_projection_std.inverse() + 1.0 / colour_observation_std).inverse();
      }
#endif
      if(use_beta) {
        // Correct the beta parameters
        if(confidence > is_confident_value) {
          new_component.a = components_tp1[asigned_filter].a + 1.0;
          new_component.b = components_tp1[asigned_filter].b;
        } else if(confidence < is_not_confident_value) {
          new_component.a = components_tp1[asigned_filter].a;
          new_component.b = components_tp1[asigned_filter].b + 1.0;
        } else {
          new_component.a = components_tp1[asigned_filter].a;
          new_component.b = components_tp1[asigned_filter].b;
        }
      }
      // Countign variables
      new_component.miss_detections_count = 0;
      new_component.obs_count = components_tp1[asigned_filter].obs_count + 1;

      // Append the components
      components_t.push_back(new_component);
      // Save the id of the updated filter
      updated_filters.push_back(asigned_filter);
    }
    // if a measurement if not assigned, create a new tracking filter with 1 count
    else {
      int cat = (confidence >  is_confident_value ? 1 : (confidence < is_not_confident_value ? -1 : 0));
      tracker_component new_component = state_creator(obs, cat, colour);
      new_component.id = last_id++;
      new_component.obs_count = 1;
      new_component.miss_detections_count = 0;
      // Append the components
      components_t.push_back(new_component);
    }
  }

  // Increment the count fo miss detected filters
  for(size_t i = 0; i < num_components; i++) {
    // If the indice if the filter is not found on the updated filter list
    if(std::find(updated_filters.begin(), updated_filters.end(), i) == updated_filters.end()) {
      // Increment the miss detections
      components_tp1[i].miss_detections_count++;
      // Increment the likelihood of not being a person
      if(use_beta)
        components_tp1[i].b += 0.5;
      // If the number of miss detections is lower than a threshold append this filter
      if(components_tp1[i].miss_detections_count < num_frames_missdetected) {
        components_t.push_back(components_tp1[i]);
      }
      else {
      }
    }
    else {
    }
  }

  // The observation likelihood
  observation_likelihood = std::exp(log_observation_likelihood);
}

/*
template<int dim_state, int dim_measurement>
void GBTracker<dim_state, dim_measurement>::correct_ut(const measurements &meas, size_t dim_noise, const measurement_function &observe) {
  throw std::runtime_error("Not implemented");
}

template<int dim_state, int dim_measurement>
void GBTracker<dim_state, dim_measurement>::correct_mc(const state_probability_function &state_likelihood) {
  throw std::runtime_error("Not implemented");
}
*/

template<int dim_state, int dim_measurement>
void GBTracker<dim_state, dim_measurement>::get_state(pcl::PointCloud<icl::PointWithVelocity>::Ptr &output) {
  size_t index = 0;
  size_t i = 0;
  output->points.resize(components_t.size());
  // The states are the peaks of the GMM
  for(; i < components_t.size(); i++) {
    if(components_t[i].obs_count >= min_frames_creation) {
      if(use_beta) {
#if 0
        int status;
        gsl_sf_result result;
        status = gsl_sf_beta_inc_e(components_t[i].a, components_t[i].b, is_person_probability, &result);
        double prob_is_person_greater_than;
        if(status != GSL_SUCCESS) {
/*
            ROS_WARN_STREAM("Error calculating I_x(a,b)) with a:" << components_t[i].a
                             << ", b:" << components_t[i].b
                             << " and x: " << is_person_probability
                             << " status: " << status
                             << " results: " << result.val
                             << " error: " << result.err);
*/
          // This is hack to handle this situation
          if(components_t[i].a > 1)
            prob_is_person_greater_than = (components_t[i].b > (1 - is_person_probability) * components_t[i].a ? 0.0 : 1.0);
          else
            prob_is_person_greater_than = 0.0;
        }
        else {
          prob_is_person_greater_than = 1.0 - result.val;
        }

        if(prob_is_person_greater_than < certainty_person_extraction)
            continue;
#else
        const double expected_prob = components_t[i].a / (components_t[i].a + components_t[i].b);
        const double var_prob = std::exp(std::log(components_t[i].a)
                                         + std::log(components_t[i].b)
                                         - 2.0*std::log(components_t[i].a + components_t[i].b)
                                         - std::log(components_t[i].a + components_t[i].b + 1.0));
        if(expected_prob < is_person_probability || var_prob > probability_var_extraction)
          continue;
#endif
      }
      // Copy the data to the pointcloud structure
      output->points[index].x = components_t[i].mean[0];
      output->points[index].y = components_t[i].mean[1];
      output->points[index].z = 0.0;
      output->points[index].velocity_x = components_t[i].mean[2];
      output->points[index].velocity_y = components_t[i].mean[3];
      output->points[index].velocity_z = 0.0;
      output->points[index].id = components_t[i].id;
      index++;
    }
  }
  output->points.resize(index);
}
