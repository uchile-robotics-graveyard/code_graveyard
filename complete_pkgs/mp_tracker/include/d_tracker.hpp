template<int dim_state, int dim_measurement>
DTracker<dim_state,dim_measurement>::DTracker(const DTracker<dim_state,dim_measurement> &dt):
    use_beta_mixture(dt.use_beta_mixture),
    use_optimal_beta_merge(dt.use_optimal_beta_merge),
    cap_max_weights(dt.cap_max_weights),
    components_t(dt.components_t.size()),
    components_tp1(dt.components_tp1.size()),
    last_id(dt.last_id),
    python_loaded(dt.python_loaded) {
  // Don't use the default error handler
  gsl_set_error_handler_off();

  num_targets = dt.num_targets;
  // Just in case, some default parameters just in case
  p_detect = dt.p_detect;
  clutter_rfs = dt.clutter_rfs;
  truncate_threshold = dt.truncate_threshold;
  merge_gauss_threshold = dt.merge_gauss_threshold;
  merge_beta_threshold = dt.merge_beta_threshold;
  birth_prior_weights.resize(dt.birth_prior_weights.size());
  std::copy(dt.birth_prior_weights.begin(), dt.birth_prior_weights.end(), birth_prior_weights.begin());
  // Update process
  F = dt.F; Q = dt.Q; T = dt.T;
  sigma_f = dt.sigma_f;
  // Observation process
  H = dt.H; R = dt.R;
  sigma_h = dt.sigma_h;
  // copy into the arrays holing the different gaussian/beta parameters
  std::copy(dt.components_t.begin(), dt.components_t.end(), components_t.begin());
  // Keep copying
  std::copy(dt.components_tp1.begin(), dt.components_tp1.end(), components_tp1.begin());
  // Finaly the likelihoods
  observation_likelihood = dt.observation_likelihood;
  log_observation_likelihood = dt.log_observation_likelihood;

  priv = ros::NodeHandle("~");
  _mov_pub = priv.advertise<geometry_msgs::PoseStamped>("/bender/nav/goal_server/goal", 1);
  obj_pub = priv.advertise<visualization_msgs::Marker>( "object_marker", 0 );
  _transf_serv = priv.serviceClient<bender_srvs::Transformer>("/bender/tf/simple_pose_transformer/transform",1);
  pose_bender = priv.subscribe("/bender/nav/robot_pose_publisher/pose", 1, &DTracker::PoseCallback,this);

  publish_state = false;
  trPerson.id=-1;
  last_time = time(NULL); 
}

template<int dim_state, int dim_measurement>
DTracker<dim_state,dim_measurement>::DTracker(bool use_beta, bool use_optimal_merge, bool cap_max_weights_):
    use_beta_mixture(use_beta),
    use_optimal_beta_merge(use_optimal_beta_merge),
    cap_max_weights(cap_max_weights_),
    components_t(0),
    components_tp1(0),
    num_targets(0),
    last_id(1),
    python_loaded(false) {
  // Don't use the default error handler
  gsl_set_error_handler_off();

  num_targets = 0;
  p_detect = 0.75;
  // Just in case, some default parameters just in case
  clutter_rfs = 1E-5;
  truncate_threshold = 1E-12;
  merge_gauss_threshold = 1E-4;
  merge_beta_threshold = 0.15;
  certainty_person_extraction = 0.2;
  is_person_probability = 0.5;

  priv = ros::NodeHandle("~");
  _mov_pub = priv.advertise<geometry_msgs::PoseStamped>("/bender/nav/goal_server/goal", 1);
  obj_pub = priv.advertise<visualization_msgs::Marker>( "object_marker", 0 );
  _transf_serv = priv.serviceClient<bender_srvs::Transformer>("/bender/tf/simple_pose_transformer/transform",1);
  // _move_approach_serv = priv.serviceClient<bender_srvs::NavGoal>("/bender/nav/goal_server/approach",1);
   pose_bender = priv.subscribe("/bender/nav/robot_pose_publisher/pose", 1, &DTracker::PoseCallback,this);

  publish_state = false;
  trPerson.id=-1;
  last_time = time(NULL); 
    
}

template<int dim_state, int dim_measurement>
void DTracker<dim_state, dim_measurement>::load_debug_plugin(const std::string &python_file) {
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

template<int dim_state, int dim_measurement>
void DTracker<dim_state, dim_measurement>::PoseCallback(geometry_msgs::PoseStamped in){
  PoseBender = in;
}



template<int dim_state, int dim_measurement>
void DTracker<dim_state, dim_measurement>::debug_plot(bool t, double time, const Eigen::Matrix4d &T,  const pcl::PointCloud<pcl::PointXYZ>::Ptr &laser_pc, const measurements &meas) {
  static size_t frame_num = 0;

  if(!python_loaded)
    return;

  // Aquire python's GIL
  PyEval_RestoreThread(gil_save);

  python::list weights, as, bs, means, covs, laser, obs;
  // Fill the arrays
  if(birth_prior_means.size() > 0) {
    for(size_t i = 0; i < birth_prior_means.size(); i++) {
      // Create the a mean list and append to the means list
      python::list mean;
      for(size_t j = 0; j < dim_state; j++) {
        mean.append(birth_prior_means[i](j));
      }
      means.append(mean);
      // Create a covariance double list and append to the covariances list
      python::list cov;
      for(size_t j = 0; j < dim_state; j++) {
        python::list row;
        for(size_t k = 0; k < dim_state; k++) {
          row.append(birth_prior_covs[i](j, k));
        }
        cov.append(row);
      }
      covs.append(cov);
      // Append the weights
      weights.append(1.0);
    }
  }
  else if(t) { // do time state_t
    for(size_t i = 0; i < components_t.size(); i++) {
      // Create the a mean list and append to the means list
      python::list mean;
      for(size_t j = 0; j < dim_state; j++) {
        mean.append(components_t[i].mean(j));
      }
      means.append(mean);
      // Create a covariance double list and append to the covariances list
      python::list cov;
      for(size_t j = 0; j < dim_state; j++) {
        python::list row;
        for(size_t k = 0; k < dim_state; k++) {
          row.append(components_t[i].cov(j, k));
        }
        cov.append(row);
      }
      covs.append(cov);
      // Append the weights
      weights.append(components_t[i].weight);
      if(use_beta_mixture) {
        as.append(components_t[i].a);
        bs.append(components_t[i].b);
      }
    }
  }
  else { // do time state_t+1
    for(size_t i = 0; i < components_tp1.size(); i++) {
      python::list mean;
      for(size_t j = 0; j < dim_state; j++) {
        mean.append(components_tp1[i].mean(j));
      }
      means.append(mean);
      python::list cov;
      for(size_t j = 0; j < dim_state; j++) {
        python::list row;
        for(size_t k = 0; k < dim_state; k++) {
          row.append(components_tp1[i].cov(j, k));
        }
        cov.append(row);
      }
      covs.append(cov);
      weights.append(components_tp1[i].weight);
      if(use_beta_mixture) {
        as.append(components_tp1[i].a);
        bs.append(components_tp1[i].b);
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
  // Create the transformation matrix
  python::list trans;
  for(size_t i = 0; i < 4; i++) {
      python::list row;
      for(size_t j = 0; j < 4; j++) {
        row.append(T(i,j));
      }
      trans.append(row); 
  }
  // Call the plotting function
  try {
    if(use_beta_mixture) {
      python_plotter(frame_num++, time, trans, weights, means, covs, laser, obs, as, bs);
    }
    else {
      python_plotter(frame_num++, time, trans, weights, means, covs, laser, obs, python::object(), python::object());
    }
  } catch(...) {
    PyErr_Print();
    PyErr_Clear();
  }
  // Relesae GIL
  gil_save = PyEval_SaveThread();
}

template<int dim_state, int dim_measurement>
DTracker<dim_state, dim_measurement>::~DTracker() {
  if(python_loaded)
    // Aquire python's GIL
    PyEval_RestoreThread(gil_save);
    // Finalise python iterpreter
    Py_Finalize();
}

template<int dim_state, int dim_measurement>
void DTracker<dim_state, dim_measurement>::predict() {
  size_t num_components = components_t.size();
  observation_likelihood = 0.0;
  components_tp1.resize(num_components);
  for(size_t i = 0; i < num_components; i++) {
    components_tp1[i].weight = p_survival * components_t[i].weight;
    components_tp1[i].mean = F * components_t[i].mean;
    components_tp1[i].cov = F * components_t[i].cov * F.transpose() + Q;
    if(use_beta_mixture) {
      double var_beta = components_t[i].a * components_t[i].b
                        / (std::pow(components_t[i].a + components_t[i].b, 2)*(components_t[i].a + components_t[i].b + 1.0));
      double exp_beta = components_t[i].a / (components_t[i].a + components_t[i].b);
      const double k = 1.1;
      components_tp1[i].a = (exp_beta * (1.0 - exp_beta)) / (k * var_beta) * exp_beta;
      components_tp1[i].b = (exp_beta * (1.0 - exp_beta)) / (k * var_beta) * (1.0 - exp_beta);
      if(components_tp1[i].a <= 0.0 || components_tp1[i].b <= 0.0) {
        ROS_WARN_STREAM("a or b components set to 0, a:" << components_tp1[i].a
                         << " and b:" << components_tp1[i].b
                         << " original a: " << components_t[i].a
                         << " and original b:" <<  components_t[i].b);
        components_tp1[i].a = components_t[i].a / (components_t[i].a + components_t[i].b) + 0.001;
        components_tp1[i].b = components_t[i].b / (components_t[i].a + components_t[i].b) + 0.001;
      }
/*
      components_tp1[i].a = components_t[i].a; // / (components_t[i].a + components_t[i].b);
      components_tp1[i].b = components_t[i].b + 1; // / (components_t[i].a + components_t[i].b);
*/
    }
  }
  // Update for the target id estimation
  for(typename ids_storage::iterator it = targets_id.begin(); it != targets_id.end(); it++) {
    it->second = F * it->second;
  }
}

template<int dim_state, int dim_measurement>
void DTracker<dim_state, dim_measurement>::predict_ut(size_t dim_noise, const update_function &update) {
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
    components_tp1[i].weight = p_survival * components_t[i].weight;

    if(use_beta_mixture) {
      double var_beta = components_t[i].a * components_t[i].b
                        / (std::pow(components_t[i].a + components_t[i].b, 2)*(components_t[i].a + components_t[i].b + 1.0));
      double exp_beta = components_t[i].a / (components_t[i].a + components_t[i].b);
      const double k = 1.1;
      components_tp1[i].a = (exp_beta * (1.0 - exp_beta)) / (k * var_beta) * exp_beta;
      components_tp1[i].b = (exp_beta * (1.0 - exp_beta)) / (k * var_beta) * (1.0 - exp_beta);
      if(components_tp1[i].a <= 0.0 || components_tp1[i].b <= 0.0) {
        ROS_WARN_STREAM("a or b components set to 0, a:" << components_tp1[i].a
                         << " and b:" << components_tp1[i].b
                         << " original a: " << components_t[i].a
                         << " and original b:" <<  components_t[i].b);
        components_tp1[i].a = components_t[i].a / (components_t[i].a + components_t[i].b) + 0.001;
        components_tp1[i].b = components_t[i].b / (components_t[i].a + components_t[i].b) + 0.001;
      }
/*
      components_tp1[i].a = components_t[i].a; // / (components_t[i].a + components_t[i].b);
      components_tp1[i].b = components_t[i].b + 1; // / (components_t[i].a + components_t[i].b);
*/
    }
  }
}

template<int dim_state, int dim_measurement>
void DTracker<dim_state, dim_measurement>::correct_ut(const measurements &meas, size_t dim_noise, const measurement_function &observe, const std::vector<int> &categories) {
  size_t num_mixtures = components_tp1.size();
  size_t num_observation = meas.size();
  // Clear this variable to star accumilating the log likelihood
  log_observation_likelihood = 0.0;

  measurements n(num_mixtures);
  measurement_mats S(num_mixtures);
  state_measurement_mats K(num_mixtures);
  state_measurement_mats G(num_mixtures);
  state_mats P(num_mixtures);
  std::vector<double> prob_detect(num_mixtures);

  const size_t dim_sigma = dim_state + dim_noise;
  for(size_t i = 0; i < num_mixtures; i++) {
    // Set up the sigma points
    std::vector<Eigen::VectorXd> sigma_points;
    std::vector<double> sigma_weights;
    Eigen::VectorXd point = Eigen::VectorXd::Zero(dim_sigma);
    Eigen::MatrixXd Sigma = sigma_h * Eigen::MatrixXd::Identity(dim_sigma, dim_sigma);
    point.block(0,0,dim_state,1) = components_tp1[i].mean;
    Sigma.block(0,0,dim_state,dim_state) = components_tp1[i].cov;
    // Create the sigma points
    unsctd_sampler.get_points(point, Sigma, 0.3, 1.0, sigma_points, sigma_weights);
    // Assumes error dh/dx ~ 1
    n[i] = measurement::Zero();
    S[i] = measurement_mat::Zero();
    K[i] = state_measurement_mat::Zero();
    G[i] = state_measurement_mat::Zero();
    measurements z(sigma_points.size());

    for(size_t j = 0; j < sigma_points.size(); j++) {
      const state &x_j = sigma_points[j].block(0,0,dim_state,1);
      measurement error = sigma_points[j].block(dim_state,0,dim_noise,1);
      z[j] = observe(x_j, error);
      n[i] += sigma_weights[j] * z[j];
    }
    for(size_t j = 0; j < sigma_points.size(); j++) {
      const state &x_j = sigma_points[j].block(0,0,dim_state,1);
      const double &weight_j = sigma_weights[j];
      const measurement dz = z[j] - n[i];
      S[i] += weight_j * dz * dz.transpose();
      G[i] += weight_j * (x_j - components_tp1[i].mean) * dz.transpose();
    }
    //K[i] = G[i] * S[i].inverse();
    K[i] = S[i].transpose().ldlt().solve(G[i].transpose()).transpose();
    P[i] = components_tp1[i].cov -  K[i] * G[i].transpose();
    // Constant probability of detection for the time being
    prob_detect[i] = p_detect;
  }

  // Generate the final array
  size_t total_components = num_mixtures +  num_observation * (num_mixtures + 1);
  components_t.resize(total_components);

  // First, object not detected
  for(size_t i = 0; i < num_mixtures; i++) {
    components_t[i].weight = (1.0 - prob_detect[i])*components_tp1[i].weight;
    components_t[i].mean = components_tp1[i].mean;
    components_t[i].cov = components_tp1[i].cov;
    // Calculate the log observation likelihood
    log_observation_likelihood -= prob_detect[i] * components_tp1[i].weight;
    if(use_beta_mixture) {
      components_t[i].a = components_tp1[i].a;
      components_t[i].b = components_tp1[i].b;
    }
  }
  // Observed components
  for(size_t l = 0; l < num_observation; l++) {
    double sum_weight = 0.0;
    measurement obs = meas[l];
    for(size_t j = 0; j < num_mixtures; j++) {
      unsigned int indx = num_mixtures + l*(num_mixtures + 1)+j;
      components_t[indx].weight = prob_detect[j] * components_tp1[j].weight * normal_eval(obs, n[j], S[j]);
      if(use_beta_mixture) {
        components_t[indx].a = components_tp1[j].a;
        components_t[indx].b = components_tp1[j].b;
      }
      sum_weight += components_t[indx].weight;
      components_t[indx].mean = components_tp1[j].mean + K[j] * (obs - n[j]);
      components_t[indx].cov = P[j];
    }
    unsigned int birth_prior_indx = num_mixtures + l*(num_mixtures + 1) + num_mixtures;
    if(l < birth_prior_means.size()) {
      sum_weight += birth_prior_weights[l];
      components_t[birth_prior_indx].mean = birth_prior_means[l];
      components_t[birth_prior_indx].cov = birth_prior_covs[l];
      components_t[birth_prior_indx].weight = birth_prior_weights[l] / (clutter_rfs + sum_weight);
    }
/*
    else {
      // Dummy state with weight 0, just to pre-size the arrays
      components_t[birth_prior_indx].mean = state::Zero();
      components_t[birth_prior_indx].cov = state_mat::Identity();
      components_t[birth_prior_indx].weight = 0.0;
    }
*/
    if(use_beta_mixture) {
      components_t[birth_prior_indx].a = 2.0;
      components_t[birth_prior_indx].b = 2.0;
    }
    for(size_t j = 0; j < num_mixtures; j++) {
      unsigned int indx = num_mixtures + l*(num_mixtures + 1)+j;
      components_t[indx].weight /= (clutter_rfs + sum_weight);
    }
    // The log-likelihood
    log_observation_likelihood += std::log(clutter_rfs + sum_weight);
  }
  // The observation likelihood
  observation_likelihood = std::exp(log_observation_likelihood);
  // Clear birth priors
  clear_birth_prior();
}

template<int dim_state, int dim_measurement>
void DTracker<dim_state, dim_measurement>::correct(const measurements &observations, const std::vector<int> &categories) {
  const size_t num_observation = observations.size();
  const size_t num_mixtures = components_t.size();
  const size_t num_birth = birth_prior_means.size();
  // Clear this variable to star accumilating the log likelihood
  log_observation_likelihood = 0.0;

  // Construct update components
  measurements h(num_mixtures);
  measurement_mats S(num_mixtures);
  state_measurement_mats K(num_mixtures);
  state_mats P(num_mixtures);
  std::vector<double> prob_detect(num_mixtures);

  // Get the transformation between frames
  for(size_t i = 0; i < num_mixtures; i++) {
    // Predict the observation
    h[i] = H * components_tp1[i].mean;
    S[i] = R + H * components_tp1[i].cov * H.transpose();
    //K[i] = components_tp1[i].cov * H.transpose() * S[i].inverse();
    K[i] = S[i].transpose().ldlt().solve(H * components_tp1[i].cov.transpose()).transpose();
    P[i] = (state_mat::Identity() - K[i] * H) * components_tp1[i].cov;
    // Constant probability of detection for now
    prob_detect[i] = p_detect;
  }
  // Update
  const size_t total_components = num_mixtures * (num_observation + 1) + num_birth;
  components_t.resize(total_components);
  // First, object not detected
  std::vector<double> beta_a1b, beta_ab1;
  for(size_t i = 0; i < num_mixtures; i++) {
    components_t[i].weight = (1.0 - prob_detect[i]) * components_tp1[i].weight;
    components_t[i].mean = components_tp1[i].mean;
    components_t[i].cov = components_tp1[i].cov;
    // Calculate the log observation likelihood
    log_observation_likelihood -= prob_detect[i] * components_tp1[i].weight;
    if(use_beta_mixture) {
/*
      gsl_sf_result res_a1b, res_ab1, res_ab;
      int status;
      status = gsl_sf_lnbeta_e(components_tp1[i].a, components_tp1[i].b, &res_ab);
      if(status != GSL_SUCCESS) {
        ROS_ERROR_STREAM("Error calculating ln(Beta(a,b)) with a:" << components_tp1[i].a << " and b:" << components_tp1[i].b);
        throw std::runtime_error("Problem calculating ln(Beta(a,b))");
      }
      status = gsl_sf_lnbeta_e(components_tp1[i].a + 1, components_tp1[i].b, &res_a1b);
      if(status != GSL_SUCCESS) {
        ROS_ERROR_STREAM("Error calculating ln(Beta(a+1,b)) with a:" << components_tp1[i].a << " and b:" << components_tp1[i].b);
        throw std::runtime_error("Problem calculating ln(Beta(a+1,b))");
      }
      status = gsl_sf_lnbeta_e(components_tp1[i].a, components_tp1[i].b + 1, &res_ab1);
      if(status != GSL_SUCCESS) {
        ROS_ERROR_STREAM("Error calculating ln(Beta(a,b+1)) with a:" << components_tp1[i].a << " and b:" << components_tp1[i].b);
        throw std::runtime_error("Problem calculating ln(Beta(a,b+1))");
      }
      // Store the values
      beta_a1b.push_back(std::exp(res_a1b.val - res_ab.val));
      beta_ab1.push_back(std::exp(res_ab1.val - res_ab.val));
*/
      // Store the values
      beta_a1b.push_back(components_tp1[i].a / (components_tp1[i].a + components_tp1[i].b));
      beta_ab1.push_back(components_tp1[i].b / (components_tp1[i].a + components_tp1[i].b));
    }
  }
  // Second, detection of objects
  for(size_t i = 0; i < num_observation; i++) {
    // Get the measurements
    measurement obs = observations[i];
    int obs_class = (!use_beta_mixture? 0 : categories[i]);
    double sum_weight = 0.0;
    for(size_t j = 0; j < num_mixtures; j++) {
      unsigned int indx = (i+1)*num_mixtures+j;
      // Update weight
      components_t[indx].weight = prob_detect[j] * components_tp1[j].weight * normal_eval(obs, h[j], S[j]);
      if(use_beta_mixture) {
        if(obs_class > 0) {
          components_t[indx].weight *= beta_a1b[j];
          components_t[indx].a = components_tp1[j].a + 1.0;
          components_t[indx].b = components_tp1[j].b;
        } else if(obs_class < 0) {
          components_t[indx].weight *= beta_ab1[j];
          components_t[indx].a = components_tp1[j].a;
          components_t[indx].b = components_tp1[j].b + 1.0;
        } else {
          components_t[indx].a = components_tp1[j].a;
          components_t[indx].b = components_tp1[j].b;
        }
      }
      sum_weight += components_t[indx].weight;
      // Correct in global coordinates
      components_t[indx].mean = components_tp1[j].mean + K[j] *(obs - h[j]);
      // New covariance
      components_t[indx].cov = P[j];
    }
    unsigned int birth_prior_indx = total_components - i - 1;
    if(i < birth_prior_means.size()) {
      sum_weight += birth_prior_weights[i];
      components_t[birth_prior_indx].mean = birth_prior_means[i];
      components_t[birth_prior_indx].cov = birth_prior_covs[i];
      components_t[birth_prior_indx].weight = birth_prior_weights[i] / (clutter_rfs + sum_weight);
      if(use_beta_mixture) {
        components_t[birth_prior_indx].a = 8.0;
        components_t[birth_prior_indx].b = 2.0;
      }
    }
/*
    else {
      // Dummy state with weight 0, just to pre-size the arrays
      components_t[birth_prior_indx].mean = state::Zero();
      components_t[birth_prior_indx].cov = state_mat::Identity();
      components_t[birth_prior_indx].weight = 0.0;
    }
*/
    for(size_t j = 0; j < num_mixtures; j++) {
      components_t[(i+1)*num_mixtures+j].weight /= (clutter_rfs + sum_weight);
    }
    // The log-likelihood
    log_observation_likelihood += std::log(clutter_rfs + sum_weight);
  }
  // The observation likelihood
  observation_likelihood = std::exp(log_observation_likelihood);
  // Clear birth priors
  clear_birth_prior();
}

template<int dim_state, int dim_measurement>
void DTracker<dim_state, dim_measurement>::correct_mc(const state_probability_function &state_likelihood) {
  // Sample from the mixture of gaussians
  std::sort(components_tp1.begin(), components_tp1.end());
  std::reverse(components_tp1.begin(), components_tp1.end());
  double total_weight = 0;
  for(size_t i = 0; i < components_tp1.size(); i++) total_weight += components_tp1[i].weight;
  // 30 particles per dimention per number of people
  size_t num_samples = std::max((int)(30*5*total_weight), 200);
  typename _particle<dim_state>::particles particles(num_samples);
  _sample_from_mixture(components_tp1, num_samples, particles);

  double weight = 0.0;
  size_t valid_likelihoods = 0;
  // Evaluate the likelihood function for each sample
  for(size_t i = 0; i < num_samples; i++) {
    double particle_likelihood = state_likelihood(particles[i].state);
    if(particle_likelihood > 0) valid_likelihoods++;
    particles[i].weight = particles[i].weight * particle_likelihood;
    weight += particles[i].weight;
  }
  if(weight == 0 || valid_likelihoods < 5) {
    components_t.resize(0);
    return;
  }
  // Resample particles to get uniform weights
  std::sort(particles.begin(), particles.end());
  std::reverse(particles.begin(), particles.end());
  typename _particle<dim_state>::particles resampled(num_samples);
  _sample_from_samples(particles, num_samples, resampled);

#if 1
  // Use EM-GMM to recreate the gaussian mixture
  size_t num_components = (total_weight < 0.5 ? 1.0 : std::floor(total_weight + 0.5));
  size_t num_models = 5;
  std::vector<cv::EM> models(num_models);
  double best_complexity = std::numeric_limits<double>::max();
  size_t final_mixture = 0;
  size_t model_indx = 0;
  // Create the OpenCV samples
  cv::Mat samples(num_samples, dim_state, CV_64FC1);
  for(size_t i = 0; i < num_samples; i++) {
    for(size_t j = 0; j < dim_state; j++)
      samples.at<double>(i,j) = resampled[i].state(j);
  }
  // Train models of different sized
  for(size_t i = 0; i < num_models; i++) {
    int num_mixtures = num_components - std::floor(0.5*num_models + 0.5) + i;
    if(num_mixtures <= 0)
      continue;
    models[i] = cv::EM(num_mixtures, cv::EM::COV_MAT_GENERIC);
    cv::Mat samples_loglike(num_samples, 1, CV_64FC1);
    try {
      models[i].train(samples, samples_loglike);
    } catch(...) { i++;continue; }
    // Compare the models using a complexity measurement
    double loglike = cv::sum(samples_loglike);
    // Number of parameters: one weight per component minus one, as the sum up to 1
    //                       + the mean for each component and half the covariance
    //                       matrix (as it is symetric)
    const double num_vars = num_mixtures - 1.0
                            + num_mixtures * dim_state
                            + num_mixtures * (dim_state * dim_state + dim_state) / 2.0;
    // BIC
    double complexity = -2.0 * loglike  + num_vars * std::log(num_samples);
    /*
    // AICc
    double complexity = 2.0 * num_vars - 2.0 * loglike + 2.0 * num_vars * (num_vars + 1.0) / (num_samples - num_vars - 1);
    */
    if(complexity < best_complexity) {
      best_complexity = complexity;
      model_indx = i;
      final_mixture = num_mixtures;
    }
  }
  const cv::Mat means(models[model_indx].get<cv::Mat>("means"));
  const cv::Mat weights(models[model_indx].get<cv::Mat>("weights"));

  // Copy back to the components_t
  components_t.resize(final_mixture);
  num_targets = 0;
  for(size_t i = 0; i < final_mixture; i++) {
    const cv::Mat cov(models[model_indx].get<std::vector<cv::Mat> >("covs")[i]);
    components_t[i].weight = total_weight * weights.at<double>(i);
    num_targets += components_t[i].weight;
    cv::cv2eigen(means.row(i), components_t[i].mean);
    cv::cv2eigen(cov, components_t[i].cov);
  }
#else
  // Use EM-GMM to recreate the gaussian mixture
  size_t num_components = (total_weight < 0.5 ? 1.0 : std::floor(total_weight + 0.5));
  size_t num_models = 5;
  std::vector<CvEM> models(num_models);
  double best_complexity = std::numeric_limits<double>::max();
  size_t final_mixture = 0;
  int damodel = NULL;
  cv::Mat samples(num_samples, dim_state, CV_32FC1);
  for(size_t i = 0; i < num_samples; i++) {
    for(size_t j = 0; j < dim_state; j++)
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
      Eigen::Matrix<double,dim_state,1> mean;
      Eigen::Matrix<double,dim_state,dim_state> cov;
      cv::Mat tmp_mean(models[i].get_means());
      cv::Mat tmp_cov(models[i].get_covs()[0]);
      cv::cv2eigen(tmp_mean.row(0).t(), mean);
      cv::cv2eigen(tmp_cov, cov);
      for(size_t j = 0; j < num_samples; j++) {
        Eigen::Matrix<double,dim_state, 1> df = resampled[j].state - mean;
        Eigen::Matrix<double,dim_state, 1> v = cov.llt().solve(df);
        const double log_normal = -0.5*df.transpose()*v + dim_state*LOG_SQRT_2PI_INV - 0.5 * std::log(cov.determinant());
        loglike += log_normal;
      }
    }
    // Number of parameters: one weight per component minus one, as the sum up to 1
    //                       + the mean for each component and half the covariance
    //                       matrix (as it is symetric)
    const double num_vars = num_mixtures - 1.0
                            + num_mixtures * dim_state
                            + num_mixtures * (dim_state * dim_state + dim_state) / 2.0;
    // BIC
    double complexity = -2.0 * loglike  + num_vars * std::log(num_samples);
    /*
    // AICc
    double complexity = 2.0 * num_vars - 2.0 * loglike + 2.0 * num_vars * (num_vars + 1.0) / (num_samples - num_vars - 1);
    */
    if(complexity < best_complexity) {
      best_complexity = complexity;
      damodel = i;
      final_mixture = num_mixtures;
    }
  }
  const cv::Mat means(models[damodel].get_means());
  const cv::Mat weights(models[damodel].get_weights());

  // Copy back to the components_t
  components_t.resize(final_mixture);
  num_targets = 0;
  for(size_t i = 0; i < final_mixture; i++) {
    const cv::Mat cov(models[damodel].get_covs()[i]);
    components_t[i].weight = total_weight * weights.at<double>(i);
    num_targets += components_t[i].weight;
    cv::cv2eigen(means.row(i), components_t[i].mean);
    cv::cv2eigen(cov, components_t[i].cov);
  }
#endif
}

template<int dim_state, int dim_measurement>
bool DTracker<dim_state, dim_measurement>::StartFollow() {
    publish_state = true;
    return true;
}


template<int dim_state, int dim_measurement>
bool DTracker<dim_state, dim_measurement>::PauseFollow() {
    publish_state = false;
    return true;
}


template<int dim_state, int dim_measurement>
bool DTracker<dim_state, dim_measurement>::StopFollow() {
    trPerson.id = -1;
    publish_state = false;
    return true;
}


template<int dim_state, int dim_measurement>
bool DTracker<dim_state, dim_measurement>::train_person() {

	float x_bender = PoseBender.pose.position.x, y_bender = PoseBender.pose.position.y;

    int id, cnt=0;
    double c = -1,x,y,vx,vy;
  //  while(cnt<=3 && (c==-1 || c>3)){
    	c=-1;
        for(typename ids_storage::iterator it = targets_id.begin(); it != targets_id.end(); it++) {
            double dx = it->second(0)-x_bender;
            double dy = it->second(1)-y_bender;
            double dvx = it->second(2);
            double dvy = it->second(3);
            id = it->first;
            double dist = std::sqrt(dx*dx + dy*dy);  
            if (c==-1 || dist <c){
                c=dist;
                x=dx; y=dy; vx=dvx; vy=dvy;

            } 
        }
		// if(c!=-1 && c<=3)	cnt++;
  //   }

    if(c!=-1 && c<=3){
      cout<<"Persona guardada"<<endl;
     // publish_state = true;
      publish_state = true;
      trPerson.id = id;
      trPerson.x = x;
      trPerson.y = y;
      trPerson.vx = vx;
      trPerson.vy = vy;
      
      return true;
    }
    
  return false;
}

template<int dim_state, int dim_measurement>
void DTracker<dim_state, dim_measurement>::prune() {
  for(size_t i = 0; i < components_t.size(); i++) {
    // Check for this mixture to be consistend
    if(boost::math::isnan(components_t[i].mean) || boost::math::isnan(components_t[i].weight)) {
      components_t.erase(components_t.begin() + i);
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
  /*
  std::vector<typename Eigen::LLT<state_mat>::Traits::MatrixL,
              Eigen::aligned_allocator<typename Eigen::LLT<state_mat>::Traits::MatrixL> > els_matrices;
  */
  std::vector<state_mat, Eigen::aligned_allocator<state_mat> > els_matrices;

  // Get the initial values for I
  for(unsigned int i = 0; i < components_t.size(); i++) {
    els_matrices.push_back(components_t[i].cov.llt().matrixL());
    if(!boost::math::isnan(components_t[i].weight) && components_t[i].weight > truncate_threshold
        && (!use_beta_mixture || (components_t[i].a > 1.0 && components_t[i].a / (components_t[i].a + components_t[i].b) > certainty_person_threshold))) {
      I.push_back(i);
    }
  }
  if(I.size() == 0) {
    components_t.resize(0);
    return;
  }
  do {
    // Find the components with the highest weight in I
    double max_weight = -1;
    unsigned int max_indx = 0;
    size_t remv_indx = 0;
    for(unsigned int i = 0; i < I.size(); i++) {
      const double &weight = components_t[I[i]].weight;
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
    double sum_psi_a = 0.0, sum_psi_b = 0.0;
    double merged_a = 0.0, merged_b = 0.0;
    if(use_beta_mixture) {
      // Initial values for the beta mixture
      const double psi_ab = gsl_sf_psi(components_t[max_indx].a + components_t[max_indx].b);
      const double psi_a = gsl_sf_psi(components_t[max_indx].a);
      const double psi_b = gsl_sf_psi(components_t[max_indx].b);
      sum_psi_a = components_t[max_indx].weight * (psi_ab - psi_a);
      sum_psi_b = components_t[max_indx].weight * (psi_ab - psi_b);
      merged_a = components_t[max_indx].weight * components_t[max_indx].a;
      merged_b = components_t[max_indx].weight * components_t[max_indx].b;
    }
    state average_state = components_t[max_indx].weight * components_t[max_indx].mean;
    double total_weight = components_t[max_indx].weight;
    const state &max_state = components_t[max_indx].mean;
    for(unsigned int i = 0; i < I.size(); i++) {
      state dx = components_t[I[i]].mean - max_state;
      state v = els_matrices[I[i]].template triangularView<Eigen::Upper>().solve(dx);
      const double sq_dist =  v.squaredNorm();
      if(sq_dist < merge_gauss_threshold) {
        double bt_dist = 0.0;
        if(use_beta_mixture)
          try {
            bt_dist = bhatta_dist_beta(components_t[max_indx].a, components_t[max_indx].b, components_t[I[i]].a, components_t[I[i]].b);
          } catch(std::runtime_error e) {
            ROS_WARN_STREAM("Problem calculating bhatta distance (" << components_t[max_indx].a << ", " << components_t[max_indx].b << ", " << components_t[I[i]].a << ", " << components_t[I[i]].b << ")" << std::endl << "Exception: " << e.what());
            bt_dist = merge_beta_threshold + 1;
          }
        // Do this comparation after the euclidean distance, to avoid calculating the bhatta distance all
        // the time
        if(use_beta_mixture && bt_dist > merge_beta_threshold) {
          continue;
        }
        if(use_beta_mixture) {
          const double psi_ab = gsl_sf_psi(components_t[I[i]].a + components_t[I[i]].b);
          const double psi_a = gsl_sf_psi(components_t[I[i]].a);
          const double psi_b = gsl_sf_psi(components_t[I[i]].b);
          sum_psi_a += components_t[I[i]].weight * (psi_ab - psi_a);
          sum_psi_b += components_t[I[i]].weight * (psi_ab - psi_b);
          // This is for an initial value for the optimisation
          merged_a += components_t[I[i]].weight * components_t[I[i]].a;
          merged_b += components_t[I[i]].weight * components_t[I[i]].b;
        }
        indices.push_back(I[i]);
        total_weight += components_t[I[i]].weight;
        average_state += components_t[I[i]].weight * components_t[I[i]].mean;
      }
    }
    // Generate the new component
    double merged_weight = total_weight;
    state merged_mean = average_state / total_weight;
    if(use_beta_mixture && use_optimal_beta_merge) {
      sum_psi_a /= total_weight;
      sum_psi_b /= total_weight;
      // Solve with GSL. First define the optimising function
      double params[] = {sum_psi_a, sum_psi_b};
      gsl_multiroot_function_fdf f = {&optimise_beta_params,
                                      &optimise_beta_params_df,
                                      &optimise_beta_params_fdf,
                                      2, params};
      // The starting points
      gsl_vector *x_0 = gsl_vector_alloc(2);
      gsl_vector_set(x_0, 0, merged_a);
      gsl_vector_set(x_0, 1, merged_b);
      // Create the solver
      gsl_multiroot_fdfsolver *s = gsl_multiroot_fdfsolver_alloc(gsl_multiroot_fdfsolver_gnewton, 2);
      gsl_multiroot_fdfsolver_set(s, &f, x_0);
      // Run the solver untill convergence or for 100 iterations
      size_t iter = 0;
      int status;
      do {
        status = gsl_multiroot_fdfsolver_iterate(s);
        if(status != GSL_SUCCESS)
          break;
        // TODO Check status
        status = gsl_multiroot_test_residual(s->f, 1E-5);
        iter++;
      } while(status == GSL_CONTINUE && iter < 100);
      // Get parameters back
      double optimal_a = gsl_vector_get(s->x, 0);
      double optimal_b = gsl_vector_get(s->x, 1);
      if(status != GSL_SUCCESS || optimal_a <= 0.0 || optimal_b <= 0.0) {
        ROS_WARN_STREAM("Problem optimising values for (a,b) : ("
                        << optimal_a << ", " << optimal_b
                        << "), starting point: (" << merged_a << ", " << merged_b << ")");
      }
      else {
        merged_a = optimal_a;
        merged_b = optimal_b;
      }
      // Release memory
      gsl_multiroot_fdfsolver_free(s);
      gsl_vector_free(x_0);
    }
    state dx = components_t[max_indx].mean - merged_mean;
    state_mat merged_cov = components_t[max_indx].weight * (components_t[max_indx].cov + dx*dx.transpose());
    for(unsigned int i = 0; i < indices.size(); i++) {
      dx = components_t[indices[i]].mean - merged_mean;
      merged_cov += components_t[indices[i]].weight * (components_t[indices[i]].cov + dx*dx.transpose());
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
      if(use_beta_mixture) {
        final_a.push_back(merged_a);
        final_b.push_back(merged_b);
      }
      final_weights.push_back(merged_weight);
      final_means.push_back(merged_mean);
      final_covs.push_back(merged_cov);
      num_targets += merged_weight;
    }
    else if(final_weights[max_components-1] < merged_weight) {
      // Replace to with the final item
      const size_t final_indx = final_weights.size()-1;
      if(use_beta_mixture) {
        final_a[final_indx] = merged_a;
        final_b[final_indx] = merged_b;
      }
      final_weights[final_indx] = merged_weight;
      final_means[final_indx] = merged_mean;
      final_covs[final_indx] = merged_cov;
      num_targets += merged_weight;
    }
  } while (I.size() > 0);
  // Assign the final mixture
  size_t final_size = final_weights.size();
  components_t.resize(final_size);
  for(size_t i = 0; i < final_size; i++) {
    components_t[i].weight = final_weights[i];
    components_t[i].mean = final_means[i];
    components_t[i].cov = final_covs[i];
    if(use_beta_mixture) {
      components_t[i].a = final_a[i];
      components_t[i].b = final_b[i];
    }
  }
  // Clear the cache of the bhattachaya distance
  clear_bhatta_cache();
  
}



template<int dim_state, int dim_measurement>
geometry_msgs::PoseStamped DTracker<dim_state, dim_measurement>::publish_marker( ) {

    float x_bender = PoseBender.pose.position.x;
    float y_bender = PoseBender.pose.position.y;
    float d = sqrt((trPerson.x-x_bender)*(trPerson.x-x_bender) + (trPerson.y-y_bender)*(trPerson.y-y_bender));
    float d_new = max(d-1.0,0.0); //separar, si d-1 es menor a 0 rotar

    geometry_msgs::PoseStamped out_msg,out_msg2;
    out_msg.header.frame_id = "/map";///bender/base_link";
    out_msg.header.stamp = ros::Time(0);  
    // out_msg.pose.position.x = trPerson.x;
    // out_msg.pose.position.y = trPerson.y; 
    //x1 + (x2-x1)*(d-1.0)/d
    out_msg.pose.position.x = x_bender + (trPerson.x-x_bender)*d_new/d;
    out_msg.pose.position.y = y_bender + (trPerson.y-y_bender)*d_new/d;

    out_msg.pose.position.z = 0;     

    float theta = atan2((trPerson.y-y_bender),(trPerson.x-x_bender));
    out_msg.pose.orientation.z = sinf(theta/2.0);
    out_msg.pose.orientation.w = cosf(theta/2.0);

 	out_msg2=out_msg;
    visualization_msgs::Marker marker;
    marker.header = out_msg.header;
    marker.ns = "follow";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = out_msg2.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    obj_pub.publish( marker );

    return  out_msg2;

}

template<>
void DTracker<6,3>::get_state(pcl::PointCloud<icl::PointWithVelocity>::Ptr &output, bool one_per_gaussian, bool get_only_moving) {
//  size_t num_targets = std::floor(this->num_targets + 0.5);
  size_t index = 0;
  size_t i = 0;
  findperson=false;
  output->points.resize(components_t.size());
  // The states are the peaks of the GMM
  for(; i < components_t.size(); i++) {
    if(components_t[i].weight >= 0.5) {
      if(use_beta_mixture) {
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
      }
      cout<<get_only_moving<<endl;
    //  get_only_moving=false; //ver si funciona
      if(get_only_moving) {
        const Eigen::Vector2d &vel = components_t[i].mean.block(2,0,2,1);
        if(vel.norm() < 0.2)
          continue;
      }
      if(!one_per_gaussian) {
        // Extract one for each expected number of people in this peak
        int num_copies = std::floor(components_t[i].weight + 0.5);
        if(index + num_copies > output->points.size()) {
          output->points.resize(index + num_copies);
        }
        for(int j = 0; j < num_copies; j++) {
          output->points[index].x = components_t[i].mean[0];
          output->points[index].y = components_t[i].mean[1];
          output->points[index].z = 0.0;
          output->points[index].velocity_x = components_t[i].mean[3];
          output->points[index].velocity_y = components_t[i].mean[4];
          output->points[index].velocity_z = 0.0;
          output->points[index].id = 0;
          index++;
        }
      }
      else {
        // Or extract for each peakñ
        output->points[index].x = components_t[i].mean[0];
        output->points[index].y = components_t[i].mean[1];
        output->points[index].z = 0.0;
        output->points[index].velocity_x = components_t[i].mean[3];
        output->points[index].velocity_y = components_t[i].mean[4];
        output->points[index].velocity_z = 0.0;
        output->points[index].id = 0;
        index++;
      }
    }
  }
  output->points.resize(index);
/*
  if(index < num_targets && components_t.size() > 0) {
    ROS_WARN_STREAM("Expected " << num_targets << " targets, but only found " << index << ". Filling the gaps");
    do {
      assign(output, index, components_t[i % components_t.size()].mean);
      index++;i++;
    } while(index < num_targets);
  }
*/
  // Calculate the ids
  Eigen::MatrixXd costs(index, targets_id.size());
  Eigen::MatrixXd costs_dist(index, targets_id.size());
  for(size_t i = 0; i < index; i++) {
    size_t j = 0;
    for(typename ids_storage::iterator it = targets_id.begin(); it != targets_id.end(); it++) {
      double dx = it->second(0) - output->points[i].x;
      double dy = it->second(1) - output->points[i].y;
      double dvx = it->second(2) - output->points[i].velocity_x;
      double dvy = it->second(3) - output->points[i].velocity_y;
      double dist = std::sqrt(dx*dx + dy*dy + dvx*dvx + dvy*dvy);
      costs(i, j) = dist;
      if(it->first == trPerson.id) cout<< trPerson.id<<" - "<<dist<<endl; 
      j++;
    }
  }
  costs_dist = costs;
  // Solve the assignment problem
  munkres::Munkres<Eigen::MatrixXd, Eigen::MatrixXi> hungarian;
  hungarian.solve(costs);

  typename ids_storage::iterator it = targets_id.begin();
  for(size_t j = 0; j < targets_id.size(); j++) {
//    bool used = false;
    for(size_t i = 0; i < index; i++) {
      // If a match
    	bool match = false;
      if(costs(i,j) == 0) {
        output->points[i].id  = it->first;
//        used = true;
        if(output->points[i].id  == trPerson.id){
          trPerson.x=output->points[i].x;
          trPerson.y=output->points[i].y;
          trPerson.vx=output->points[i].velocity_x;
          trPerson.vy=output->points[i].velocity_y;
          findperson=true;
          match=true;
        }
       // break;
      }
    if(!match){
      trPerson.x=output->points[i].x;
      trPerson.y=output->points[i].y;
      trPerson.vx=output->points[i].velocity_x;
      trPerson.vy=output->points[i].velocity_y;
      findperson=true;
    }

    }

    // Next element
    ++it;
  }
  // Clear the current ids
  targets_id.clear();
  // Chekc unsaigned ids
  for(size_t j = 0; j < index; j++) {
    if(output->points[j].id == 0) {
      output->points[j].id = ++last_id;
    }
    state copy;
    copy << output->points[j].x, output->points[j].y, output->points[j].z, output->points[j].velocity_x, output->points[j].velocity_y, output->points[j].velocity_z;
    targets_id[output->points[j].id] = copy;
  }
  if(!findperson){
    state copy;
    copy << trPerson.x, trPerson.y, trPerson.vx, trPerson.vy;
    targets_id[trPerson.id] = copy;
  }
}

//esta funcion se ocupa
template<int dim_state, int dim_measurement>
void DTracker<dim_state, dim_measurement>::get_state(pcl::PointCloud<icl::PointWithVelocity>::Ptr &output, bool one_per_gaussian, bool get_only_moving) {
//  size_t num_targets = std::floor(this->num_targets + 0.5);
	cout<<use_beta_mixture<<" "<<one_per_gaussian<<" "<<get_only_moving<<" "<<components_t.size()<<endl;
  size_t index = 0;
  size_t i = 0;
  findperson = false;
  output->points.resize(components_t.size());
  // The states are the peaks of the GMM
  for(; i < components_t.size(); i++) {
    if(components_t[i].weight >= 0.7) {
      if(use_beta_mixture) {
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
      }
      if(get_only_moving) {
        const Eigen::Vector2d &vel = components_t[i].mean.block(2,0,2,1);
        if(vel.norm() < 0.2)
          continue;
      }
      // birth_prior_means
      if(!one_per_gaussian) {
        // Extract one for each expected number of people in this peak
        int num_copies = std::floor(components_t[i].weight + 0.5);
        if(index + num_copies > output->points.size()) {
          output->points.resize(index + num_copies);
        }
        for(int j = 0; j < num_copies; j++) {
          output->points[index].x = components_t[i].mean[0];
          output->points[index].y = components_t[i].mean[1];
          output->points[index].z = 0.0;
          output->points[index].velocity_x = components_t[i].mean[2];
          output->points[index].velocity_y = components_t[i].mean[3];
          output->points[index].velocity_z = 0.0;
          output->points[index].id = 0;
          index++;
        }
      }
      else {
        // Or extract for each peak
        output->points[index].x = components_t[i].mean[0];
        output->points[index].y = components_t[i].mean[1];
        output->points[index].z = 0.0;
        output->points[index].velocity_x = components_t[i].mean[2];
        output->points[index].velocity_y = components_t[i].mean[3];
        output->points[index].velocity_z = 0.0;
        output->points[index].id = 0;
        index++;
      }
    }
  }
  output->points.resize(index);
  cout<<"  - "<<index<<endl;
/*
  if(index < num_targets && components_t.size() > 0) {
    ROS_WARN_STREAM("Expected " << num_targets << " targets, but only found " << index << ". Filling the gaps");
    do {
      assign(output, index, components_t[i % components_t.size()].mean);
      index++;i++;
    } while(index < num_targets);
  }
*/
  // Calculate the ids 
  float distmin = -1;
  float delta;
  int jmin;
  Eigen::MatrixXd costs(index, targets_id.size());
    Eigen::MatrixXd costs_dist(index, targets_id.size());
  // cout<<"n person detect : "<<index<<endl;
  for(size_t i = 0; i < index; i++) {
    size_t j = 0;
    for(typename ids_storage::iterator it = targets_id.begin(); it != targets_id.end(); it++) {
      time_t now_time = time(NULL); 
      delta = difftime(now_time,last_time);
      // double dx = it->second(0) - output->points[i].x;
      // double dy = it->second(1) - output->points[i].y;
      // double dvx = it->second(2) - output->points[i].velocity_x;
      // double dvy = it->second(3) - output->points[i].velocity_y;
      // double dist = std::sqrt(dx*dx + dy*dy + dvx*dvx + dvy*dvy); 

      float dx0 = it->second(0) - output->points[i].x;
      float dy0 = it->second(1) - output->points[i].y;
      float dx = it->second(0)+it->second(2)*delta - output->points[i].x;
      float dy = it->second(1)+it->second(3)*delta - output->points[i].y;
      float dx2 = it->second(0)+output->points[i].velocity_x*delta - output->points[i].x;
      float dy2 = it->second(1)+output->points[i].velocity_y*delta - output->points[i].y;
      dx = dx0;//min(abs(dx),abs(dx2));
      dy = dy0;//min(abs(dy),abs(dy2));
      double dist = std::sqrt(dx*dx + dy*dy); 
      // cout<<dist<<"(x:"<<it->second(0)+it->second(2)*delta<<","<<output->points[i].x<<") "<<"(y:"<<it->second(1)+it->second(3)*delta<<","<<output->points[i].y<<") "<<endl;
      if(it->first == trPerson.id){
       if(distmin == -1 || dist<distmin){
        jmin = j;
        distmin = dist;
      }
     }
      costs(i, j) = dist; 
     // j++;
    }
  }
  costs_dist = costs;
  // cout<<distmin<<endl;
  bool obstacle = false;
  // Solve the assignment problem
  munkres::Munkres<Eigen::MatrixXd, Eigen::MatrixXi> hungarian;
  hungarian.solve(costs);
  typename ids_storage::iterator it = targets_id.begin();
  for(size_t j = 0; j < targets_id.size(); j++) {
//    bool used = false;
    for(size_t i = 0; i < index; i++) {
      // If a match
    	bool match = false;
      // if(costs(i,j) == 0) {
        output->points[i].id  = it->first;

        if(output->points[i].id  == trPerson.id && j == jmin && (distmin > 0) && distmin < 0.8){
       //if( i == jmin && (distmin > 0) && distmin < 0.8){

        //  bender_srvs::Transformer tin;


          trPerson.x=output->points[i].x;
          trPerson.y=output->points[i].y;
          trPerson.vx=output->points[i].velocity_x;
          trPerson.vy=output->points[i].velocity_y;
          last_time = time(NULL); 

          findperson=true;
		  match = true;          
        }

        //break;
      // }

      if(!match){
      		bool in_centro = false, t1= false, t2= false;
      		float delta_t = 0.0;
      	    float obst_x = output->points[i].x+output->points[i].velocity_x*delta_t;
      		float obst_y = output->points[i].y+output->points[i].velocity_y*delta_t;

            float x_bender = PoseBender.pose.position.x, y_bender = PoseBender.pose.position.y;
            float x_object = trPerson.x, y_object = trPerson.y;

            float xmin = min(x_bender,x_object), xmax = max(x_bender,x_object);
            float ymin = min(y_bender,y_object), ymax = max(y_bender,y_object);

            if (obst_x>xmin && obst_x<xmax && obst_y>ymin && obst_y<ymax)
            		in_centro = true;

      		// delta_t = 0.002;
      	 //    obst_x = output->points[i].x+output->points[i].velocity_x*delta_t;
      		// obst_y = output->points[i].y+output->points[i].velocity_y*delta_t;

        //     if (obst_x>xmin && obst_x<xmax && obst_y>ymin && obst_y<ymax)    		t1 = true;


      		// delta_t = 0.1;
      	 //    obst_x = output->points[i].x+output->points[i].velocity_x*delta_t;
      		// obst_y = output->points[i].y+output->points[i].velocity_y*delta_t;

        //     if (obst_x>xmin && obst_x<xmax && obst_y>ymin && obst_y<ymax)    		t2 = true;

          //   if(in_centro || t1 || t2){
          //   	cout<<"obstaculo "<<in_centro<<" "<<t1<<" "<<t2<<endl;
	         //    obstacle = true;
	         // }


      }

    }
    // Next element
    ++it;
  }
  if(findperson && publish_state && !obstacle) {
    geometry_msgs::PoseStamped out_msg = this->publish_marker();
    if(delta>0.25 )
    	_mov_pub.publish(out_msg);
}

  // Clear the current ids
  targets_id.clear();
  // Chekc unsaigned ids
  for(size_t j = 0; j < index; j++) {
    if(output->points[j].id == 0) {
      output->points[j].id = ++last_id;
    }
    state copy;
    copy << output->points[j].x, output->points[j].y, output->points[j].velocity_x, output->points[j].velocity_y;
    targets_id[output->points[j].id] = copy;
  }

  if(!findperson && trPerson.id != -1){
    float xtr = trPerson.x, ytr = trPerson.y;
    // if(last_seen){
    //   xtr = trPerson.x + trPerson.vx*delta;
    //   ytr = trPerson.y + trPerson.vy*delta;
    // }
    state copy;
    copy << xtr, ytr, trPerson.vx, trPerson.vy;
    targets_id[trPerson.id] = copy;
  }
last_seen = findperson;
}

template<int D, int M>
void _sample_from_mixture(const typename DTracker<D,M>::components_type &mixture,
                         size_t num_samples, typename _particle<D>::particles &out) {
  boost::mt19937 rng;
  boost::uniform_01<> unf;
  boost::normal_distribution<> nor(0,1);
  boost::variate_generator<boost::mt19937, boost::uniform_01<> > var_unf(rng, unf);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > var_nor(rng, nor);

  std::vector<Eigen::MatrixXd> sqrtCovs(mixture.size());
  double total_weight = 0;
  for(size_t i = 0; i < mixture.size(); i++) {
    sqrtCovs[i] = mixture[i].cov.llt().matrixL();
    total_weight += mixture[i].weight;
  }
  out.resize(num_samples);
  for(size_t i = 0; i < num_samples; i++) {
    double sample_weight = var_unf() * total_weight;
    size_t j = 0;
    for(; j < mixture.size(); j++) {
      sample_weight -= mixture[j].weight;
      if(sample_weight <= 0)
        break;
    }
    if(j == mixture.size()) j--;
    Eigen::Matrix<double,5,1> rand_01;
    for(size_t k = 0; k < 5; k++) {
      rand_01(k) = var_nor();
    }
    // Convert the N(0,I) to N(u,S)
    out[i].weight = total_weight / num_samples;
    out[i].state = mixture[j].mean +  sqrtCovs[j] * rand_01;
  }
}

template<int D>
void _sample_from_samples(const typename _particle<D>::particles &in,
                         size_t num_samples, typename _particle<D>::particles &out) {
  boost::mt19937 rng;
  boost::uniform_01<> unf;
  boost::variate_generator<boost::mt19937, boost::uniform_01<> > var_unf(rng, unf);

  double total_weight = 0;
  for(size_t i = 0; i < in.size(); i++) {
    total_weight += in[i].weight;
  }
  for(size_t i = 0; i < num_samples; i++) {
    double sample_weight = var_unf() * total_weight;
    size_t j = 0;
    for(; j < in.size(); j++) {
      sample_weight -= in[j].weight;
      if(sample_weight <= 0)
        break;
    }
    if(j == in.size()) j--;
    out[i].state = in[j].state;
    out[i].weight = total_weight / num_samples;
  }
}
