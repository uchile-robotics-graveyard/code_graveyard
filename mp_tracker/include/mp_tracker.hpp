template<typename Container>
void MPTracker::get_state(Container &output, bool manage_tracks) {
  size_t num_targets = std::floor(this->num_targets + 0.5);
  resize(output, num_targets);
  size_t index = 0;
  size_t i = 0;
  std::vector<Eigen::VectorXd> estimates;
  estimates.reserve(num_targets);
  // The states are the peaks of the GMM
  for(; i < states_t.size(); i++) {
    if(states_t[i]->weight > 0.5) {

      int num_copies = std::floor(states_t[i]->weight + 0.5);
      size_t tmp = _size(output);
      if(index + num_copies > tmp) {
        resize(output, index + num_copies);
      }
      for(int j = 0; j < num_copies; j++) {
        estimates.push_back(states_t[i]->posvelheight);
        assign(output, index, states_t[i]->posvelheight);
        index++;
      }
    }
  }
#ifdef MANAGE_TRACKS
  // Make the track matching based on nearest neightbours
  if(manage_tracks) {
    const double et = last_update_time - start_running;
#ifdef DEBUG
    std::cout << "# Estimating paths at time " << et << std::endl;
#endif
    size_t n = estimates.size(), m = active_paths.size();
    Eigen::MatrixXd costs = max_path_distance * Eigen::MatrixXd::Ones(n,n+m);
    for(size_t i = 0; i < n; i++) {
      const Eigen::VectorXd &current = estimates[i];
      for(size_t j = 0; j < m; j++) {
        // Cost of estimation i of being part of track j
        const double dt = et - paths[active_paths[j]].rbegin()->first;
        F(0, 2) = dt;
        F(1, 3) = dt;
        const Eigen::VectorXd rought = F * paths[active_paths[j]].rbegin()->second;
        double cost = (current - rought).norm();
        costs(i, j) = cost;
      }
    }
    // Solve the assignment problem
    munkres::Munkres<Eigen::MatrixXd, Eigen::MatrixXi> hungarian;
    hungarian.solve(costs);
    // Continue the paths or start new paths
    std::vector<size_t> new_active_paths(n);
    std::vector<bool> disassociated_paths(m);
    std::fill(disassociated_paths.begin(), disassociated_paths.end(), true);
    for(size_t i = 0; i < n; i++) {
      bool assigned = false;
      for(size_t j = 0; j < m; j++) {
        if(costs(i,j) == 0) {
          paths[active_paths[j]][last_update_time - start_running] = estimates[i];
          new_active_paths[i] = active_paths[j];
          disassociated_paths[j] = false;
          assigned = true;
          break;
        }
      }
      // Start a new track
      if(!assigned) {
        paths[last_path][et] = estimates[i];
        new_active_paths[i] = last_path;
        last_path++;
      }
    }
    // Keep tracks even if they are not detected for 1 second
    for(size_t j = 0; j < m; j++) {
      if(disassociated_paths[j]) {
        const double last_update = paths[active_paths[j]].rbegin()->first;
        // If a path in unseen for less than a second keep it as active
        if(std::fabs(last_update - et) < 1.0) {
#ifdef DEBUG
          std::cout << "# Path:" << active_paths[j] << " not detected for " << std::fabs(last_update - et) << " seconds." << std::endl;
#endif
          new_active_paths.push_back(active_paths[j]);
        }
      }
    }
#ifdef DEBUG
    std::cout << "#new_active_paths.size()=" << new_active_paths.size() << std::endl;
#endif
    active_paths.resize(new_active_paths.size());
    std::copy(new_active_paths.begin(), new_active_paths.end(), active_paths.begin());
  }
#endif
  if(index < num_targets && states_t.size() > 0) {
    do {
      assign(output, index, states_t[i % states_t.size()]->posvelheight);
      index++;i++;
    } while(index < num_targets);
  }
}

