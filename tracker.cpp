#include "tracker.h"

#define TIMING true
#define START_TIMER(msg, start_timer) \
    do { \
        if (TIMING) { \
			start_timer = clock(); \
			double start_t = double(start_timer) / CLOCKS_PER_SEC; \
			cout << "TIMER | " << msg << " | "<<start_t << endl; \
        } \
    } while (false)

#define END_TIMER(msg, start_timer) \
    do { \
        if (TIMING) { \
			clock_t end_timer = clock(); \
			double start_t = double(start_timer) / CLOCKS_PER_SEC; \
			double end_t = double(end_timer)/ CLOCKS_PER_SEC; \
			double elapsed = end_t - start_t; \
			cout << "TIMER | " << msg << " | "<< end_t << endl; \
			cout << "TIMER Elapsed " << elapsed << endl; \
			printf("\n"); \
		} \
	} while (false)


void get_gt_info(map<string, tracking>& info, json gt_data) {
	for (json mes : gt_data) {
		if (mes["type"] == "gt_reconstruct") {
			Matrix44 M_L_U;
			string user;
			get_pose_matrix(mes, user, M_L_U);

			//If user hasn't been added yet
			if (info.find(user) == info.end()) {
				tracking t;
				t.is_beacon = false;
				info.insert(make_pair(user, t));
			}
			info.at(user).gt_poses.push_back(Pose3(M_L_U)); //Load all GT poses into tracking.
		}
	}
}

void get_beacon_info(map<string, tracking>& info, json beacon_data) {
	// Beacon position will 
	for (json beacon : beacon_data) {
		Rot3 rot();
		Vector3 v;
		auto raw_position = beacon["position"];
		int i = 0;
		for (const auto& row : raw_position) {
			v(i) = row;
			i++;
		}

		string user = to_string(beacon["ID"]);
		Pose3 beacon_pos(Rot3::Identity(), v);

			//If beacon hasn't been added yet
			if (info.find(user) == info.end()) {
				tracking t;
				t.gt_poses.push_back(beacon_pos); // Only need to push back once
				t.is_beacon = true;
				info.insert(make_pair(user, t));
			}
	}
}

Tracker::Tracker(
			const string& id,
            const Pose3 T_body_to_imu,
			const Pose3 T_body_to_decawave,
			const double smoother_lag,
			const bool use_smoother,
			const bool use_filter,
            const SharedNoiseModel& SLAM_noise_model,
            const SharedNoiseModel& UWB_noise_model,
            const SharedNoiseModel& Velocity_noise_model,
            const SharedNoiseModel& Bias_noise_model,
			std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params,
            const Vector6 prior_imu_bias,
			const Vector3 prior_velocity,
            const string& debug_dir) : 
            
            translation_filt(
                200.,
                Eigen::Array<double, 3, 1>::Constant(0.25),   // min_cutoff
                Eigen::Array<double, 3, 1>::Constant(0.01),    // beta > 1
                Eigen::Array<double, 3, 1>::Constant(1),   // d_cutoff
                Eigen::Array<double, 3, 1>::Zero(),          // zero
                Eigen::Array<double, 3, 1>::Ones(),          // one
                [](auto& in) { return in.abs(); }            // abs function
            ){

	this->T_body_to_imu = T_body_to_imu;
	this->T_body_to_decawave = T_body_to_decawave;

    this->prior_imu_bias = imuBias::ConstantBias(prior_imu_bias);
	this->prior_velocity = prior_velocity;

	// Instantiate IMU preintegration
	this->imu_preintegrated = new PreintegratedCombinedMeasurements(imu_preintegration_params, this->prior_imu_bias);
	this->delta_t = 1/200.0;

	this->debug_dir = debug_dir;

	
    // Initialize noise models
    this->SLAM_noise_model = SLAM_noise_model;
    this->UWB_noise_model = UWB_noise_model;
    this->Velocity_noise_model = Velocity_noise_model;
    this->Bias_noise_model = Bias_noise_model;

    // Instantiate graph
    this->graph = new NonlinearFactorGraph();

    // Values initialized in class
    // Tracking initialized in class

	this->use_filter = use_filter;

	this->use_smoother = use_smoother;
	if (use_smoother) {
		ISAM2Params isam_params;
		// Must be some way to set up a verbose option.
		isam_params.factorization = ISAM2Params::QR;
		isam_params.relinearizeThreshold = 0.01;
		isam_params.relinearizeSkip = 1; // More informed optimization at the cost of more computing.
		isam_params.enableDetailedResults = true;
		this->smoother = new IncrementalFixedLagSmoother(smoother_lag, isam_params);
	}
	else {
    	// Instantiate iSAM
		ISAM2Params isam_params;
		// Must be some way to set up a verbose option.
		isam_params.factorization = ISAM2Params::QR;
		isam_params.relinearizeThreshold = 0.01;
		isam_params.relinearizeSkip = 1; // More informed optimization at the cost of more computing.
		isam_params.enableDetailedResults = true;
		ISAM2DoglegParams dogleg;
		isam_params.optimizationParams = dogleg;
		this->isam = new ISAM2(isam_params);
	}
	// key_to_ts gets initialized in class

}

Pose3 Tracker::report_estimate(Pose3 initial, double timestamp){

	Pose3 reported_pose;
	if (use_filter) {
		// Timestamps in seconds, mincutoff beta should be in seconds
		Vector3 filtered_translation = translation_filt(initial.translation() , timestamp);
		Pose3 good_pose(Pose3(initial.rotation(), filtered_translation));
		reported_pose = good_pose;

		cout << "Filtering changed pose by " << (initial.translation().norm() - filtered_translation.norm()) << endl;
		track.est_poses.push_back(good_pose);
		track.est_timestamps.push_back(timestamp);
	}
	else { // Don't filter
		track.est_poses.push_back(initial);
		track.est_timestamps.push_back(timestamp);
		reported_pose = initial;
	}

	return reported_pose;
}

Key AnchorKey(string name) {return symbol('s', stoi(name));}


// Assuming we have already called
// get_beacon_info(tracker.anchors, json::parse(beacon_fs));
void Tracker::init_anchor(string id){
    Pose3 prior_beacon_pose(anchors[id].gt_poses[0]);
    vals.insert(AnchorKey(id), prior_beacon_pose);
    graph->add(NonlinearEquality<Pose3>(AnchorKey(id), prior_beacon_pose));
}

void Tracker::init_anchors(json anchor_json) {
	get_beacon_info(anchors, anchor_json); // Reads raw pose data into map
	for (auto& [id, anchor_track]: anchors){
		init_anchor(id); // Uses anchor pose to set pose prior, and inserts into values.
	}
}

void Tracker::init_state(json calibration_stream) {
    track.Ix = 0;
    track.Iv = 0;
    track.Ib = 0;

	// prior_pose needs to come from the first pose in the sensor stream.

	bool set_pose_prior = false;
	bool last_was_pose = false;
	double latest_pose_timestamp = 0;
	Values result;

	int imu_counter = 0;
	int vicon_counter = 0;

	bool calibrate_bias = true;

	for (json mes: calibration_stream) {
		// First set all priors
		if ( mes["type"]=="aligned_slam_pose" && !set_pose_prior) {
			Pose3 start_slam_pose; 
			Vector3 start_slam_velocity(0,0,0);
			double timestamp;

			Pose3 T_world_to_body;
			// velocity and body pose an no_uwb 0.0 re computed from body poses in the world frame
			get_pose_from_HTM(mes["T_body_world"],T_world_to_body);
			Pose3 T_body_to_world = T_world_to_body.inverse();
			start_slam_pose = T_body_to_world;
			
			// get_V(mes, start_slam_velocity); // velocity // NOTE: CHANGED VELOCITY!

			Rot3 rot_imu_to_body = T_body_to_imu.rotation();
			Pose3 prior_pose = start_slam_pose;

			// Pose3 pose_velocity(Rot3::Identity(), start_slam_velocity);
			// Vector3 prior_velocity = start_slam_velocity;

			vals.insert(X(track.Ix), prior_pose);
			vals.insert(V(track.Iv), prior_velocity);
			vals.insert(B(track.Ib), prior_imu_bias);

			graph->addPrior(X(track.Ix), prior_pose, SLAM_noise_model);
			graph->addPrior(V(track.Iv), prior_velocity, Velocity_noise_model);
			graph->addPrior(B(track.Ib), prior_imu_bias, Bias_noise_model);

			last_was_pose = true;
			set_pose_prior = true;

			isam->update(*graph, vals);
			prev_state = NavState(prior_pose, prior_velocity);
			imu_preintegrated->resetIntegrationAndSetBias(prior_imu_bias);
			track.changing_bias = prior_imu_bias;

			// Clear for next iteration
			// graph->resize(0);
			// vals.clear();
			// Need to get result out of iSAM

			result = vals;
			PreintegrationBase::Bias optimized_bias = result.at<gtsam::PreintegrationBase::Bias>(B(track.Ib));
			Pose3 latest_pose = result.at<Pose3>(X(track.Ix));
			Vector3 latest_velocity = result.at<Vector3>(V(track.Iv));

			cout << "Prior bias estimate" << std::endl;
			prior_imu_bias.print();

			cout << "Optimized bias applied to preintegrator" << std::endl;
			imu_preintegrated->print();


			track.est_poses.push_back(latest_pose); // We'll take the estimate out of values and put it here.
			track.gt_poses.push_back(latest_pose);
			track.est_timestamps.push_back(latest_pose_timestamp);
			track.gt_timestamps.push_back(latest_pose_timestamp);
			track.est_velocities.push_back(latest_velocity);
			track.constant_bias = optimized_bias;
			track.changing_bias = optimized_bias;

			vals.clear(); // TODO: Do we need to clear here? if we're using LM

			// Initialize our NavState
			prev_state = NavState(latest_pose, latest_velocity);

		}
	}

	
}

void Tracker::exec_iSAM(NavState& proposed, double mes_timestamp, 
    string msg, bool print){

	Values result;

	try {
        if (print) {
            cout << "Keys in vals: ";
            for (const auto& key : vals.keys()) {
                cout << DefaultKeyFormatter(key) << " ";
            }
            cout << endl;

            cout << "Keys in graph: ";
            for (const auto& f : *graph) {
                auto keys = f->keys();
                for (Key k : keys) {
                    cout << DefaultKeyFormatter(k) << " ";
                }
            }
            cout << endl;
        }
		
		clock_t isam_t;
		START_TIMER("Start iSAM, "+msg+", ts="+to_string(mes_timestamp), isam_t);
		isam->update(*graph, vals);
		result = isam->calculateEstimate();
		END_TIMER("Ended iSAM "+msg, isam_t);

		report_estimate(result.at<Pose3>(X(track.Ix)), mes_timestamp);

		track.est_velocities.push_back(result.at<Vector3>(V(track.Iv)));

		prev_state = NavState(result.at<Pose3>(X(track.Ix)), result.at<Vector3>(V(track.Iv)));
		track.changing_bias = result.at<PreintegrationBase::Bias>(B(track.Ib));

		// Clear for next iteration
		graph->resize(0);
		vals.clear();

		imu_preintegrated->resetIntegrationAndSetBias(track.changing_bias);
	}
	catch (const std::exception& e) {
		cerr << "Optimizer update failed: " << e.what() << endl;
		cerr << "Data timestamp is " << mes_timestamp << endl;
		graph->saveGraph(debug_dir+"/graph.dot", result);
		graph->print("");
		cerr << "Graph dumped to factor_graph.dot" << endl;


		write_trajectory_TUM_format( track.est_poses, track.est_timestamps, *estimated_trajectory_fs, T_body_to_imu);
		estimated_trajectory_fs->close();

		write_trajectory_TUM_format( track.gt_poses, track.gt_timestamps, *slam_trajectory_fs, T_body_to_imu);
		slam_trajectory_fs->close();

		throw; // rethrow
	}
}

void Tracker::exec_smoother(NavState& proposed, double mes_timestamp, 
    string msg, bool print){

	Values result;

	try {
        if (print) {
            cout << "Keys in vals: ";
            for (const auto& key : vals.keys()) {
                cout << DefaultKeyFormatter(key) << " ";
            }
            cout << endl;

            cout << "Keys in graph: ";
            for (const auto& f : *graph) {
                auto keys = f->keys();
                for (Key k : keys) {
                    cout << DefaultKeyFormatter(k) << " ";
                }
            }
            cout << endl;

        }
		
		clock_t isam_t;
		START_TIMER("Start iSAM, "+msg+", ts="+to_string(mes_timestamp), isam_t);
		smoother->update(*graph, vals, key_timestamps); // Crashes on this line specifically
		result = smoother->calculateEstimate();
		END_TIMER("Ended iSAM "+msg, isam_t);

		report_estimate(result.at<Pose3>(X(track.Ix)), mes_timestamp);

		track.est_velocities.push_back(result.at<Vector3>(V(track.Iv)));

		prev_state = NavState(result.at<Pose3>(X(track.Ix)), result.at<Vector3>(V(track.Iv)));

		track.changing_bias = result.at<PreintegrationBase::Bias>(B(track.Ib));

		cout << " Bias estimate " << endl;
		track.changing_bias.print();

		// Clear for next iteration
		graph->resize(0);
		vals.clear();
		key_timestamps.clear();


		imu_preintegrated->resetIntegrationAndSetBias(track.changing_bias);
	}
	catch (const std::exception& e) {
		cerr << "Optimizer update failed: " << e.what() << endl;
		cerr << "Data timestamp is " << mes_timestamp << endl;
		graph->saveGraph(debug_dir+"/graph.dot", result);
		graph->print("");
		cerr << "Graph dumped to factor_graph.dot" << endl;

		write_trajectory_TUM_format( track.est_poses, track.est_timestamps, *estimated_trajectory_fs, T_body_to_imu);
		estimated_trajectory_fs->close();

		write_trajectory_TUM_format( track.gt_poses, track.gt_timestamps, *slam_trajectory_fs, T_body_to_imu);
		slam_trajectory_fs->close();
		throw; // rethrow
	}
}


void Tracker::processSLAM(const json& mes)
{
	cout << "Used GT" << endl;
	track.Ix++;
	track.Iv++;
	track.Ib++;

	// Extract GT pose
	Pose3 T_world_to_body;
	string usrname;
	// Notation; T_body_world in the file is = T_world_to_body
	// For proper gravity compensation, GTSAM needs T_body_to_world
	// But I plot everything and output data from post process using T_world_to_body
	get_pose_from_HTM(mes["T_body_world"],T_world_to_body);
	Pose3 T_body_to_world = T_world_to_body.inverse();
	Pose3 gt_pose = T_body_to_world;

	// Pose3 gt_pose = T_world_to_imu;
	track.gt_poses.push_back(gt_pose);
	track.gt_timestamps.push_back(mes["t"]);

	// Add IMU factor
	auto* current_imu_preintegration =
		dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);

	// Add this key -> timestamp mapping to our map
	key_timestamps[X(track.Ix)] = (double)mes["t"];
	key_timestamps[V(track.Iv)] = (double)mes["t"];
	key_timestamps[B(track.Ib)] = (double)mes["t"];
	for (auto const &[id, tracking_] : anchors) {
		key_timestamps[AnchorKey(id)] = (double)mes["t"];
	}

	CombinedImuFactor imu_factor(
		X(track.Ix - 1), V(track.Iv - 1),
		X(track.Ix), V(track.Iv),
		B(track.Ib - 1), B(track.Ib),
		*current_imu_preintegration);

	graph->add(imu_factor);
	cout << "Added IMU factor " << graph->size() - 1 << endl;

	// Add GT prior factor
	graph->add(PriorFactor<Pose3>(X(track.Ix), gt_pose, SLAM_noise_model));
	cout << "Added Prior factor " << graph->size() - 1 << endl;

	// Predict current state
	NavState proposed = current_imu_preintegration->predict(prev_state, track.changing_bias);

	// Insert initial values
	vals.insert(X(track.Ix), proposed.pose());
	vals.insert(V(track.Iv), proposed.v());
	vals.insert(B(track.Ib), track.changing_bias);

    // Run iSAM
	if (use_smoother) { exec_smoother(proposed, (double)mes["t"], "GT", true); }
	else { exec_iSAM(proposed, (double)mes["t"], "GT", true); }
	

	// translation_filt.clear(); // clear filter
}

void Tracker::processUWB(const json& mes, int& uwb_counter)
{
	cout << "Processing range This -> " << mes["id"] << " for t=" << mes["t"] << endl;

	track.Ix++;
	track.Iv++;
	track.Ib++;
	uwb_counter++;

	double measured_range = (double)mes["range"];

	// Add this key -> timestamp mapping to our map
	key_timestamps[X(track.Ix)] = (double)mes["t"];
	key_timestamps[V(track.Iv)] = (double)mes["t"];
	key_timestamps[B(track.Ib)] = (double)mes["t"];
	for (auto const &[id, tracking_] : anchors) {
		if (id != "") key_timestamps[AnchorKey(id)] = (double)mes["t"];
	}

	string dst = to_string((int)mes["id"]);

	
	// Needs "Pose of antenna in body frame" i.e. decawave_to_body
	graph->add(RangeFactorWithTransform<Pose3, Pose3, double>(
		X(track.Ix), AnchorKey(dst), measured_range, UWB_noise_model, T_body_to_decawave.inverse()));
	// graph->add(RangeFactor<Pose3, Pose3, double>(
	// 	X(track.Ix), AnchorKey(dst), measured_range, UWB_noise_model));

	cout << "Added Range factor to state " << track.Ix << endl;

	// Add IMU factor
	auto* current_imu_preintegration =
		dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);

	CombinedImuFactor imu_factor(
		X(track.Ix - 1), V(track.Iv - 1),
		X(track.Ix), V(track.Iv),
		B(track.Ib - 1), B(track.Ib),
		*current_imu_preintegration);

	graph->add(imu_factor);
	cout << "Added IMU factor " << graph->size() - 1 << endl;
	imu_preintegrated->print();

	// Predict state and insert
	// NavState proposed = current_imu_preintegration->predict(prev_state, track.constant_bias);
	NavState proposed = current_imu_preintegration->predict(prev_state, track.changing_bias);
	vals.insert(X(track.Ix), proposed.pose());
	vals.insert(V(track.Iv), proposed.v());
	vals.insert(B(track.Ib), track.changing_bias);

	// Run optimization
	if (use_smoother) { exec_smoother(proposed, (double)mes["t"], "SynthUWB", true); }
	else { exec_iSAM(proposed, (double)mes["t"], "SynthUWB", true); }
}



std::shared_ptr<PreintegratedCombinedMeasurements::Params> get_imu_preintegration_params(int ASCALE, int GSCALE, Pose3 T_inertial_to_world) {


	double GYRO_NOISE_DENSITY = 0.0002049600985797649; 
	double ACCEL_NOISE_DENSITY = 0.002064189891192468;

	Matrix33 continuous_time_accel_noise_cov = I_3x3 * pow(ACCEL_NOISE_DENSITY, 2) * ASCALE;
	Matrix33 continuous_time_gyro_noise_cov = I_3x3 * pow(GYRO_NOISE_DENSITY, 2) * GSCALE;


	double GYRO_BIAS_RW = 3.1998555455947417e-06;
	double ACCEL_BIAS_RW = 0.00022919238444020807;

	Matrix33 continuous_time_accel_bias_rw = I_3x3 * pow(ACCEL_BIAS_RW, 2) * ASCALE;
	Matrix33 continuous_time_gyro_bias_rw = I_3x3 * pow(GYRO_BIAS_RW, 2) * GSCALE;

	Matrix66 initial_bias_cov = I_6x6 * 1e-5 * ASCALE;
	Matrix33 integration_cov = I_3x3 * 1e-5 * ASCALE;

	// const Vector3 gravity = T_inertial_to_world.rotation() * Vector3(0,0,-9.806);

	// std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params =std::shared_ptr<PreintegrationCombinedParams>( new PreintegrationCombinedParams(gravity));
	// // std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params =std::shared_ptr<PreintegrationCombinedParams>( new PreintegrationCombinedParams(Vector3(0,0,-9.81)));
	std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = PreintegratedCombinedMeasurements::Params::MakeSharedU();

	imu_preintegration_params->accelerometerCovariance = continuous_time_accel_noise_cov;
	imu_preintegration_params->gyroscopeCovariance = continuous_time_gyro_noise_cov;

	imu_preintegration_params->biasAccCovariance = continuous_time_accel_bias_rw;
	imu_preintegration_params->biasOmegaCovariance = continuous_time_gyro_bias_rw;

	imu_preintegration_params->integrationCovariance = integration_cov;
	imu_preintegration_params->biasAccOmegaInt = initial_bias_cov;

	imu_preintegration_params->use2ndOrderCoriolis = false;

	return imu_preintegration_params;
}