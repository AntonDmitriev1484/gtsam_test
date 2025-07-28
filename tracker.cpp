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

Tracker::Tracker(const string& id,
            const Pose3 T_imu_body,
            const double delta_t,
			const double smoother_lag,
			const bool use_smoother,
			const bool use_filter,
			const double uwb_stdev,
            const SharedNoiseModel& GT_noise_model,
            const SharedNoiseModel& UWB_noise_model,
            const SharedNoiseModel& FakePrior_noise_model,
            const SharedNoiseModel& Velocity_noise_model,
            const SharedNoiseModel& Bias_noise_model,
			std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params,
            const imuBias::ConstantBias prior_imu_bias,
            const string& debug_dir) : 
            
            translation_filt(
                200.,
                Eigen::Array<double, 3, 1>::Constant(5.),   // min_cutoff
                Eigen::Array<double, 3, 1>::Constant(1),    // beta > 1
                Eigen::Array<double, 3, 1>::Constant(10.),   // d_cutoff
                Eigen::Array<double, 3, 1>::Zero(),          // zero
                Eigen::Array<double, 3, 1>::Ones(),          // one
                [](auto& in) { return in.abs(); }            // abs function
            ){

	this->delta_t = delta_t;

	this->T_imu_body = T_imu_body;
    this->prior_imu_bias = prior_imu_bias;
	// Instantiate IMU preintegration
	this->imu_preintegrated = new PreintegratedCombinedMeasurements(imu_preintegration_params, prior_imu_bias);

	// Initialize UWB RNG
	std::random_device rd; 
	std::mt19937 gen(rd());    
	this->uwb_rng = gen; 
	this->uwb_stdev = uwb_stdev;

	
    // Initialize noise models
    this->GT_noise_model = GT_noise_model;
    this->UWB_noise_model = UWB_noise_model;
    this->FakePrior_noise_model = FakePrior_noise_model;
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

void Tracker::init_state(json mes) {
	 // Find first SLAM pose, and use it to set GT prior.
    track.Ix = 0;
    track.Iv = 0;
    track.Ib = 0;

	Pose3 start_slam_pose; 
	Vector3 start_slam_velocity;
	double timestamp;

	get_GT_HTM(mes, start_slam_pose);
	get_V(mes, start_slam_velocity);

	Pose3 prior_pose = start_slam_pose * T_imu_body.inverse();
	// Pose3 prior_pose = start_slam_pose;

	// Velocity is computed using SLAM poses in the world frame.
	// Therefore all we should need to do, is rotate the velocity vector into the body frame.
	Pose3 pose_velocity(Rot3::Identity(), start_slam_velocity);
	Vector3 prior_velocity = (pose_velocity * T_imu_body.inverse()).translation();
	// Note: Had problems here with rotation matrices, I trust the pose matrix here though.


    vals.insert(X(track.Ix), prior_pose);
    vals.insert(V(track.Iv), prior_velocity);
    vals.insert(B(track.Ib), prior_imu_bias);

    graph->addPrior(X(track.Ix), prior_pose, GT_noise_model);
    graph->addPrior(V(track.Iv), prior_velocity, Velocity_noise_model);
    graph->addPrior(B(track.Ib), prior_imu_bias, Bias_noise_model);

    track.est_poses.push_back(prior_pose); // We'll take the estimate out of values and put it here.
    track.gt_poses.push_back(prior_pose);
	track.est_timestamps.push_back(timestamp);
    track.est_velocities.push_back(prior_velocity);
    track.constant_bias = prior_imu_bias;
	track.changing_bias = prior_imu_bias;

		// Add this key -> timestamp mapping to our map
	key_timestamps[X(track.Ix)] = (double)mes["t"];
	key_timestamps[V(track.Iv)] = (double)mes["t"];
	key_timestamps[B(track.Ib)] = (double)mes["t"];

    // Once priors have been inserted into graph and vals,
    // initialize isam with these estimates.
	if (use_smoother) {
		smoother->update(*graph, vals, key_timestamps);
	}
	else {
		isam->update(*graph, vals);
	}
	graph->resize(0);
	vals.clear();


    // Initialize our NavState
    prev_state = NavState(track.est_poses.back(), track.est_velocities.back());
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

		// report_estimate(proposed.pose(), mes_timestamp); // NOTE: CHANGE HERE
		report_estimate(result.at<Pose3>(X(track.Ix)), mes_timestamp);

		track.est_velocities.push_back(result.at<Vector3>(V(track.Iv)));
		est_velocity_vectors.push_back(Pose3(Rot3::Identity(), result.at<Vector3>(V(track.Iv))));

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

		// track.est_poses.push_back(result.at<Pose3>(X(track.Ix)));
		// track.est_timestamps.push_back(mes_timestamp);

		report_estimate(result.at<Pose3>(X(track.Ix)), mes_timestamp);

		track.est_velocities.push_back(result.at<Vector3>(V(track.Iv)));
		est_velocity_vectors.push_back(Pose3(Rot3::Identity(), result.at<Vector3>(V(track.Iv))));

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
	Pose3 gt_pose_slam;
	string usrname;
	get_GT_HTM(mes,gt_pose_slam);
	Pose3 gt_pose = gt_pose_slam * T_imu_body.inverse(); // Transform pose to the body frame.
	// Pose3 gt_pose = gt_pose_slam;
	track.gt_poses.push_back(gt_pose);
	track.gt_timestamps.push_back(mes["t"]);
	

	Vector3 slam_velocity;
	double timestamp = mes["t"];
	get_V(mes, slam_velocity);

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
	graph->add(PriorFactor<Pose3>(X(track.Ix), gt_pose, GT_noise_model));
	cout << "Added Prior factor " << graph->size() - 1 << endl;

	// Add velocity prior factor
	// Pose3 pose_velocity(Rot3::Identity(), slam_velocity);
	// Pose3 prior_velocity = (pose_velocity * T_imu_body.inverse());
	// graph->add(PriorFactor<Vector3>(V(track.Iv), prior_velocity.translation(), Velocity_noise_model));
	// postproc_velocity_vectors.push_back(pose_velocity); // Plot the velocity vector rotated into the body, into the world frame

	// Predict current state
	// NavState proposed = current_imu_preintegration->predict(prev_state, track.constant_bias);
	NavState proposed = current_imu_preintegration->predict(prev_state, track.changing_bias);

	// Insert initial values
	vals.insert(X(track.Ix), proposed.pose());
	vals.insert(V(track.Iv), proposed.v());
	// vals.insert(B(track.Ib), track.constant_bias);
	vals.insert(B(track.Ib), track.changing_bias);

    // Run iSAM
	if (use_smoother) { exec_smoother(proposed, (double)mes["t"], "GT", true); }
	else { exec_iSAM(proposed, (double)mes["t"], "GT", true); }
	

	// translation_filt.clear(); // clear filter
}

void Tracker::processSyntheticUWB(const json& mes, int& uwb_counter, double uwb_stdev)
{
	cout << "Processing synthetic range for : " << mes["t"] << endl;

	// Extract GT pose
	Pose3 gt_pose_slam;
	get_GT_HTM(mes, gt_pose_slam);
	Pose3 gt_pose = gt_pose_slam * T_imu_body.inverse(); // Transform pose to the body frame.
	// Pose3 gt_pose = gt_pose_slam;

	track.Ix++;
	track.Iv++;
	track.Ib++;

	// Add this key -> timestamp mapping to our map
	key_timestamps[X(track.Ix)] = (double)mes["t"];
	key_timestamps[V(track.Iv)] = (double)mes["t"];
	key_timestamps[B(track.Ib)] = (double)mes["t"];
	for (auto const &[id, tracking_] : anchors) {
		key_timestamps[AnchorKey(id)] = (double)mes["t"];
	}

	vector<string> ids = {"2", "3", "5"};
	for (string dst_user: ids){ 
		uwb_counter++;
		// string dst_user = ids[uwb_counter % 3];
		tracking& dst = anchors[dst_user];
		Point3 anchor_pos = dst.gt_poses.back().translation();
		Point3 user_pos = gt_pose.translation();

		// Ground truth range from GT poses
		double true_range = distance3(anchor_pos, user_pos); 

		std::normal_distribution<double> uwb_distribution(true_range, uwb_stdev);  // N(mean, stddev)
		double noised_range = uwb_distribution(uwb_rng);

		// Add UWB (range) factor
		graph->add(RangeFactor<Pose3, Pose3, double>(
			X(track.Ix), AnchorKey(dst_user), noised_range, UWB_noise_model));

		// Pose3 T_body_decawave(Rot3::Identity(), Vector3(-0.12, 0.015, -0.1));
		// graph->add(RangeFactorWithTransform<Pose3, Pose3, double>(
		// 	X(track.Ix), AnchorKey(dst_user), noised_range, UWB_noise_model, T_body_decawave));

		cout << "Added Range factor " << graph->size() - 1 << endl;
		cout << " True range " << true_range << " Noised range " << noised_range << " Noise " << uwb_stdev << endl;
	}

	// graph->add(PriorFactor<Pose3>(X(track.Ix), gt_pose, GT_noise_model));

	// Magnetometer Factor
	// N_body_frame = T_world_to_body * N_world_frame
	Pose3 N_world_frame(Rot3::Identity(), Vector3(0,1,0)); //Correct
	Pose3 N_world_frame_adjusted( Rot3::Identity(), gt_pose.translation() + Vector3(0,1,0));

	// Body -> Mag = World -> Mag * Inv(World -> Body)
	// Pose3 N_body_frame =  N_world_frame_adjusted * gt_pose.inverse();
	Vector3 N_body_frame = gt_pose.rotation().matrix().inverse() * Vector3(1, 0 ,0);
			// WHY DO I NEED TO INVERT THIS ROTATION? TODO: Use pose for this.
	double scale = 1; // Magnitude is 55k nT, or 55 muT - I think this is just in case your raw measurement is not already normalized?

	Point3 bias(1e-3, 1e-3, 1e-3);
	noiseModel::Diagonal::shared_ptr MAG_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);
	graph->add(MagPoseFactor<Pose3>(X(track.Ix), N_body_frame, scale, N_world_frame.translation(), bias, MAG_noise_model));
	// Should the world frame mag vector be aligned to 0,0,0?
	
	Vector3 measured = N_body_frame; // Normalize to see where the vector points relative to the GT pose.
	cout << " Synthetic vector " << measured.x() << " " << measured.y() << " " << measured.z() << " magnitude " << N_body_frame.norm() << endl;
	suwb_base_poses.push_back(gt_pose);
	mag_vectors.push_back(Pose3(Rot3::Identity(), N_body_frame));


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

void Tracker::processAssistedUWB(const json& mes, int& uwb_counter)
{
	cout << "Processing assisted range for t=" << mes["t"] << endl;

	// Extract GT pose
	Pose3 gt_pose_slam;
	get_GT_HTM(mes, gt_pose_slam);
	Pose3 gt_pose = gt_pose_slam * T_imu_body.inverse(); // body pose in world frame

	track.Ix++;
	track.Iv++;
	track.Ib++;
	uwb_counter++;

	// Add this key -> timestamp mapping to our map
	key_timestamps[X(track.Ix)] = (double)mes["t"];
	key_timestamps[V(track.Iv)] = (double)mes["t"];
	key_timestamps[B(track.Ib)] = (double)mes["t"];
	for (auto const &[id, tracking_] : anchors) {
		key_timestamps[AnchorKey(id)] = (double)mes["t"];
	}

	string dst_user = to_string((int)mes["id"]);

	Pose3 T_body_decawave(Rot3::Identity(), Vector3(-0.12, 0.015, -0.1));

	tracking& dst = anchors[dst_user];
	Point3 anchor_pos = (T_body_decawave * dst.gt_poses.back()).translation();
	Point3 user_pos = gt_pose.translation();


	double measured_range = (double)mes["range"];
	// Ground truth range from GT poses
	double true_range = distance3(anchor_pos, user_pos); 

	std::normal_distribution<double> uwb_distribution(true_range, uwb_stdev);  // N(mean, stddev)
	double noised_range = uwb_distribution(uwb_rng);

	// Add UWB (range) factor
	// graph->add(RangeFactor<Pose3, Pose3, double>(
	// 	X(track.Ix), AnchorKey(dst_user), measured_range, UWB_noise_model));

	// Math Source: https://www.sunnywale.com/uploadfile/2021/1230/DW1000%20User%20Manual_Awin.pdf sec 4.7.1
	double A = 121.74; // Source: https://github.com/AntonDmitriev1484/DecawaveMDEK1001-SNR-Firmware-Mod/blob/master/Decawave_firmware_mod/examples/ss_twr_init/ss_init_main.c
	double fp_power = 10 * log10( (pow((float)mes["firstpathamp1"], 2)
										+ pow((float)mes["firstpathamp2"], 2) 
										+ pow((float)mes["firstpathamp3"], 2))
										/ pow((float)mes["rxpreamcount"], 2)
									) - A;

	double rx_power = 10 * log10( (((float)mes["maxgrowthcir"]) * pow(2, 17))
									/ pow((float)mes["rxpreamcount"], 2)) - A;
	
	double snr = (float)mes["firstpathamp1"] / (float)mes["maxnoise"];
	double nlos_score = (rx_power - fp_power);

	double min_nlos = 7.5;
	double max_nlos = 15;
	bool nlos = nlos_score > min_nlos;
	double corrected_range = measured_range;
	double helmet_bias = 0.183; // 18.3cm

	// if (nlos) {
		double max_score = 15;
		double scale_factor = 3 * (nlos_score-min_nlos) / (max_nlos-min_nlos);
		// TODO: pick right function to go here, scale factor should be between 1 and 3.

		corrected_range =  (measured_range - (scale_factor*helmet_bias));
	// }



	graph->add(RangeFactorWithTransform<Pose3, Pose3, double>(
		X(track.Ix), AnchorKey(dst_user), corrected_range, UWB_noise_model, T_body_decawave));


	cout << "Added Range factor " << graph->size() - 1 << endl;
	cout << "UWB "<<mes["id"]<<". Synthetic Range " << true_range << ", Real Range " << measured_range << ", Corrected Range " << corrected_range << endl;
	cout << "UWB "<<mes["id"]<<". NLOS " << nlos_score << ", SNR " << snr << endl;

	// Magnetometer Factor
	// N_body_frame = T_world_to_body * N_world_frame
	Pose3 N_world_frame(Rot3::Identity(), Vector3(0,1,0)); //Correct
	Pose3 N_world_frame_adjusted( Rot3::Identity(), gt_pose.translation() + Vector3(0,1,0));

	// Body -> Mag = World -> Mag * Inv(World -> Body)
	// Pose3 N_body_frame =  N_world_frame_adjusted * gt_pose.inverse();
	Vector3 N_body_frame = gt_pose.rotation().matrix().inverse() * Vector3(1, 0 ,0);
			// WHY DO I NEED TO INVERT THIS ROTATION? TODO: Use pose for this.
	double scale = 1; // Magnitude is 55k nT, or 55 muT - I think this is just in case your raw measurement is not already normalized?

	Point3 bias(1e-3, 1e-3, 1e-3);
	noiseModel::Diagonal::shared_ptr MAG_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);
	graph->add(MagPoseFactor<Pose3>(X(track.Ix), N_body_frame, scale, N_world_frame.translation(), bias, MAG_noise_model));
	// Should the world frame mag vector be aligned to 0,0,0?
	
	Vector3 measured = N_body_frame; // Normalize to see where the vector points relative to the GT pose.
	cout << " Synthetic vector " << measured.x() << " " << measured.y() << " " << measured.z() << " magnitude " << N_body_frame.norm() << endl;
	suwb_base_poses.push_back(gt_pose);
	mag_vectors.push_back(Pose3(Rot3::Identity(), N_body_frame));


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


std::shared_ptr<PreintegratedCombinedMeasurements::Params> get_imu_preintegration_params(int ASCALE, int GSCALE) {


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


	std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = PreintegratedCombinedMeasurements::Params::MakeSharedU();
	// std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = std::make_shared<PreintegratedCombinedMeasurements::Params>(Vector3(0, -9.81, 0));
	imu_preintegration_params->accelerometerCovariance = continuous_time_accel_noise_cov;
	imu_preintegration_params->gyroscopeCovariance = continuous_time_gyro_noise_cov;

	imu_preintegration_params->biasAccCovariance = continuous_time_accel_bias_rw;
	imu_preintegration_params->biasOmegaCovariance = continuous_time_gyro_bias_rw;

	imu_preintegration_params->integrationCovariance = integration_cov;
	imu_preintegration_params->biasAccOmegaInt = initial_bias_cov;

	imu_preintegration_params->use2ndOrderCoriolis = false;

	return imu_preintegration_params;
}