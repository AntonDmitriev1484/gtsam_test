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

Tracker::Tracker(const std::string& id,
                 const Pose3 T_body_to_imu,
                 const Pose3 T_body_to_decawave,
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
                 const std::string& debug_dir)
    : delta_t(delta_t),
      T_body_to_imu(T_body_to_imu),
      T_body_to_decawave(T_body_to_decawave),
      prior_imu_bias(prior_imu_bias),
      imu_preintegrated(new PreintegratedCombinedMeasurements(imu_preintegration_params, prior_imu_bias)),
      uwb_rng(std::mt19937(std::random_device{}())),
      uwb_stdev(uwb_stdev),
      GT_noise_model(GT_noise_model),
      UWB_noise_model(UWB_noise_model),
      FakePrior_noise_model(FakePrior_noise_model),
      Velocity_noise_model(Velocity_noise_model),
      Bias_noise_model(Bias_noise_model),
      graph(new NonlinearFactorGraph()),
      use_filter(use_filter),
      use_smoother(use_smoother),
      translation_filt(
          200.,
          Eigen::Array<double, 3, 1>::Constant(5.),   // min_cutoff
          Eigen::Array<double, 3, 1>::Constant(1.),   // beta > 1
          Eigen::Array<double, 3, 1>::Constant(10.),  // d_cutoff
          Eigen::Array<double, 3, 1>::Zero(),         // zero
          Eigen::Array<double, 3, 1>::Ones(),         // one
          [](auto& in) { return in.abs(); }           // abs function
      ) 
{

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
    track.Ix = 0;
    track.Iv = 0;
    track.Ib = 0;

	Pose3 start_slam_pose; 
	Vector3 start_slam_velocity;
	double timestamp;

	// velocity and body pose are computed from body poses in the world frame
	get_pose_from_HTM(mes["T_body_world"], start_slam_pose);
	get_V(mes, start_slam_velocity); // velocity

	Rot3 rot_imu_to_body = T_body_to_imu.rotation().inverse();
	Pose3 prior_pose = start_slam_pose;

	Pose3 pose_velocity(Rot3::Identity(), start_slam_velocity);
	Vector3 prior_velocity = start_slam_velocity;


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

void Tracker::processIMU(const json& mes) {

		// Add IMU measurement
		Vector3 accel;
		Vector3 gyro;
		get_IMU(mes, accel, gyro);

		imu_preintegrated->integrateMeasurement(accel, gyro, delta_t);
		imu_counter++;


		cout << "Preintegration on at " << mes["t"] << " a: " << accel.x() << " " << accel.y() << " " << accel.z() << ", g: " << gyro.x() << " " << gyro.y() << " " << gyro.z() << endl;

		// Just for plotting at IMU frequency
		PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);
		auto proposed = current_imu_preintegration->predict(prev_state, track.changing_bias);
		report_estimate(proposed.pose(), mes["t"]);

		for (json mes : pose_buffer) {
			processSLAM(mes);
			imu_count_at_last_correction = imu_counter;
			imu_count_at_last_imu_factor = imu_counter;
			gt_counter++;
		}
		pose_buffer.clear();

		for (json mes : range_buffer) {
			// Note: axed synth UWB for now
			processAssistedUWB(mes);
		}
		range_buffer.clear();
}



void Tracker::processSLAM(const json& mes)
{
	if (imu_counter == imu_count_at_last_imu_factor) {
		// Pass this measurement and buffer it until the next IMU becomes available
		cout << " Skipped SLAM pose " << endl;
		pose_buffer.push_back(mes);
		return;
	}

	cout << "Used GT" << endl;
	track.Ix++;
	track.Iv++;
	track.Ib++;

	// Extract GT pose
	Pose3 T_world_to_body;
	string usrname;
	// Notation; T_body_world = T_world_to_body
	get_pose_from_HTM(mes["T_body_world"],T_world_to_body);

	// Equivalent of saying: T_world_to_body = T_imu_to_body * T_world_to_imu
	// Pose3 gt_pose = T_world_to_imu.compose(T_body_to_imu.inverse());
	Pose3 gt_pose = T_world_to_body;

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
	graph->add(PriorFactor<Pose3>(X(track.Ix), gt_pose, GT_noise_model));
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
	

	imu_count_at_last_correction = imu_counter;
	imu_count_at_last_imu_factor = imu_counter;
	gt_counter++;

	// translation_filt.clear(); // clear filter
}

void Tracker::processSyntheticUWB(const json& mes, int& uwb_counter, double uwb_stdev)
{
	cout << "Processing synthetic range for : " << mes["t"] << endl;

		// Extract GT pose
	Pose3 T_world_to_body;
	get_pose_from_HTM(mes["T_body_world"], T_world_to_body);
	Pose3 gt_pose = T_world_to_body;

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

	bool USE_TRILATERATION = false;

	vector<string> ids = {"2", "3", "5"}; // Trilateration: Get a range to all anchors
	vector<string> used_ids = {ids[uwb_counter % 3] }; // Default: Get a range to a single anchor, round robin
	if (USE_TRILATERATION) used_ids = ids;

	for (string dst_user: used_ids){ 
		uwb_counter++;

		tracking& dst = anchors[dst_user];
		Point3 anchor_pos = dst.gt_poses.back().translation();

		Point3 user_antenna_pos = (gt_pose.compose(T_body_to_decawave)).translation();

		// Ground truth range from GT poses
		double true_range = distance3(anchor_pos, user_antenna_pos); 

		std::normal_distribution<double> uwb_distribution(true_range, uwb_stdev);  // N(mean, stddev)
		double noised_range = uwb_distribution(uwb_rng);

		graph->add(RangeFactorWithTransform<Pose3, Pose3, double>(
			X(track.Ix), AnchorKey(dst_user), noised_range, UWB_noise_model, T_body_to_decawave));

		cout << "Added Range factor " << graph->size() - 1 << endl;
		cout << " True range " << true_range << " Noised range " << noised_range << " Noise " << uwb_stdev << endl;
	}


	// Magnetometer Factor
	Vector3 N_world_frame = Vector3(1,0,0);
	Vector3 N_body_frame = gt_pose.rotation() * N_world_frame; // Seems inverse or not makes no difference here.
	double scale = 1; // Magnitude is 55k nT, or 55 muT - I think this is just in case your raw measurement is not already normalized?

	Point3 bias(1e-3, 1e-3, 1e-3);
	noiseModel::Diagonal::shared_ptr MAG_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);
	graph->add(MagPoseFactor<Pose3>(X(track.Ix), N_body_frame, scale, N_world_frame, bias, MAG_noise_model));
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

void Tracker::processAssistedUWB(const json& mes)
{

	if (imu_counter == imu_count_at_last_correction) { 
		range_buffer.push_back(mes);
		return; 
	}

	cout << "Processing assisted range for t=" << mes["t"] << endl;

	// Extract GT pose
	Pose3 T_world_to_body;
	get_pose_from_HTM(mes["T_body_world"], T_world_to_body);
	Pose3 gt_pose = T_world_to_body;

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

	tracking& dst = anchors[dst_user];
	Point3 anchor_pos = dst.gt_poses.back().translation();
	Point3 user_antenna_pos = (gt_pose.compose(T_body_to_decawave)).translation();

	double measured_range = (double)mes["range"];
	// Ground truth range from GT poses
	double true_range = distance3(anchor_pos, user_antenna_pos); 

	std::normal_distribution<double> uwb_distribution(true_range, uwb_stdev);  // N(mean, stddev)
	double noised_range = uwb_distribution(uwb_rng);

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
	double smoothed_nlos_score = nlos_score;
	int NLOS_SCORE_WINDOW_SIZE = 20;

	if (nlos_score_window.size() > NLOS_SCORE_WINDOW_SIZE) {
		nlos_score_window.erase(nlos_score_window.begin());
		nlos_score_window.push_back(nlos_score);
		smoothed_nlos_score = 0;
		for (double s: nlos_score_window) {
			smoothed_nlos_score += s;
		}
		smoothed_nlos_score /= NLOS_SCORE_WINDOW_SIZE;
	}
	else {
		nlos_score_window.push_back(nlos_score);
	}


	nlos_score = smoothed_nlos_score;

	double min_nlos = 10;
	double max_nlos = 15;
	bool nlos = nlos_score > min_nlos;
	double corrected_range = measured_range;
	double helmet_bias = 0.183; // 18.3cm
	// if (nlos) {
	// 	double scale_factor = 3 * (nlos_score-min_nlos) / (max_nlos-min_nlos);
	// 	cout << "scale factor: " << scale_factor << endl;
	// 	corrected_range =  (measured_range - ((scale_factor+1)*helmet_bias));
	// }


	graph->add(RangeFactorWithTransform<Pose3, Pose3, double>(
		X(track.Ix), AnchorKey(dst_user), measured_range, UWB_noise_model, T_body_to_decawave));


	cout << "Added Range factor " << graph->size() - 1 << endl;
	cout << "UWB "<<mes["id"]<<". Synthetic Range " << true_range << ", Real Range " << measured_range << ", Corrected Range " << corrected_range << endl;
	cout << "UWB "<<mes["id"]<<". NLOS " << nlos_score << ", SNR " << snr << endl;

	// Synthetic Magnetometer Factor

	Vector3 N_world_frame = Vector3(1,0,0);
	Vector3 N_body_frame = gt_pose.rotation() * N_world_frame; // Seems inverse or not makes no difference here.
	double scale = 1; // Magnitude is 55k nT, or 55 muT - I think this is just in case your raw measurement is not already normalized?

	Point3 bias(1e-3, 1e-3, 1e-3);
	noiseModel::Diagonal::shared_ptr MAG_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);
	graph->add(MagPoseFactor<Pose3>(X(track.Ix), N_body_frame, scale, N_world_frame, bias, MAG_noise_model));
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


	imu_count_at_last_imu_factor = imu_counter;
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

void Tracker::write_results() {
		// Before writing files for evluation, need to be able to transform all
	// body poses in world frame to slambody (cam1) poses in world frame.

	// TODO: Commenting out for now, so I can fix other compiler errors
	// Pose3 T_imu_to_cam1;
	// get_pose_from_HTM(transforms["T_imu_to_cam1"], T_imu_to_cam1);

	// Pose3 T_body_to_sbody_in_world = T_body_to_imu.compose(T_imu_to_cam1);
	// Pose3& out_transform = T_body_to_sbody_in_world;

	// write_trajectory_TUM_format( t.track.est_poses, t.track.est_timestamps, estimated_trajectory_fs, out_transform);
	// estimated_trajectory_fs.close();

	// write_trajectory_TUM_format( t.track.gt_poses, t.track.gt_timestamps, slam_trajectory_fs, out_transform);
	// slam_trajectory_fs.close();

	// write_timestamps( t.track.est_poses, t.track.est_timestamps, estimated_timestamp_fs);
	// estimated_timestamp_fs.close();


	// cout << "Dumping magnetometer and velocity vectors for visual debug" << endl;
	
	// ofstream suwb_base_poses_fs(out_dir + "/suwb_base_poses.txt");
	// write_trajectory_KITTI_format( t.suwb_base_poses, suwb_base_poses_fs);
	// suwb_base_poses_fs.close();

	// ofstream gt_base_poses_fs(out_dir + "/gt_base_poses.txt");
	// write_trajectory_KITTI_format( t.track.gt_poses, gt_base_poses_fs);
	// gt_base_poses_fs.close();

	// ofstream mag_vectors_fs(out_dir + "/mag_vectors_fs.txt");
	// write_trajectory_KITTI_format( t.mag_vectors, mag_vectors_fs);
	// mag_vectors_fs.close();

	// ofstream postproc_velocity_fs(out_dir + "/vel_vectors.txt");
	// write_trajectory_KITTI_format( t.postproc_velocity_vectors, postproc_velocity_fs);
	// postproc_velocity_fs.close();


	// NOTE: THIS WILL CHANGE FOR EACH DATASET duration

	// double duration_s = 45;
	// cout << " Applied " << uwb_counter << " uwb measurements for "<< duration_s<< " seconds of data " << endl;
	// double f_uwb = uwb_counter /duration_s;
	// cout << " UWB frequency in the graph is " << f_uwb << endl;

	// cout << " Applied " << gt_counter << " slam measurements for "<< duration_s<< " seconds of data " << endl;
	// double f_gt = gt_counter /duration_s;
	// cout << " GT frequency in the graph is " << f_gt << endl;

}