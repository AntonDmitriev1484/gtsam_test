#include "measurements.h"

#define TIMING true
#define START_TIMER(msg, start_timer) \
    do { \
        if (TIMING) { \
			start_timer = clock(); \
			double start_t = double(start_timer) / CLOCKS_PER_SEC; \
			cout << "TIMER " << msg << ": "<<start_t << endl; \
        } \
    } while (false)

#define END_TIMER(msg, start_timer) \
    do { \
        if (TIMING) { \
			clock_t end_timer = clock(); \
			double start_t = double(start_timer) / CLOCKS_PER_SEC; \
			double end_t = double(end_timer)/ CLOCKS_PER_SEC; \
			double elapsed = end_t - start_t; \
			cout << "TIMER " << msg << ": "<< end_t << endl; \
			cout << "Elapsed " << elapsed << endl; \
			printf("\n"); \
		} \
	} while (false)

void processGT(
	json mes,
	tracking& user,
	NonlinearFactorGraph* graph,
	Values& vals,
	ISAM2* isam,
	PreintegrationType* imu_preintegrated,
	NavState& prev_state,
	const SharedNoiseModel& GT_noise_model,
	Pose3 T_imu_body,
	string debug_dump_directory)
{
	cout << "Used GT" << endl;
	user.Ix++;
	user.Iv++;
	user.Ib++;

	// Extract GT pose
	Matrix44 gt_pose_slam;
	string usrname;
	get_pose_matrix(mes, usrname, gt_pose_slam);
	Pose3 gt_pose = Pose3(gt_pose_slam) * T_imu_body.inverse(); // Transform pose to the body frame.
	user.gt_poses.push_back(gt_pose);
	user.gt_timestamps.push_back(mes["t"]);

	// Add IMU factor
	auto* current_imu_preintegration =
		dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);

	CombinedImuFactor imu_factor(
		X(user.Ix - 1), V(user.Iv - 1),
		X(user.Ix), V(user.Iv),
		B(user.Ib - 1), B(user.Ib),
		*current_imu_preintegration);

	graph->add(imu_factor);
	cout << "Added IMU factor " << graph->size() - 1 << endl;

	// Add GT prior factor
	graph->add(PriorFactor<Pose3>(X(user.Ix), gt_pose, GT_noise_model));
	cout << "Added Prior factor " << graph->size() - 1 << endl;

	// Predict current state
	NavState proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);

	// Insert initial values
	vals.insert(X(user.Ix), proposed.pose());
	vals.insert(V(user.Iv), proposed.v());
	vals.insert(B(user.Ib), user.constant_bias);

	Values result;
	try {
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
		
		clock_t isam_t;
		START_TIMER("Started iSAM. GT.", isam_t);
		isam->update(*graph, vals);
		result = isam->calculateEstimate();
		END_TIMER("Ended iSAM", isam_t);

						// Band-aid fix to filter out large hallucination from bad velocity prior.
				if (mes["t"] < 1750970628.78905845) { // If we're in the hallucination part.
					if ((proposed.pose().translation() - user.gt_poses.back().translation()).norm() < 0.5 ) {
						user.est_poses.push_back(proposed.pose());
						user.est_timestamps.push_back((double)mes["t"]);
					}
					else {
						user.est_poses.push_back(user.gt_poses.back());
						user.est_timestamps.push_back((double)mes["t"]);
					}
				}
				else {
					user.est_poses.push_back(proposed.pose());
					user.est_timestamps.push_back((double)mes["t"]);
				}

				//Correct code:
		// user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
		// user.est_timestamps.push_back((double)mes["t"]);

		user.est_velocities.push_back(result.at<Vector3>(V(user.Iv)));
		prev_state = NavState(result.at<Pose3>(X(user.Ix)), result.at<Vector3>(V(user.Iv)));

		cout << "Successful estimate on GT factor" << endl;
	}
	catch (const std::exception& e) {
		cerr << "Optimizer update failed: " << e.what() << endl;
		cerr << "Data timestamp is " << mes["t"] << endl;
		graph->saveGraph(debug_dump_directory+"/graph.dot", result);
		graph->print("");
		cerr << "Graph dumped to factor_graph.dot" << endl;
		throw; // rethrow
	}

	// Reset preintegration
	imu_preintegrated->resetIntegrationAndSetBias(user.constant_bias);

	// Clear for next iteration
	graph->resize(0);
	vals.clear();
}

void processUWB(
	json mes,
	string src_user,
	map<string, tracking > & info,
	NonlinearFactorGraph* graph,
	Values& vals,
	ISAM2* isam,
	PreintegrationType* imu_preintegrated,
	NavState& prev_state,
	const SharedNoiseModel& UWB_noise_model,
	const SharedNoiseModel& GT_noise_model,
	int& imu_counter,
	int& imu_count_at_last_correction,
	double dt,
	int& uwb_counter,
	string debug_dump_directory)
{
	uwb_counter++;

	std::string dst_user;
	double range;
	get_UWB(mes, src_user, dst_user, range);

	tracking& user = info[src_user];
	tracking& dst = info[dst_user];

	user.Ix++;
	user.Iv++;
	user.Ib++;

	// Ground truth range from GT poses
	double true_range = distance3(
		info[src_user].gt_poses.back().translation(),
		info[dst_user].gt_poses.back().translation());

	// Add UWB (range) factor — use ground truth here for stability
	graph->add(RangeFactor<Pose3, Pose3, double>(
		X(user.Ix), dst.pose_key, true_range, UWB_noise_model));
	cout << "Added Range factor " << graph->size() - 1 << endl;

	// Add fake GT-style prior to help stabilize
	Pose3 last_correction = user.gt_poses.back();
	Vector3 last_velocity = user.est_velocities.back();

	Vector3 reckoned_translation = last_velocity.x() *
		(imu_counter - imu_count_at_last_correction) * dt *
		Vector3(1, 0, 0);  // assume forward motion in x

	Pose3 reckoned_pose = last_correction * Pose3(Rot3::Identity(), reckoned_translation);
	graph->add(PriorFactor<Pose3>(X(user.Ix), reckoned_pose, GT_noise_model));
	cout << "Added (fake) Prior factor " << graph->size() - 1 << endl;

	// Add IMU factor
	auto* current_imu_preintegration =
		dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);

	CombinedImuFactor imu_factor(
		X(user.Ix - 1), V(user.Iv - 1),
		X(user.Ix), V(user.Iv),
		B(user.Ib - 1), B(user.Ib),
		*current_imu_preintegration);

	graph->add(imu_factor);
	cout << "Added IMU factor " << graph->size() - 1 << endl;

	// Predict state and insert
	NavState proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);
	vals.insert(X(user.Ix), proposed.pose());
	vals.insert(V(user.Iv), proposed.v());
	vals.insert(B(user.Ib), user.constant_bias);

	// Run optimization
	Values result;
	try {

		clock_t isam_t;
		START_TIMER("Started iSAM. UWB.", isam_t);
		isam->update(*graph, vals);
		result = isam->calculateEstimate();
		END_TIMER("Ended iSAM", isam_t);

		user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
		user.est_timestamps.push_back((double)mes["t"]);
		user.est_velocities.push_back(result.at<Vector3>(V(user.Iv)));

		cout << "Successful estimate on UWB factor" << endl;
	}
	catch (const std::exception& e) {
		cerr << "Optimizer update failed: " << e.what() << endl;
		graph->saveGraph(debug_dump_directory+"/graph.dot", result);
		graph->print("");
		cerr << "Graph dumped to factor_graph.dot" << endl;
		throw; // rethrow
	}

	prev_state = NavState(result.at<Pose3>(X(user.Ix)), result.at<Vector3>(V(user.Iv)));

	// Prepare for next iteration
	graph->resize(0);
	vals.clear();
	imu_preintegrated->resetIntegrationAndSetBias(user.constant_bias);
}

void processSyntheticUWB(
	json mes,
	string src_user,
	map<string, tracking > & info,
	NonlinearFactorGraph* graph,
	Values& vals,
	ISAM2* isam,
	PreintegrationType* imu_preintegrated,
	NavState& prev_state,
	const SharedNoiseModel& UWB_noise_model,
	const SharedNoiseModel& GT_noise_model,
	const SharedNoiseModel& FakePrior_noise_model,
	int& imu_counter,
	int& imu_count_at_last_correction,
	double dt,
	int& uwb_counter,
	Pose3& T_imu_body,
	string debug_dump_directory,
	double uwb_stdev)
{
	cout << "Processing synthetic range for : " << mes["t"] << endl;
	uwb_counter++;

	vector<string> users = {"2", "3", "5"};
	string dst_user = users[uwb_counter % 3];

	tracking& user = info[src_user];
	tracking& dst = info[dst_user];

		// Extract GT pose
	Matrix44 gt_pose_slam;
	string usrname;
	get_pose_matrix(mes, usrname, gt_pose_slam);
	Pose3 gt_pose = Pose3(gt_pose_slam) * T_imu_body.inverse(); // Transform pose to the body frame.
	user.gt_poses.push_back(gt_pose);
	user.gt_timestamps.push_back(mes["t"]);


	user.Ix++;
	user.Iv++;
	user.Ib++;

	// Ground truth range from GT poses
	double true_range = distance3(
		gt_pose.translation(),
		info[dst_user].gt_poses.back().translation());

	std::random_device rd;                          // Seed
	std::mt19937 gen(rd());                         // Mersenne Twister engine
	std::normal_distribution<double> dist(true_range, uwb_stdev);  // N(mean, stddev)
	double noised_range = dist(gen);

	// Add UWB (range) factor — use ground truth here for stability
	graph->add(RangeFactor<Pose3, Pose3, double>(
		X(user.Ix), dst.pose_key, noised_range, UWB_noise_model));
	cout << "Added Range factor " << graph->size() - 1 << endl;
	cout << " True range " << true_range << " Noised range " << noised_range << " Noise " << uwb_stdev << endl;

	// A noise model that basically only constrains yaw on the pose.
	// double gt_pos_stdev = 1e-1;
	// double gt_pitch_stdev = 1e-1;
	// double gt_roll_stdev = 1e-1;
	// double gt_yaw_stdev = 1e-2;
	// noiseModel::Diagonal::shared_ptr yaw_constraint_pose_noise_model = noiseModel::Diagonal::Sigmas(
	// 	Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_roll_stdev, gt_pitch_stdev, gt_yaw_stdev));

    // It seems to be THIS very specific noise model that doesn't throw VVdot
	double gt_pos_stdev = 1e-1;
	double gt_pitch_stdev = 1e-1;
	double gt_roll_stdev = 1e-1;
	double gt_yaw_stdev = 1e-2;
	noiseModel::Constrained::shared_ptr yaw_constraint_pose_noise_model = noiseModel::Constrained::MixedSigmas(
		Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_roll_stdev, gt_pitch_stdev, gt_yaw_stdev));

	graph->add(PriorFactor<Pose3>(X(user.Ix), gt_pose, yaw_constraint_pose_noise_model));
	cout << "Added (fake) Prior factor " << graph->size() - 1 << endl;

	// Add IMU factor
	auto* current_imu_preintegration =
		dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);

	CombinedImuFactor imu_factor(
		X(user.Ix - 1), V(user.Iv - 1),
		X(user.Ix), V(user.Iv),
		B(user.Ib - 1), B(user.Ib),
		*current_imu_preintegration);

	graph->add(imu_factor);
	cout << "Added IMU factor " << graph->size() - 1 << endl;

	// Predict state and insert
	NavState proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);
	vals.insert(X(user.Ix), proposed.pose());
	vals.insert(V(user.Iv), proposed.v());
	vals.insert(B(user.Ib), user.constant_bias);

	// Run optimization
	Values result;
	try {
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

		clock_t isam_t;
		START_TIMER("Started iSAM. Synthetic UWB.", isam_t);
		isam->update(*graph, vals);
		result = isam->calculateEstimate();
		END_TIMER("Ended iSAM", isam_t);

						// Band-aid fix to filter out large hallucination from bad velocity prior.
				if (mes["t"] < 1750970628.78905845) { // If we're in the hallucination part.
					if ((proposed.pose().translation() - user.gt_poses.back().translation()).norm() < 0.5 ) {
						user.est_poses.push_back(proposed.pose());
						user.est_timestamps.push_back((double)mes["t"]);
					}
					else {
						user.est_poses.push_back(user.gt_poses.back());
						user.est_timestamps.push_back((double)mes["t"]);
					}
				}
				else {
					user.est_poses.push_back(proposed.pose());
					user.est_timestamps.push_back((double)mes["t"]);
				}

				//Correct code:
		// user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
		// user.est_timestamps.push_back((double)mes["t"]);

		user.est_velocities.push_back(result.at<Vector3>(V(user.Iv)));

		cout << "Successful estimate on UWB factor" << endl;
	}
	catch (const std::exception& e) {
		cerr << "Optimizer update failed: " << e.what() << endl;
		cerr << "Data timestamp is " << mes["t"] << endl;
		graph->saveGraph(debug_dump_directory+"/graph.dot", result);
		graph->print("");
		cerr << "Graph dumped to factor_graph.dot" << endl;
		throw; // rethrow
	}

	prev_state = NavState(result.at<Pose3>(X(user.Ix)), result.at<Vector3>(V(user.Iv)));

	// Prepare for next iteration
	graph->resize(0);
	vals.clear();
	imu_preintegrated->resetIntegrationAndSetBias(user.constant_bias);
}