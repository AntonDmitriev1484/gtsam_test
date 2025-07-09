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
            const Pose3& T_imu_body,
            const double delta_t,
            const SharedNoiseModel& GT_noise_model,
            const SharedNoiseModel& UWB_noise_model,
            const SharedNoiseModel& FakePrior_noise_model,
            const SharedNoiseModel& Velocity_noise_model,
            const SharedNoiseModel& Bias_noise_model,
			std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params,
            const imuBias::ConstantBias prior_imu_bias,
            const string& debug_dir) {

	this->delta_t = delta_t;
	this->T_imu_body = T_imu_body;

    // TODO: this->prior_velocity <= make this initializable to non-zero after post_process
    this->prior_imu_bias = prior_imu_bias;

    // Initialize noise models
    this->GT_noise_model = GT_noise_model;
    this->UWB_noise_model = UWB_noise_model;
    this->FakePrior_noise_model = FakePrior_noise_model;
    this->Velocity_noise_model = Velocity_noise_model;
    this->Bias_noise_model = Bias_noise_model;

	// Instantiate IMU preintegration
	this->imu_preintegrated = new PreintegratedCombinedMeasurements(imu_preintegration_params, prior_imu_bias);

    // Instantiate graph
    this->graph = new NonlinearFactorGraph();

    // Values initialized in class
    // Tracking initialized in class
    
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

void Tracker::init(json sensor_stream) {

    // Find first SLAM pose, and use it to set GT prior.
    track.Ix = 0;
    track.Iv = 0;
    track.Ib = 0;

	int pose_num = 0;
	this->mes_start = 0;
    Pose3 find_first_gt_pose;
    int mes_idx = 0;
    double start_timestamp = 0;

    for (json mes : sensor_stream) {
        if (mes["type"] == "slam_pose") {
            start_timestamp = (double) mes["t"];
            mes_start = mes_idx;
            Matrix44 gt_pose_world;
            string usrname;
            get_pose_matrix(mes, usrname, gt_pose_world);

            //Pose3 gt_pose = slam_to_world * gt_pose_slam;
            Pose3 asdf(gt_pose_world);
            Pose3 gt_pose = asdf * T_imu_body.inverse();

            find_first_gt_pose = gt_pose;
            break;
        }
        mes_idx ++;
    }

    // <<< Problem behind start spaghetti is here >>>
    Pose3 start_pose = find_first_gt_pose;
    Vector3 prior_velocity(-0.3, -0.8, 0); // I'm assuming this should be in the world frame?

    vals.insert(X(track.Ix), start_pose);
    vals.insert(V(track.Iv), prior_velocity);
    vals.insert(B(track.Ib), prior_imu_bias);

    graph->addPrior(X(track.Ix), start_pose, GT_noise_model);
    graph->addPrior(V(track.Iv), prior_velocity, Velocity_noise_model);
    graph->addPrior(B(track.Ib), prior_imu_bias, Bias_noise_model);

    track.est_poses.push_back(start_pose); // We'll take the estimate out of values and put it here.
    track.est_timestamps.push_back(start_timestamp);
    track.est_velocities.push_back(prior_velocity);
    track.constant_bias = prior_imu_bias;

    // Once priors have been inserted into graph and vals,
    // initialize isam with these estimates.
    isam->update(*graph, vals);
	graph->resize(0);
	vals.clear();

    // Initialize our NavState
    prev_state = NavState(track.est_poses.back(), track.est_velocities.back());

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

	// TODO: I have no idea which one of these is right but they all seem to work?

	// Rot3 r = T_imu_body.inverse().rotation();
	// Vector3 prior_velocity = r.matrix().inverse() * (start_slam_velocity);

	Rot3 r = T_imu_body.inverse().rotation();
	Vector3 prior_velocity = r.matrix() * (start_slam_velocity);

    vals.insert(X(track.Ix), prior_pose);
    vals.insert(V(track.Iv), prior_velocity);
    vals.insert(B(track.Ib), prior_imu_bias);

    graph->addPrior(X(track.Ix), prior_pose, GT_noise_model);
    graph->addPrior(V(track.Iv), prior_velocity, Velocity_noise_model);
    graph->addPrior(B(track.Ib), prior_imu_bias, Bias_noise_model);

    track.est_poses.push_back(prior_pose); // We'll take the estimate out of values and put it here.
    track.est_timestamps.push_back(timestamp);
    track.est_velocities.push_back(prior_velocity);
    track.constant_bias = prior_imu_bias;

    // Once priors have been inserted into graph and vals,
    // initialize isam with these estimates.
    isam->update(*graph, vals);
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

				//Correct code:
		track.est_poses.push_back(result.at<Pose3>(X(track.Ix)));
		track.est_timestamps.push_back(mes_timestamp);

		track.est_velocities.push_back(result.at<Vector3>(V(track.Iv)));
		prev_state = NavState(result.at<Pose3>(X(track.Ix)), result.at<Vector3>(V(track.Iv)));

		cout << "Successful estimate on "<<msg<<" factor" << endl;
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
	Matrix44 gt_pose_slam;
	string usrname;
	get_pose_matrix(mes, usrname, gt_pose_slam);
	Pose3 gt_pose = Pose3(gt_pose_slam) * T_imu_body.inverse(); // Transform pose to the body frame.
	track.gt_poses.push_back(gt_pose);
	track.gt_timestamps.push_back(mes["t"]);

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

	// Add GT prior factor
	graph->add(PriorFactor<Pose3>(X(track.Ix), gt_pose, GT_noise_model));
	cout << "Added Prior factor " << graph->size() - 1 << endl;

	// Predict current state
	NavState proposed = current_imu_preintegration->predict(prev_state, track.constant_bias);

	// Insert initial values
	vals.insert(X(track.Ix), proposed.pose());
	vals.insert(V(track.Iv), proposed.v());
	vals.insert(B(track.Ib), track.constant_bias);

    // Run iSAM
	exec_iSAM(proposed, (double)mes["t"], "GT", true);

	// Clear for next iteration
	graph->resize(0);
	vals.clear();
	imu_preintegrated->resetIntegrationAndSetBias(track.constant_bias);
}

void Tracker::processSUWB(const json& mes, int& uwb_counter, double uwb_stdev)
{
	cout << "Processing synthetic range for : " << mes["t"] << endl;

		// Extract GT pose
		Matrix44 gt_pose_slam;
		string usrname;
		get_pose_matrix(mes, usrname, gt_pose_slam);
		Pose3 gt_pose = Pose3(gt_pose_slam) * T_imu_body.inverse(); // Transform pose to the body frame.
		track.gt_poses.push_back(gt_pose);
		track.gt_timestamps.push_back((double)mes["t"]);

	

		track.Ix++;
		track.Iv++;
		track.Ib++;

	vector<string> ids = {"2", "3", "5"};
	// for (string dst_user: ids){ 

		string dst_user = ids[uwb_counter % 3];
		tracking& dst = anchors[dst_user];

		// Ground truth range from GT poses
		double true_range = distance3(
			gt_pose.translation(),
			dst.gt_poses.back().translation()); 

		std::random_device rd;                          // Seed TODO set this up in constructor
		std::mt19937 gen(rd());                         // Mersenne Twister engine
		std::normal_distribution<double> dist(true_range, uwb_stdev);  // N(mean, stddev)
		double noised_range = dist(gen);

		// Add UWB (range) factor — use ground truth here for stability
		graph->add(RangeFactor<Pose3, Pose3, double>(
			X(track.Ix), AnchorKey(dst_user), noised_range, UWB_noise_model));
		cout << "Added Range factor " << graph->size() - 1 << endl;
		cout << " True range " << true_range << " Noised range " << noised_range << " Noise " << uwb_stdev << endl;
	// }

	
	// graph->add(PriorFactor<Pose3>(X(track.Ix), gt_pose, yaw_constraint_pose_noise_model));
	// cout << "Added (fake) Prior factor " << graph->size() - 1 << endl;

	// Magnetometer Factor
	// N_body_frame = T_world_to_body * N_world_frame
	Vector3 N_world_frame(0,1,0);
	Vector3 N_body_frame = gt_pose.rotation().matrix() * N_world_frame;

	double scale = 1; // Magnitude is 55k nT, or 55 muT - I think this is just in case your raw measurement is not already normalized?
	Point3 measured = N_body_frame * scale;
	Point3 bias(1e-3, 1e-3, 1e-3);
	noiseModel::Diagonal::shared_ptr MAG_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);

	graph->add(MagPoseFactor<Pose3>(X(track.Ix), measured, scale, N_world_frame, bias, MAG_noise_model));

	suwb_base_poses.push_back(gt_pose);
	mag_vectors.push_back(Pose3(Rot3::Identity(), gt_pose.rotation().inverse().matrix() * N_body_frame));
	
	
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
	NavState proposed = current_imu_preintegration->predict(prev_state, track.constant_bias);
	vals.insert(X(track.Ix), proposed.pose());
	vals.insert(V(track.Iv), proposed.v());
	vals.insert(B(track.Ib), track.constant_bias);

	// Run optimization
	exec_iSAM(proposed, (double)mes["t"], "SynthUWB", true);

	// Prepare for next iteration
	graph->resize(0);
	vals.clear();
	imu_preintegrated->resetIntegrationAndSetBias(track.constant_bias);
}

