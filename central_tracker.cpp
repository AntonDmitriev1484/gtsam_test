#include "central_tracker.h"

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

vector<string> get_users(string data_dir) {
        // Regex to match subdirectory names like "nucX"
    std::regex nuc_pattern("^nuc(\\d+)$");
    vector<string> nuc_ids;

    for (const auto& entry : std::filesystem::directory_iterator(data_dir)) {
            if (entry.is_directory()) {
                std::string dirname = entry.path().filename().string();
                std::smatch match;
                if (std::regex_match(dirname, match, nuc_pattern)) {
                    // match[1] contains the "X" part
                    nuc_ids.push_back(match[1].str());
                }
            }
        }

    return nuc_ids;
}

std::map<string, json> get_imu_params(const vector<string>& user_ids, string data_dir) {
    std::map<string, json> imu_parameters;
    for (string id : user_ids) {
		string imu_param_path = data_dir +"/nuc" + id +"/imu.json";
        //std::filesystem::path imu_param_path = std::filesystem::path(data_dir) / ("nuc" + id) / "imu.json";
        std::ifstream f(imu_param_path);
        json imu_json;
        f >> imu_json;  // load JSON
        imu_parameters[id] = imu_json;
    }
    return imu_parameters;
}


// Assuming we have already called
// get_beacon_info(tracker.anchors, json::parse(beacon_fs));
void CentralTracker::init_anchor(string id){
    Pose3 prior_beacon_pose(anchors[id].gt_poses[0]);
    vals.insert(AnchorKey(id), prior_beacon_pose);
    graph->add(NonlinearEquality<Pose3>(AnchorKey(id), prior_beacon_pose));
}

void CentralTracker::init_anchors(json anchor_json) {
	get_beacon_info(anchors, anchor_json); // Reads raw pose data into map
	for (auto& [id, anchor_track]: anchors){
		init_anchor(id); // Uses anchor pose to set pose prior, and inserts into values.
	}
}

CentralTracker::CentralTracker(
    const bool use_smoother,
    const double smoother_lag,
    const string data_dir,
    const string out_dir,
    double uwb_synth_stdev,
	json transform_json,
	json anchor_json
    ) 
    {

    // Instantiate graph
    this->graph = new NonlinearFactorGraph();
    // Values initialized in class
	this->use_smoother = use_smoother;
	if (use_smoother) {
		ISAM2Params isam_params;
		isam_params.factorization = ISAM2Params::QR;
		isam_params.relinearizeThreshold = 0.01;
		isam_params.relinearizeSkip = 1; // More informed optimization at the cost of more computing.
		isam_params.enableDetailedResults = true;
		this->smoother = new IncrementalFixedLagSmoother(smoother_lag, isam_params);
	}
	else {
    	// Instantiate iSAM
		ISAM2Params isam_params;
		isam_params.factorization = ISAM2Params::QR;
		isam_params.relinearizeThreshold = 0.01;
		isam_params.relinearizeSkip = 1; // More informed optimization at the cost of more computing.
		isam_params.enableDetailedResults = true;
		ISAM2DoglegParams dogleg;
		isam_params.optimizationParams = dogleg;
		this->isam = new ISAM2(isam_params);
	}
	// key_to_ts gets initialized in class


    // Read in data
    vector<string> user_ids = get_users(data_dir);
    std::map<string, json> imu_params = get_imu_params(user_ids, data_dir);


    // Initializing Trackers

    // --- Noise Models ---

	// VIO noise model
	double vio_ori_stdev = 0.175;
	double vio_pos_stdev = 0.2;
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));

	// UWB noise model
	double uwb_stdev = 0.05;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	// GT noise model - (use to define pose prior)
	double gt_pos_stdev = 1e-2;
	double gt_ori_stdev = 1e-2;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));
	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 1e-2);
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-2);
	
    
    Pose3 T_body_to_imu;
	get_pose_from_HTM(transform_json["T_body_to_imu"], T_body_to_imu);

    Pose3 T_body_to_decawave;
	get_pose_from_HTM(transform_json["T_body_to_decawave"], T_body_to_decawave);

	bool use_filter = false;

    // Initialize every user
    for (string id: user_ids) {

        // All needs to be read out of a json file.
        json& imu_param = imu_params.at(id);
        double dt = 1/200.0;
		std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params 
            = get_imu_preintegration_params(1, 10);
        imuBias::ConstantBias prior_imu_bias;
        imu_preintegration_params->setBodyPSensor(T_body_to_imu);

        // Create an output directory for this tracker
        string out = out_dir + "/nuc"+id+"/";
        if (!std::filesystem::exists(out)) {
				std::filesystem::create_directories(out);
				std::cout << "Directory created: " << out << std::endl;
		}
        string debug_dir = out_dir+"/debug";
        ofstream estimated_trajectory_fs(out + "/est.txt");
        ofstream slam_trajectory_fs(out+"/slam.txt");
        ofstream estimated_timestamp_fs(out+"/est_timestamps.txt");
        ofstream log_dump_fs(out + "/log_dump.txt");

            // TODO: Modify tracker so that you pass a graph and optimizer pointer.
        Tracker t(
            id, T_body_to_imu, T_body_to_decawave, 
            dt, use_filter, uwb_synth_stdev, 
            GT_noise_model, UWB_noise_model, VIO_pose_noise_model, 
            prior_velocity_noise_model, prior_bias_noise_model,
            imu_preintegration_params, prior_imu_bias,
            debug_dir);
        
        t.estimated_trajectory_fs = &estimated_trajectory_fs;
        t.slam_trajectory_fs = &slam_trajectory_fs;

        std::pair<string, Tracker> pair(id, t);
        this->users.insert(pair);
	}

    this->init_anchors(anchor_json);


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

		write_trajectory_TUM_format( track.est_poses, track.est_timestamps, *estimated_trajectory_fs, T_body_to_imu);
		estimated_trajectory_fs->close();

		write_trajectory_TUM_format( track.gt_poses, track.gt_timestamps, *slam_trajectory_fs, T_body_to_imu);
		slam_trajectory_fs->close();
		throw; // rethrow
	}
}