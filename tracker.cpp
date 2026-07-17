#include "tracker.h"

#define TIMING true
#define START_TIMER(msg, start_timer) \
    do { \
        if (TIMING) { \
			start_timer = clock(); \
			double start_t = double(start_timer) / CLOCKS_PER_SEC; \
			log_fs << "TIMER | " << msg << " | "<<start_t << endl; \
        } \
    } while (false)

#define END_TIMER(msg, start_timer) \
    do { \
        if (TIMING) { \
			clock_t end_timer = clock(); \
			double start_t = double(start_timer) / CLOCKS_PER_SEC; \
			double end_t = double(end_timer)/ CLOCKS_PER_SEC; \
			double elapsed = end_t - start_t; \
			log_fs << "TIMER | " << msg << " | "<< end_t << endl; \
			log_fs << "TIMER Elapsed " << elapsed << endl; \
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
			info.at(user).slam_poses.push_back(Pose3(M_L_U)); //Load all GT poses into tracking.
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
				t.slam_poses.push_back(beacon_pos); // Only need to push back once
				t.is_beacon = true;
				info.insert(make_pair(user, t));
			}
	}
}

// one_euro_filter<Eigen::Array<double,1,1>, double>
// make_filter()
// {
// }

Tracker::Tracker(
    const string id,
	map<int, Tracker>& others,
    const Pose3 T_body_to_imu,
    const Pose3 T_body_to_decawave,
    const double smoother_lag,
    const bool use_smoother,
    const bool use_filter,
    const bool use_uwb,
	const bool synth_live_slam_mode,
	const string anchor_loc_strategy,
    const SharedNoiseModel& SLAM_noise_model,
    const SharedNoiseModel& UWB_noise_model,
    const SharedNoiseModel& Velocity_noise_model,
    const SharedNoiseModel& Bias_noise_model,
	const SharedNoiseModel& selfloc_UWB_noise_model,
	const SharedNoiseModel& selfloc_Anchor_noise_model,
    std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params,
    const Vector6 prior_imu_bias,
    const Vector3 prior_velocity,
	const string out_dir,
    const string debug_dir
) :
    // Filters
	
	// Smooth, higher APE
    translation_filt(
        200.,
        Eigen::Array<double, 3, 1>::Constant(0.25),
        Eigen::Array<double, 3, 1>::Constant(0.01),
        Eigen::Array<double, 3, 1>::Constant(1),
        Eigen::Array<double, 3, 1>::Zero(),
        Eigen::Array<double, 3, 1>::Ones(),
        [](auto& in) { return in.abs(); }
    ),

	// Jittery, low APE
	// translation_filt(
    //     200.,
    //     Eigen::Array<double, 3, 1>::Constant(0.25),
    //     Eigen::Array<double, 3, 1>::Constant(5),
    //     Eigen::Array<double, 3, 1>::Constant(1),
    //     Eigen::Array<double, 3, 1>::Zero(),
    //     Eigen::Array<double, 3, 1>::Ones(),
    //     [](auto& in) { return in.abs(); }
    // ),

	id(id),

	other_trackers(others),

    // Transforms
    T_body_to_imu(T_body_to_imu),
    T_body_to_decawave(T_body_to_decawave),

    // Priors
    prior_imu_bias(imuBias::ConstantBias(prior_imu_bias)),
    prior_velocity(prior_velocity),

    // IMU
    imu_preintegrated(new PreintegratedCombinedMeasurements(
        imu_preintegration_params,
        imuBias::ConstantBias(prior_imu_bias)
    )),
    delta_t(1.0 / 200.0),

	out_dir(out_dir),
    debug_dir(debug_dir),

    // Noise models
    SLAM_noise_model(SLAM_noise_model),
    UWB_noise_model(UWB_noise_model),
    Velocity_noise_model(Velocity_noise_model),
    Bias_noise_model(Bias_noise_model),
    selfloc_Anchor_noise_model(selfloc_Anchor_noise_model),
    selfloc_UWB_noise_model(selfloc_UWB_noise_model),

    // Graph
    graph(new NonlinearFactorGraph()),

    // Flags / state
    use_filter(use_filter),
    use_uwb(use_uwb),
	synth_live_slam_mode(synth_live_slam_mode),
    imu_available(0),
	slam_status("tracking"),
    use_smoother(use_smoother),
	anchor_loc_strategy(anchor_loc_strategy)
	
{
    if (use_smoother) {
        ISAM2Params isam_params;
        isam_params.factorization = ISAM2Params::QR;
        isam_params.relinearizeThreshold = 0.01;
        isam_params.relinearizeSkip = 1;
        isam_params.enableDetailedResults = true;

        smoother = new IncrementalFixedLagSmoother(smoother_lag, isam_params);
    } else {
        ISAM2Params isam_params;
        isam_params.factorization = ISAM2Params::QR;
        isam_params.relinearizeThreshold = 0.01;
        isam_params.relinearizeSkip = 1;
        isam_params.enableDetailedResults = true;

        ISAM2DoglegParams dogleg;
        isam_params.optimizationParams = dogleg;

        isam = new ISAM2(isam_params);
    }

	string uwb_str = "uwb";
	if (!use_uwb) uwb_str = "no_uwb";

	estimated_trajectory_htm_json_fs = ofstream(out_dir + "/est_"+uwb_str+".json");
	estimated_trajectory_fs = ofstream(out_dir + "/est_"+uwb_str+".txt");
	// slam_trajectory_fs = ofstream(out_dir+"/slam.txt");
	log_fs = ofstream(out_dir+"/log_dump.txt");

}


const SharedNoiseModel& Tracker::UWBNoiseModel() {
	if (slam_status == "tracking" ) {
		// selfloc noise model applies so long as user has tracking.
		if (anchor_loc_strategy == "self-loc") {
			return selfloc_UWB_noise_model;
		}
	}
	return UWB_noise_model;
}

Pose3 Tracker::report_estimate(Pose3 initial, double timestamp){

	Pose3 reported_pose;
	if (use_filter) {
		// Timestamps in seconds, mincutoff beta should be in seconds
		Vector3 filtered_translation = translation_filt(initial.translation() , timestamp);
		Pose3 good_pose(Pose3(initial.rotation(), filtered_translation));
		reported_pose = good_pose;

		log_fs << "Filtering changed pose by " << (initial.translation().norm() - filtered_translation.norm()) << endl;
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

	Pose3 prior_beacon_pose(anchors[id].slam_poses[0]);
		vals.insert(AnchorKey(id), prior_beacon_pose);
		graph->add(NonlinearEquality<Pose3>(AnchorKey(id), prior_beacon_pose));
	

	// // Assuming prior knowledge
	// if (anchor_loc_strategy == "pre-loc") {
	// 	Pose3 prior_beacon_pose(anchors[id].slam_poses[0]);
	// 	vals.insert(AnchorKey(id), prior_beacon_pose);
	// 	graph->add(NonlinearEquality<Pose3>(AnchorKey(id), prior_beacon_pose));
	// }
	// else if (anchor_loc_strategy == "self-loc") {
	// 	// Assuming priors, that post_process.py can add manual errors to beforehand.
	// 	Pose3 prior_beacon_pose(anchors[id].slam_poses[0]);
	// 	vals.insert(AnchorKey(id), prior_beacon_pose);
	// 	graph->add(PriorFactor<Pose3>(AnchorKey(id), prior_beacon_pose, selfloc_Anchor_noise_model));
	// }
	// else if (anchor_loc_strategy == "no-loc") {
	// 	// Do nothing, don't even add the anchors into the graph.
	// }


	// // For selfloc testing on opti_multi1_free_circle

	// // 0 prior
	// // Pose3 prior_beacon_pose;
	// // prior_beacon_pose = Pose3(Rot3::Identity(), Point3(0, 0, 0));
    // // vals.insert(AnchorKey(id), prior_beacon_pose);
	// // graph->add(PriorFactor<Pose3>(AnchorKey(id), prior_beacon_pose, selfloc_Anchor_noise_model));

	// // 1.5m off prior
	// // if (id == "1") {
	// // 	Pose3 prior_beacon_pose;
	// // 	prior_beacon_pose = Pose3(Rot3::Identity(), Point3(-2.5432664840720627,
	// // 							3.1706829696740797,
	// // 							0.12732356031622472));
	// // 	vals.insert(AnchorKey(id), prior_beacon_pose);
	// // 	graph->add(PriorFactor<Pose3>(AnchorKey(id), prior_beacon_pose, selfloc_Anchor_noise_model));
	// // }
	// // else if (id == "5") {
	// // 	Pose3 prior_beacon_pose;
	// // 	prior_beacon_pose = Pose3(Rot3::Identity(), Point3(1.6420235179980649,
	// // 							-2.3033634112002345,
	// // 							0.07176977664742756));
	// // 	vals.insert(AnchorKey(id), prior_beacon_pose);
	// // 	graph->add(PriorFactor<Pose3>(AnchorKey(id), prior_beacon_pose, selfloc_Anchor_noise_model));
	// // }

	// // 1m off prior
	// // if (id == "1") {
	// // 	Pose3 prior_beacon_pose;
	// // 	prior_beacon_pose = Pose3(Rot3::Identity(), Point3(-2.3432664840720627,
	// // 							2.8706829696740797,
	// // 							0.12732356031622472));
	// // 	vals.insert(AnchorKey(id), prior_beacon_pose);
	// // 	graph->add(PriorFactor<Pose3>(AnchorKey(id), prior_beacon_pose, selfloc_Anchor_noise_model));
	// // }
	// // else if (id == "5") {
	// // 	Pose3 prior_beacon_pose;
	// // 	prior_beacon_pose = Pose3(Rot3::Identity(), Point3(1.4420235179980649,
	// // 							-2.0033634112002345,
	// // 							0.07176977664742756));
	// // 	vals.insert(AnchorKey(id), prior_beacon_pose);
	// // 	graph->add(PriorFactor<Pose3>(AnchorKey(id), prior_beacon_pose, selfloc_Anchor_noise_model));
	// // }

	// // 0.5m off prior
	// // if (id == "1") {
	// // 	Pose3 prior_beacon_pose;
	// // 	prior_beacon_pose = Pose3(Rot3::Identity(), Point3(-1.8432664840720627,
	// // 							2.8706829696740797,
	// // 							0.12732356031622472));
	// // 	vals.insert(AnchorKey(id), prior_beacon_pose);
	// // 	graph->add(PriorFactor<Pose3>(AnchorKey(id), prior_beacon_pose, selfloc_Anchor_noise_model));
	// // }
	// // else if (id == "5") {
	// // 	Pose3 prior_beacon_pose;
	// // 	prior_beacon_pose = Pose3(Rot3::Identity(), Point3(0.9420235179980649,
	// // 							-2.0033634112002345,
	// // 							0.07176977664742756));
	// // 	vals.insert(AnchorKey(id), prior_beacon_pose);
	// // 	graph->add(PriorFactor<Pose3>(AnchorKey(id), prior_beacon_pose, selfloc_Anchor_noise_model));
	// // }

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
		if ( mes["type"]=="aligned_slam_pose" && (to_string(mes["src"]) == id) && !set_pose_prior) {
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

			start_timestamp = (double)mes["t"];
			latest_pose_timestamp = (double)mes["t"];

				// Add this key -> timestamp mapping to our map
			key_timestamps[X(track.Ix)] = (double)mes["t"];
			key_timestamps[V(track.Iv)] = (double)mes["t"];
			key_timestamps[B(track.Ib)] = (double)mes["t"];
			for (auto const &[id, tracking_] : anchors) {
				key_timestamps[AnchorKey(id)] = (double)mes["t"];
			}

			if (use_smoother) {
				smoother->update(*graph, vals, key_timestamps); 
			}
			else {
				isam->update(*graph, vals);
			}

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

			log_fs << "Prior bias estimate" << std::endl;
			// prior_imu_bias.print();

			log_fs << "Optimized bias applied to preintegrator" << std::endl;
			// imu_preintegrated->print();


			track.est_poses.push_back(latest_pose); // We'll take the estimate out of values and put it here.
			track.slam_poses.push_back(latest_pose);
			track.est_timestamps.push_back(latest_pose_timestamp);
			track.slam_timestamps.push_back(latest_pose_timestamp);
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
            log_fs << "Keys in vals: ";
            for (const auto& key : vals.keys()) {
                log_fs << DefaultKeyFormatter(key) << " ";
            }
            log_fs << endl;

            log_fs << "Keys in graph: ";
            for (const auto& f : *graph) {
                auto keys = f->keys();
                for (Key k : keys) {
                    log_fs << DefaultKeyFormatter(k) << " ";
                }
            }
            log_fs << endl;
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

		// Save estimate poses for all anchors also
		for (auto& [id, anchor_track]: anchors){
			if (id !="") {
				anchor_track.est_timestamps.push_back(mes_timestamp);
				anchor_track.est_poses.push_back(result.at<Pose3>(AnchorKey(id)));
			}

		}		

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

		write_trajectory_TUM_format( track.est_poses, track.est_timestamps, estimated_trajectory_fs);
		estimated_trajectory_fs.close();

		write_trajectory_HTM_JSON_format (track.est_poses, track.est_timestamps, estimated_trajectory_htm_json_fs, "est_pose", 0, 0 ,0);
		estimated_trajectory_htm_json_fs.close();

		cerr << "Graph dumped to factor_graph.dot" << endl;

		throw; // rethrow
	}
}

void Tracker::exec_smoother(NavState& proposed, double mes_timestamp, 
    string msg, bool print){

	Values result;

	try {
        if (print) {
            log_fs << "Keys in vals: ";
            for (const auto& key : vals.keys()) {
                log_fs << DefaultKeyFormatter(key) << " ";
            }
            log_fs << endl;

            log_fs << "Keys in graph: ";
            for (const auto& f : *graph) {
                auto keys = f->keys();
                for (Key k : keys) {
                    log_fs << DefaultKeyFormatter(k) << " ";
                }
            }
            log_fs << endl;

        }
		
		clock_t isam_t;
		START_TIMER("Start Smoother, "+msg+", ts="+to_string(mes_timestamp), isam_t);
		smoother->update(*graph, vals, key_timestamps); // Crashes on this line specifically
		result = smoother->calculateEstimate();
		END_TIMER("Ended Smoother "+msg, isam_t);

		report_estimate(result.at<Pose3>(X(track.Ix)), mes_timestamp);

		track.est_velocities.push_back(result.at<Vector3>(V(track.Iv)));
		track.changing_bias = result.at<PreintegrationBase::Bias>(B(track.Ib));
		
		prev_state = NavState(result.at<Pose3>(X(track.Ix)), result.at<Vector3>(V(track.Iv)));

		// Save estimate poses for all anchors also
		for (auto& [id, anchor_track]: anchors){
			if (id !="") {
				anchor_track.est_timestamps.push_back(mes_timestamp);
				anchor_track.est_poses.push_back(result.at<Pose3>(AnchorKey(id)));
			}

		}	

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

		write_trajectory_TUM_format( track.est_poses, track.est_timestamps, estimated_trajectory_fs);
		estimated_trajectory_fs.close();

		write_trajectory_TUM_format( track.slam_poses, track.slam_timestamps, slam_trajectory_fs);
		slam_trajectory_fs.close();
		throw; // rethrow
	}
}



void Tracker::processSensor(const json& mes) {

	string slamtype = "aligned_slam_pose";
	if (synth_live_slam_mode) slamtype = "aligned_live_slam_pose"; 
	// With synth SLAM failures, the aligned_live_slam_pose are essentially the same gravity aligned poses as
	// aligned SLAM pose, but with a deformation
	// so to finish making the synthetic failure, we just integrate on top of the aligned_live_slam_pose for some time

	bool start_graph = false;

	if (to_string(mes["src"]) == this->id) {

		start_graph = (double)mes["t"] > start_timestamp;

		if (mes["type"] == "imu" && start_graph) {
			// Add IMU measurement
			Vector3 accel;
			Vector3 gyro;
			get_IMU(mes, accel, gyro);

			imu_preintegrated->integrateMeasurement(accel, gyro, delta_t);
			imu_available++;


			log_fs << "Preintegration at " << mes["t"] << " a: " << accel.x() << " " << accel.y() << " " << accel.z() << ", g: " << gyro.x() << " " << gyro.y() << " " << gyro.z() << endl;

			// Just for plotting at IMU frequency
			PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);
			auto proposed = current_imu_preintegration->predict(prev_state, track.changing_bias);
			if (slam_status == "imu" && synth_live_slam_mode){ 
				// To complete synthetic SLAM failures, insert the integration along with SLAM poses to represent "status": "imu"
				int skip = (int)200.0/30.0; // Subsampling IMU integration poses to 30Hz to match SLAM
				 if (imu_counter % skip == 0) {
					track.slam_poses.push_back(proposed.pose());
					track.slam_timestamps.push_back(mes["t"]);
				 }
				imu_counter++;
			}
			
			report_estimate(proposed.pose(), mes["t"]);

			if (!gt_pose_buffer.empty()) {
				json mes = gt_pose_buffer.front();
				gt_pose_buffer.pop_front();
				processSLAM(mes);
				imu_available = 0;
			} else if (!range_buffer.empty()) { // Must be mutually exclusive
				json mes = range_buffer.front();
				range_buffer.pop_front();
				processUWB(mes);
				imu_available = 0;
			}

		}
		else if (mes["type"] == slamtype && start_graph) {

			// Basically just for plotting and evaluating synthetic SLAM
			if (synth_live_slam_mode) {
				if (mes["status"] == "newmap" || mes["status"] == "tracking") {
					if (imu_available == 0) {
						// Pass this measurement and buffer it until the next IMU becomes available
						log_fs << " Skipped SLAM pose " << endl;
						gt_pose_buffer.push_back(mes);
						return;
					}
					else {
						log_fs << "Used SLAM pose " << endl;
						processSLAM(mes);
						imu_available = 0;
					}
				}
				else if (mes["status"] == "imu" ) {
					slam_status = "imu";
				}
			}
			else {
				// Actual logic implemented here
				if (mes["status"] == "tracking") {

					if (imu_available == 0) {
						// Pass this measurement and buffer it until the next IMU becomes available
						log_fs << " Skipped SLAM pose " << endl;
						gt_pose_buffer.push_back(mes);
						return;
					}
					else {
						log_fs << "Used SLAM pose " << endl;
						processSLAM(mes);
						imu_available = 0;
					}
				}
			}
		}
		else if ( (mes["type"] == "uwb") && use_uwb && start_graph) { 
			if (imu_available == 0) { 
				log_fs << "Skipped User to "<< mes["id"] << endl;
				range_buffer.push_back(mes);
				return; 
			}
			else {
				log_fs << "Used User to "<< mes["id"] << endl;
				processUWB(mes);
				imu_available = 0;
			}
		}
	}
	else { // If we check a measurement thats not our ID, but is 2,3, or 4
		// and is an UWB measurement to one of the anchors
		// We can use that for self-localizing anchors.
		if (mes["type"] == "uwb" && (mes["id"] == 1 || mes["id"] == 5) 
			&& (anchor_loc_strategy == "self-loc")) {
			log_fs << "Buffering " << mes["src"] << " whereas this->id " << this->id << endl;
			// processOtherUWBOnAnchor(mes); //<- so this
			other_range_buffer.push_back(mes);
		}
	}	
}


void Tracker::processSLAM(const json& mes)
{
	log_fs << "Used GT" << endl;
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

	track.slam_poses.push_back(gt_pose);
	track.slam_timestamps.push_back(mes["t"]);

	// Mark down failure intervals
	// if (mes["status"] == "imu" && slam_status == "tracking") start = mes["t"];
	// if (mes["status"] == "newmap" && slam_status == "imu") init_newmap = mes["t"];
	// if (mes["status"] == "tracking" && slam_status == "newmap") end = mes["t"];
	slam_status = mes["status"];

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
	log_fs << "Added IMU factor " << graph->size() - 1 << endl;

	// Add GT prior factor
	graph->add(PriorFactor<Pose3>(X(track.Ix), gt_pose, SLAM_noise_model));
	log_fs << "Added Prior factor " << graph->size() - 1 << endl;

	// Predict current state
	NavState proposed = current_imu_preintegration->predict(prev_state, track.changing_bias);

	// Insert initial values
	vals.insert(X(track.Ix), proposed.pose());
	vals.insert(V(track.Iv), proposed.v());
	vals.insert(B(track.Ib), track.changing_bias);

    // Run iSAM
	if (use_smoother) { exec_smoother(proposed, (double)mes["t"], "GT", true); }
	else { exec_iSAM(proposed, (double)mes["t"], "GT", true); }

}

void Tracker::processOtherUWBOnAnchor(const json& mes, double timestamp)
{
	double measured_range = (double)mes["range"];
	int dst_id = (int)mes["id"];

	// If we aren't using anchors or are assuming pre-localized anchors, skip this function entirely when we see 1 or 5
	if ((dst_id == 1) || (dst_id == 5) && (anchor_loc_strategy != "self-loc")) { return; }
	// Also does not solve the issue.

	log_fs << "User: " << this->id << " Processing range " << mes["src"] << " -> " << mes["id"] << " for t=" << mes["t"] << endl;
	// if (slam_status == "lost") log_fs << "Processing range while lost!" << endl;
	if (slam_status == "imu" || slam_status == "newmap") log_fs << "Processing range while lost!" << endl;

	other_range_counter++;

	if (!other_trackers.contains(dst_id)) { // Ranging to a static anchor

		string dst = to_string(dst_id);

		// Extract GT pose
		Pose3 T_world_to_body;
		string usrname;
		get_pose_from_HTM(mes["embedded_slam_pose"]["T_body_world"],T_world_to_body);
		Pose3 T_body_to_world = T_world_to_body.inverse();

		// Make an instantaneous anchor, to represent the other node's range to an anchor
		Key instantaneous_anchor = symbol('t', other_range_counter);
		Pose3 anchor_pose = T_body_to_world * T_body_to_decawave.inverse(); 
		
		vals.insert(instantaneous_anchor, anchor_pose);
		graph->add(NonlinearEquality<Pose3>(instantaneous_anchor, anchor_pose));
		// key_timestamps[instantaneous_anchor] = (double)mes["t"];
		key_timestamps[instantaneous_anchor] = timestamp;
		
		graph->add(RangeFactorWithTransform<Pose3, Pose3, double>(
			instantaneous_anchor, AnchorKey(dst), measured_range, UWB_noise_model, T_body_to_decawave.inverse()));
		log_fs << " Range to anchor " << mes["id"] << endl;
	}

	log_fs << "User " << this->id << " added other user's Range factor to our anchor state at " << track.Ix << endl;

}

void Tracker::processUWB(const json& mes)
{


	double measured_range = (double)mes["range"];
	int dst_id = (int)mes["id"];
	// If we aren't using anchors, skip this function entirely when we see 1 or 5
	if ((dst_id == 1) || (dst_id == 5) && (anchor_loc_strategy == "no-loc")) { return; }

	log_fs << "Processing range " + id + " -> " << mes["id"] << " for t=" << mes["t"] << endl;
	// if (slam_status == "lost") log_fs << "Processing range while lost!" << endl;
	if (slam_status == "imu" || slam_status == "newmap") log_fs << "Processing range while lost!" << endl;
	track.Ix++;
	track.Iv++;
	track.Ib++;


	// Add this key -> timestamp mapping to our map
	key_timestamps[X(track.Ix)] = (double)mes["t"];
	key_timestamps[V(track.Iv)] = (double)mes["t"];
	key_timestamps[B(track.Ib)] = (double)mes["t"];
	for (auto const &[id, tracking_] : anchors) {
		if (id != "") key_timestamps[AnchorKey(id)] = (double)mes["t"];
	}


	if (other_trackers.contains(dst_id) ) { // Another Flock node
		log_fs << "Range to user " << dst_id << endl;

		Tracker& other = other_trackers.at(dst_id);

		if (other.slam_status == "tracking" && other.track.slam_poses.size() > 0){
			Pose3 other_T_body_to_world = other.track.est_poses.back(); // T_body_to_world
			// Note: This should be taking slam_poses back?

			// Apply UWB antenna transform
			// Create a Point3 State

			Key instantaneous_anchor = symbol('n', track.Ix);
			Pose3 anchor_pose = other_T_body_to_world * other.T_body_to_decawave.inverse(); 
			// anchor pose should be T_decawave_to_world

			vals.insert(instantaneous_anchor, anchor_pose);
			graph->add(NonlinearEquality<Pose3>(instantaneous_anchor, anchor_pose));

			key_timestamps[instantaneous_anchor] = (double)mes["t"];

			graph->add(RangeFactorWithTransform<Pose3, Pose3, double>(
			X(track.Ix), instantaneous_anchor, measured_range, UWB_noise_model, T_body_to_decawave.inverse()));
	
		}
		else {
			log_fs << " Node " << mes["id"] << " lost SLAM tracking, skipping range. " << endl;
			// return;
		}
	}
	else { // Range to a static anchor

		string dst = to_string(dst_id);

		// Needs "Pose of antenna in body frame" i.e. decawave_to_body
		graph->add(RangeFactorWithTransform<Pose3, Pose3, double>(
			X(track.Ix), AnchorKey(dst), measured_range, UWB_noise_model, T_body_to_decawave.inverse()));
		log_fs << " Range to anchor " << mes["id"] << endl;

		// Add all other measurements buffered onto that static anchor.
		// SO iterate through the buffer, when done, clear all ranges that we used to that static anchor.

		vector<json> next_buffer;
		for (json other_mes: other_range_buffer) {
			// If the range we just added goes to 1
			// take all other measurements that go to 1
			if (other_mes["id"] == dst_id) {
				processOtherUWBOnAnchor(other_mes, mes["t"]); // So you must use the timestamp for this measurement
				// Not the measurement's actual timestamp, otherwise the smoother will explode.

				// processOtherUWBOnAnchor(other_mes, other_mes["t"]); 
				// Regardless of the other timestamp, register every range under the current measurement's timestamp
			}
			else {
				next_buffer.push_back(other_mes);
			}
		}
		other_range_buffer = next_buffer;
	}

	log_fs << "Added Range factor to state " << track.Ix << endl;

	// Add IMU factor
	auto* current_imu_preintegration =
		dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);

	CombinedImuFactor imu_factor(
		X(track.Ix - 1), V(track.Iv - 1),
		X(track.Ix), V(track.Iv),
		B(track.Ib - 1), B(track.Ib),
		*current_imu_preintegration);

	graph->add(imu_factor);
	log_fs << "Added IMU factor " << graph->size() - 1 << endl;
	// imu_preintegrated->print();

	// Predict state and insert
	// NavState proposed = current_imu_preintegration->predict(prev_state, track.constant_bias);
	NavState proposed = current_imu_preintegration->predict(prev_state, track.changing_bias);
	vals.insert(X(track.Ix), proposed.pose());
	vals.insert(V(track.Iv), proposed.v());
	vals.insert(B(track.Ib), track.changing_bias);

	// Run optimization
	if (use_smoother) { exec_smoother(proposed, (double)mes["t"], "UWB", true); }
	else { exec_iSAM(proposed, (double)mes["t"], "UWB", true); }
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