#include "gtsam_test.h"
#include "data_tools.h"
#include "utils.h"

#include "cmath"
#include <regex>

using namespace gtsam;
using namespace std;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

#define PLOT_ANCHORS(INFO) {\
	for (const auto& [user_name, user_info] : INFO) {                        \
        if (user_info.is_beacon) {                                          \
			hold(on);														\
            draw_points(user_info.gt_poses, "red");                      \
        }                                                                    \
    }																		\
}

#define PLOT_ESTIMATED_FOR_USERS(INFO, SHOW_LIST) {                          \
    for (const auto& [user_name, user_info] : INFO) {                        \
        if (!user_info.is_beacon) {                                          \
            if (std::find(SHOW_LIST.begin(), SHOW_LIST.end(), user_name) != SHOW_LIST.end()) { \
                hold(on);                                                    \
                draw_trajectory(user_info.est_poses, "blue");                \
                hold(on);                                                  \
                draw_trajectory(user_info.gt_poses, "green");              \
                hold(on);   \
                xlabel("X (m)");                                             \
                ylabel("Y (m)");                                             \
                zlabel("Z (m)");                                             \
				xlim({ -5,5 }); \
				ylim({ -1,9 }); \
				zlim({ 0,5 }); \
            }                                                                \
        }                                                                    \
    }                                                                        \
}



#define PLOT_FOR_USERS(INFO, SHOW_LIST) {						           \
    for (const auto& [user_name, user_info] : INFO) {                      \
        if (!user_info.is_beacon) {                                        \
            if (find(SHOW_LIST.begin(), SHOW_LIST.end(), user_name) != SHOW_LIST.end()) { \
                auto fig = figure();                                       \
                fig->name(user_name + " trajectory");                      \
                title(user_name);                                          \
                                                                           \
                hold(on);                                                  \
                draw_trajectory(user_info.vio_poses, "red");               \
                hold(on);                                                  \
                draw_trajectory(user_info.gt_poses, "green");              \
                hold(on);                                                  \
				draw_points(user_info.gt_poses, "green"); \
				hold(on); \
                draw_trajectory(user_info.est_poses, "blue");              \
                hold(on);                                                           \
                xlabel("X (m)");                                           \
                ylabel("Y (m)");                                           \
                zlabel("Z (m)");                                           \
                                                                           \
            }                                                              \
        }                                                                  \
    }                                                                      \
}

#define PLOT_W_OPTIMIZER_PARAMS_FOR_USERS(INFO, SHOW_LIST, LAMBDA, LAMBDA_FACTOR) {			   \
	 for (const auto& [user_name, user_info] : INFO) {                      \
			if (!user_info.is_beacon) {                                        \
				if (find(SHOW_LIST.begin(), SHOW_LIST.end(), user_name) != SHOW_LIST.end()) { \
					auto fig = figure();                                       \
					fig->name("Trajectory");                      \
					title(user_name+" L="+to_string(LAMBDA)+" LF="+to_string(LAMBDA_FACTOR));                                          \
																			   \
					hold(on);                                                  \
					draw_trajectory(user_info.vio_poses, "red");               \
					hold(on);													\
					draw_trajectory(user_info.gt_poses, "green");                      \
					hold(on);                                                  \
					draw_trajectory(user_info.est_poses, "blue");              \
																			   \
					xlabel("X (m)");                                           \
					ylabel("Y (m)");                                           \
					zlabel("Z (m)");                                           \
																			   \
				} \
			} \
	 } \
}

int main(int argc, char* argv[]) {


	string directory = "/home/admitriev/Datasets/UWBSLAM_pilot/";
	string trial_name = "pilot3_slow_low_post";
	string out_directory = "/home/admitriev/Research/pilot_results/" + trial_name;

	ifstream raw_fs(directory + trial_name + "/" + "all.json");
	ifstream beacon_fs(directory + trial_name + "/anchors.json");

	// Redirect stdout output to a text file.
	std::ofstream out("/home/admitriev/Research/gtsam_test/pilot_factor_graphs/print_dump.txt");
	std::cout.rdbuf(out.rdbuf()); // redirect cout to file

	json sensor_stream = json::parse(raw_fs);
	map<string, tracking> info; // Map of username to tracking information

	//get_gt_info(info, json::parse(gt_fs)); // fill user_info with gt_pose trajectory

	get_beacon_info(info, json::parse(beacon_fs));
	info.insert(pair<string, tracking>("2", tracking()));

	double dt = 1.0 / 200.0; // IMU gyro and accelerometer operate at 200Hz


	// --- Noise Models ---

	// VIO noise model

	double vio_ori_stdev = 0.175;
	double vio_pos_stdev = 0.2;
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));

	// UWB noise model

	double uwb_stdev = 0.001;
	//double uwb_stdev = 0.5;
	// They set this to 100 or 1000 in this example: https://github.com/borglab/gtsam/blob/develop/examples/RangeISAMExample_plaza2.cpp
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	// GT noise model - (use to define pose prior)
	double gt_pos_stdev = 1e-3;
	double gt_ori_stdev = 1e-3;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));
	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.01);
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 0.01);


	//// IMU noise model


	double SCALE = 1;

	double GYRO_NOISE_DENSITY = 0.0002049600985797649; 
	double ACCEL_NOISE_DENSITY = 0.002064189891192468;

	Matrix33 continuous_time_accel_noise_cov = I_3x3 * pow(ACCEL_NOISE_DENSITY, 2) * SCALE;
	Matrix33 continuous_time_gyro_noise_cov = I_3x3 * pow(GYRO_NOISE_DENSITY, 2) * SCALE;


	double GYRO_BIAS_RW = 3.1998555455947417e-06;
	double ACCEL_BIAS_RW = 0.00022919238444020807;

	Matrix33 continuous_time_accel_bias_rw = I_3x3 * pow(ACCEL_BIAS_RW, 2) * SCALE;
	Matrix33 continuous_time_gyro_bias_rw = I_3x3 * pow(GYRO_BIAS_RW, 2) * SCALE;

	Matrix66 initial_bias_cov = I_6x6 * 1e-5 * SCALE;

	Matrix33 integration_cov = I_3x3 * 1e-5 * SCALE;


	// LOOK AT ALL OF THE IMU CALIBRATION YOU'VE DONE.
	// MAKE SURE THE UNITS ARE CORRECT. REMEMBER YOU HAVE NOTES ON THE IMU PARAMETERS
	// SEE this: https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
	// AND this: kalibr_mount.zip/kalibr_mount/allan_variance_out

	boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = PreintegratedCombinedMeasurements::Params::MakeSharedD();
	imu_preintegration_params->accelerometerCovariance = continuous_time_accel_noise_cov;
	imu_preintegration_params->gyroscopeCovariance = continuous_time_gyro_noise_cov;

	imu_preintegration_params->biasAccCovariance = continuous_time_accel_bias_rw;
	imu_preintegration_params->biasOmegaCovariance = continuous_time_gyro_bias_rw;

	imu_preintegration_params->integrationCovariance = integration_cov;
	imu_preintegration_params->biasAccOmegaInt = initial_bias_cov;

	imu_preintegration_params->use2ndOrderCoriolis = false;


	imuBias::ConstantBias prior_imu_bias;

	Matrix33 transform;
	transform << 1, 0, 0,
		0, 0, 1,
		0, -1, 0;
	Pose3 sensor_to_body_transform(Rot3(transform), Vector3(0, 0, 0));
	// body_P_sensor : "pose of sensor frame w.r.t body frame"
	imu_preintegration_params->body_P_sensor = sensor_to_body_transform;

	NonlinearFactorGraph* graph = new NonlinearFactorGraph();

	// Beacon info is a string, 
	// Make Key
	const function<Key(string, int)> MK_Anchor = [](string name, int I) {
		return symbol('s', stoi(name));
	};

	// Establish and attach priors to keys

	Values vals;

	Pose3 slam_to_world(Rot3(transform), Vector3(0, 0, 1.82));

	//PLOT_ANCHORS(info);
	//////draw_trajectory(info["2"].gt_poses, "green");
	//PLOT_ESTIMATED_FOR_USERS(info, show_list);
	//show();

	// Initialize the first GT pose.

	int pose_num = 0;
	for (auto& [u, track] : info) {
		track.Ix = 0;
		track.Iv = 0;
		track.Ib = 0;

		if (track.is_beacon) { // Set nonlinearequality on anchors
			track.pose_key = MK_Anchor(u, 0);
			Pose3 prior_beacon_pose(track.gt_poses[0]); // Position of beacon in U frame extracted from GT
			vals.insert(track.pose_key, prior_beacon_pose);
			graph->add(NonlinearEquality<Pose3>(track.pose_key, prior_beacon_pose));
			//graph->add(PriorFactor<Pose3>(track.pose_key, prior_beacon_pose, GT_noise_model));
		}
		else { // Since we only have one user, user 2.
			
			Pose3 find_first_gt_pose;
			for (json mes : sensor_stream) {
				if (mes["type"] == "gt_pose") {
					Pose3 gt_slam;
					get_GT(mes, gt_slam);
					find_first_gt_pose = slam_to_world * gt_slam;
					break;
				}
			}

			Pose3 start_pose = find_first_gt_pose;
			Vector3 prior_velocity(0, 0, 0);

			vals.insert(X(track.Ix), start_pose);
			vals.insert(V(track.Iv), prior_velocity);
			vals.insert(B(track.Ib), prior_imu_bias);

			graph->addPrior(X(track.Ix), start_pose, GT_noise_model);
			graph->addPrior(V(track.Iv), prior_velocity, prior_velocity_noise_model);
			graph->addPrior(B(track.Ib), prior_imu_bias, prior_bias_noise_model);

			track.est_poses.push_back(start_pose); // We'll take the estimate out of values and put it here.
			track.est_velocitys.push_back(prior_velocity);
			//track.constant_bias = prior_imu_bias;

		}

	}

	vector<string> show_list = { "2" };
	vector<string> anchors = { "1", "3", "5" };

	// Use Preintegrator params, and bias prior, to create a new preintegrator object that we can use for an IMU factor.
	PreintegrationType* imu_preintegrated = new PreintegratedCombinedMeasurements(imu_preintegration_params, prior_imu_bias);

	ISAM2Params isam_params;
	isam_params.factorization = ISAM2Params::QR;
	isam_params.relinearizeThreshold = 0.01;
	isam_params.relinearizeSkip = 1; // More informed optimization at the cost of more computing.
	ISAM2DoglegParams dogleg;
	isam_params.optimizationParams = dogleg;
	ISAM2* isam = new ISAM2(isam_params);

	tracking& user = info.at("2");
	NavState prev_state(user.est_poses.back(), user.est_velocitys.back());

	int T_UWB = 10; // Every 30 IMU measurements, generate 1 synthetic UWB measurement.
	int uwb_counter = 0;
	int GT_CORRECTION_COUNT = 0;

	int imu_counter = 0;
	int last_imu_counter = 0;
	bool initialization_complete = true;
	bool start_graph = false;

	bool use_gt = true;
	int gt_correction_hz_max = 20;
	double gt_correction_hz = 20;
	int skip = (int) (gt_correction_hz_max / gt_correction_hz);

	bool use_uwb = true;

	int imu_count_at_last_correction = 0;
	int imu_count_at_last_imu_factor = 0;

	int factor_counter = 0;

	vector<json> gt_pose_buffer;
	vector<json> range_buffer;


	for (json mes : sensor_stream) {

		if (mes["type"] == "imu") {

			// Add IMU measurement
			start_graph = true;
			Vector3 accel;
			Vector3 gyro;
			get_IMU(mes, accel, gyro);
			imu_preintegrated->integrateMeasurement(accel, gyro, dt);
			imu_counter++;

			cout << "Preintegration on a: " << accel.x() << " " << accel.y() << " " << accel.z() << ", g: " << gyro.x() << " " << gyro.y() << " " << gyro.z() << endl;
			PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);
			auto proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);
			user.est_poses.push_back(proposed.pose());

			for (json mes : gt_pose_buffer) {
				if (GT_CORRECTION_COUNT % skip == 0) {

					cout << "Used GT" << endl;
					user.Ix++;
					user.Iv++;
					user.Ib++;

					Pose3 gt_pose_slam;
					get_GT(mes, gt_pose_slam);

					Pose3 gt_pose = slam_to_world * gt_pose_slam;

					user.gt_poses.push_back(gt_pose);


					//vector<Pose3> p = { gt_pose };
					//hold(on);
					//draw_points(p, "green");

					PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);
					CombinedImuFactor imu_factor(X(user.Ix - 1), V(user.Iv - 1), X(user.Ix), V(user.Iv), B(user.Ib - 1), B(user.Ib), *current_imu_preintegration);
					graph->add(imu_factor);
					cout << "Added IMU factor " << graph->size() - 1 << endl;

					// GT correction (currently as GPS factor)
					graph->add(PriorFactor<Pose3>(X(user.Ix), gt_pose, GT_noise_model));
					cout << "Added Prior factor " << graph->size() - 1 << endl;

					auto proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);

					vals.insert(X(user.Ix), proposed.pose());
					vals.insert(V(user.Iv), proposed.v());
					vals.insert(B(user.Ib), user.constant_bias);
					Values result;

					try {
						isam->update(*graph, vals);
						result = isam->calculateEstimate();
						user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
						user.est_velocitys.push_back(result.at<Vector3>(V(user.Iv))); // Assuming V and X are on same index

						cout << "Successful estimate on GT factor" << endl;
					}
					catch (const std::exception& e) {
						std::cerr << "Optimizer update failed: " << e.what() << std::endl;

						// Dump factor graph to .dot file
						std::ofstream os("/home/admitriev/Research/gtsam_test/pilot_factor_graphs/factor_graph.dot");
						graph->saveGraph(os, result); // Uses current result (could also pass an empty Values())
						os.close();

						graph->print("");

						//PLOT_ANCHORS(info);
						//PLOT_ESTIMATED_FOR_USERS(info, show_list);

						std::cerr << "Graph dumped to factor_graph.dot" << std::endl;
						throw; // rethrow after dumping
					}

					prev_state = NavState(result.at<Pose3>(X(user.Ix)), result.at<Vector3>(V(user.Iv)));
					// Here, you need to re-insert the optimization results as the base of the next preintegration.

				//graph->resize(0);
					vals.clear();

					imu_preintegrated->resetIntegrationAndSetBias(user.constant_bias); // Clear preintegrator
				}

				imu_count_at_last_correction = imu_counter;
				imu_count_at_last_imu_factor = imu_counter;

				GT_CORRECTION_COUNT++;
			}
			gt_pose_buffer.clear();

			for (json mes : range_buffer) {
				double range;
				string src_user = "2";
				string dst_user;

				uwb_counter++;

				get_UWB(mes, src_user, dst_user, range);

				user.Ix++;
				user.Iv++;
				user.Ib++;

				//// This is assuming we have GT poses at IMU frequency. We have GT poses at 20Hz.
				double true_range = distance3(info[src_user].gt_poses.back().translation(), info[dst_user].gt_poses.back().translation());



				////graph->add(RangeFactor<Pose3, Pose3, double>(X(info[src_user].Ix), MK_Anchor(dst_user, info[dst_user].Ix), range, UWB_noise_model));
				graph->add(RangeFactor<Pose3, Pose3, double>(X(info[src_user].Ix), info[dst_user].pose_key, true_range, UWB_noise_model));
				cout << "Added Range factor " << graph->size() - 1 << endl;

				// Placing a faked observation on this state, following the assumption that we're always moving forward where our head is going.
				// I would think this fully constrains it but apparently not???
				Pose3 last_correction = user.gt_poses.back();
				Vector3 last_velocty = user.est_velocitys.back();
				Vector3 reckoned_translation(last_velocty.x() * (imu_counter - imu_count_at_last_correction) * dt, 0, 0); // V * dT since last correction
				Pose3 reckoned_pose = last_correction * Pose3(Rot3::Identity(), reckoned_translation); // Where would be be X seconds from now, if we followed a straight line trajectory from the last GT orientation?
				graph->add(PriorFactor<Pose3>(X(user.Ix), reckoned_pose, GT_noise_model));
				cout << "Added (fake) Prior factor " << graph->size() - 1 << endl;


				PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);
				CombinedImuFactor imu_factor(X(user.Ix - 1), V(user.Iv - 1), X(user.Ix), V(user.Iv), B(user.Ib - 1), B(user.Ib), *current_imu_preintegration);
				graph->add(imu_factor);
				cout << "Added IMU factor " << graph->size() - 1 << endl;

				auto proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);

				vals.insert(X(user.Ix), proposed.pose());
				vals.insert(V(user.Iv), proposed.v());
				vals.insert(B(user.Ib), user.constant_bias);

				Values result;
				try {
					isam->update(*graph, vals);
					result = isam->calculateEstimate();
					user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
					user.est_velocitys.push_back(result.at<Vector3>(V(user.Iv))); // Assuming V and X are on same index

					cout << "Successful estimate on UWB factor" << endl;
				}
				catch (const std::exception& e) {
					std::cerr << "Optimizer update failed on UWB count: " << e.what() << std::endl;

					// Dump factor graph to .dot file
					std::ofstream os("/home/admitriev/Research/gtsam_test/pilot_factor_graphs/factor_graph.dot");
					graph->saveGraph(os, result); // Uses current result (could also pass an empty Values())
					os.close();

					//PLOT_ANCHORS(info);
					//PLOT_ESTIMATED_FOR_USERS(info, show_list);


					std::cerr << "Graph dumped to factor_graph.dot" << std::endl;
					throw; // rethrow after dumping
				}

				prev_state = NavState(result.at<Pose3>(X(user.Ix)), result.at<Vector3>(V(user.Iv)));
				// Here, you need to re-insert the optimization results as the base of the next preintegration.
				//graph->resize(0);
				vals.clear();
				// iSam internally caches past graph and vals states it was called on, 
				// so if you don't resize, you'll get duplicate keys

				imu_preintegrated->resetIntegrationAndSetBias(user.constant_bias); // Clear preintegrator
			}
			range_buffer.clear();
		
		}
		else if (use_gt && mes["type"] == "gt_pose" && start_graph) {

			if (GT_CORRECTION_COUNT % skip == 0) {


				if (imu_counter == imu_count_at_last_imu_factor) {
					// Pass this measurement and buffer it until the next IMU becomes available
					gt_pose_buffer.push_back(mes);
					continue;
				}

				cout << "Used GT non-buffered" << endl;
				user.Ix++;
				user.Iv++;
				user.Ib++;

				Pose3 gt_pose_slam;
				get_GT(mes, gt_pose_slam);

				Pose3 gt_pose = slam_to_world * gt_pose_slam;

				user.gt_poses.push_back(gt_pose);


				//vector<Pose3> p = { gt_pose };
				//hold(on);
				//draw_points(p, "green");

				PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);
				CombinedImuFactor imu_factor(X(user.Ix - 1), V(user.Iv - 1), X(user.Ix), V(user.Iv), B(user.Ib - 1), B(user.Ib), *current_imu_preintegration);
				graph->add(imu_factor);
				cout << "Added IMU factor " << graph->size() - 1 << endl;

				// GT correction (currently as GPS factor)
				graph->add(PriorFactor<Pose3>(X(user.Ix), gt_pose, GT_noise_model));
				cout << "Added Prior factor " << graph->size() - 1 << endl;

				auto proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);

				vals.insert(X(user.Ix), proposed.pose());
				vals.insert(V(user.Iv), proposed.v());
				vals.insert(B(user.Ib), user.constant_bias);
				Values result;

				try {
					isam->update(*graph, vals);
					result = isam->calculateEstimate();
					user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
					user.est_velocitys.push_back(result.at<Vector3>(V(user.Iv))); // Assuming V and X are on same index

					cout << "Successful estimate on GT factor" << endl;
				}
				catch (const std::exception& e) {
					std::cerr << "Optimizer update failed: " << e.what() << std::endl;

					// Dump factor graph to .dot file
					std::ofstream os("/home/admitriev/Research/gtsam_test/pilot_factor_graphs/factor_graph.dot");
					graph->saveGraph(os, result); // Uses current result (could also pass an empty Values())
					os.close();

					graph->print("");

					//PLOT_ANCHORS(info);
					//PLOT_ESTIMATED_FOR_USERS(info, show_list);

					std::cerr << "Graph dumped to factor_graph.dot" << std::endl;
					throw; // rethrow after dumping
				}

				prev_state = NavState(result.at<Pose3>(X(user.Ix)), result.at<Vector3>(V(user.Iv)));
				// Here, you need to re-insert the optimization results as the base of the next preintegration.

			graph->resize(0);
				vals.clear();

				imu_preintegrated->resetIntegrationAndSetBias(user.constant_bias); // Clear preintegrator
			}

			imu_count_at_last_correction = imu_counter;
			imu_count_at_last_imu_factor = imu_counter;

			GT_CORRECTION_COUNT++;
			
		}
		else if (use_uwb && mes["type"] == "uwb" && start_graph) {


			if (imu_counter == imu_count_at_last_imu_factor) {
				// Pass this measurement and buffer it until the next IMU becomes available
				range_buffer.push_back(mes);
				continue;
			}

			double range;
			string src_user = "2";
			string dst_user;

			uwb_counter++;

			get_UWB(mes, src_user, dst_user, range);

			user.Ix++;
			user.Iv++;
			user.Ib++;

			//// This is assuming we have GT poses at IMU frequency. We have GT poses at 20Hz.
			double true_range = distance3(info[src_user].gt_poses.back().translation(), info[dst_user].gt_poses.back().translation());



			////graph->add(RangeFactor<Pose3, Pose3, double>(X(info[src_user].Ix), MK_Anchor(dst_user, info[dst_user].Ix), range, UWB_noise_model));
			graph->add(RangeFactor<Pose3, Pose3, double>(X(info[src_user].Ix), info[dst_user].pose_key, true_range, UWB_noise_model));
			cout << "Added Range factor " << graph->size()-1 << endl;

			// Placing a faked observation on this state, following the assumption that we're always moving forward where our head is going.
			// I would think this fully constrains it but apparently not???
			Pose3 last_correction = user.gt_poses.back();
			Vector3 last_velocty = user.est_velocitys.back();
			Vector3 reckoned_translation( last_velocty.x() * (imu_counter - imu_count_at_last_correction) * dt, 0, 0); // V * dT since last correction
			Pose3 reckoned_pose = last_correction * Pose3(Rot3::Identity(), reckoned_translation); // Where would be be X seconds from now, if we followed a straight line trajectory from the last GT orientation?
			graph->add(PriorFactor<Pose3>(X(user.Ix), reckoned_pose, GT_noise_model));
			cout << "Added (fake) Prior factor " << graph->size() - 1 << endl;


			PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);
			CombinedImuFactor imu_factor(X(user.Ix - 1), V(user.Iv - 1), X(user.Ix), V(user.Iv), B(user.Ib - 1), B(user.Ib), *current_imu_preintegration);
			graph->add(imu_factor);
			cout << "Added IMU factor " << graph->size() - 1 << endl;

			auto proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);

			vals.insert(X(user.Ix), proposed.pose());
			vals.insert(V(user.Iv), proposed.v());
			vals.insert(B(user.Ib), user.constant_bias);

			Values result;
			try {
				isam->update(*graph, vals);
				result = isam->calculateEstimate();
				user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
				user.est_velocitys.push_back(result.at<Vector3>(V(user.Iv))); // Assuming V and X are on same index

				cout << "Successful estimate on UWB factor" << endl;
			}
			catch (const std::exception& e) {
				std::cerr << "Optimizer update failed on UWB count: " << e.what() << std::endl;

				// Dump factor graph to .dot file
				std::ofstream os("/home/admitriev/Research/gtsam_test/pilot_factor_graphs/factor_graph.dot");
				graph->saveGraph(os, result); // Uses current result (could also pass an empty Values())
				os.close();

				//PLOT_ANCHORS(info);
				//PLOT_ESTIMATED_FOR_USERS(info, show_list);


				std::cerr << "Graph dumped to factor_graph.dot" << std::endl;
				throw; // rethrow after dumping
			}

			prev_state = NavState(result.at<Pose3>(X(user.Ix)), result.at<Vector3>(V(user.Iv)));
			// Here, you need to re-insert the optimization results as the base of the next preintegration.
			graph->resize(0);
			vals.clear();
			// iSam internally caches past graph and vals states it was called on, 
			// so if you don't resize, you'll get duplicate keys

			imu_count_at_last_imu_factor = imu_counter;

			imu_preintegrated->resetIntegrationAndSetBias(user.constant_bias); // Clear preintegrator
		}
		else {

		}
	}


	PLOT_ANCHORS(info);
	PLOT_ESTIMATED_FOR_USERS(info, show_list);

	show();

	return 0;
}
