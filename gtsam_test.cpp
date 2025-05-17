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
                draw_trajectory_with_error(user_info.est_poses, user_info.est_poses_error, "blue");                \
                hold(on);                                                  \
                draw_trajectory(user_info.gt_poses, "green");              \
                hold(on);   \
                xlabel("X (m)");                                             \
                ylabel("Y (m)");                                             \
                zlabel("Z (m)");                                             \
				xlim({ -3.5,3.5 }); \
				ylim({ -1,6 }); \
				/*zlim({ 0,7 }); */ \
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
                draw_trajectory(user_info.est_poses, "blue");              \
                                                                           \
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

// Pass this the graph constructed from our dataset
void LM_lambda_search(NonlinearFactorGraph* graph, Values vals, map<string, tracking> info) {
	const function<Key(string, int)> MK = [](string username, int I) {
		Key k;
		if (username.find("static") != std::string::npos) {
			regex numberRegex(R"(\d+$)");
			smatch match;
			regex_search(username, match, numberRegex);
			k = symbol('s', stoi(match.str())); // e.x. s11 if 'static11'
		}
		else {
			k = symbol(username[0], I);
			if (username == "jeff") k = symbol('f', I);
		}
		return k;
	};

	// Run 1
	vector<double> attempt_lambdaInitial = { 20, 15, 10 };
	//vector<double> attempt_lambdaInitial = { 10, 1, 0.1, 0.001, 0.0001, 0.00001 };
	vector<double> attempt_lambdaFactor = { 100000, 10000, 1000, 100, 10, 7, 5, 3 }; // Won't run with 1

	vector<string> show_list = { "nuno" };

	for (double lambdaInitial : attempt_lambdaInitial) {

		for (double lambdaFactor : attempt_lambdaFactor) {

			LevenbergMarquardtParams lm_params;
			lm_params.diagonalDamping = true;
			lm_params.setlambdaInitial(lambdaInitial);
			lm_params.lambdaFactor = lambdaFactor;
			lm_params.linearSolverType = NonlinearOptimizerParams::LinearSolverType::MULTIFRONTAL_QR;
			LevenbergMarquardtOptimizer lm_optimizer(*graph, vals, lm_params);


			unpack_results(lm_optimizer.optimize(), MK, info);
			PLOT_W_OPTIMIZER_PARAMS_FOR_USERS(info, show_list, lambdaInitial, lambdaFactor);
			clear_results(info); // clear Est_poses trajectory
		}
	}

	show();
}



int run_cappella() {

	string directory = "/home/admitriev/Datasets/UWBSLAM_pilot/";
	string trial_name = "pilot0";
	string out_directory = "/home/admitriev/Research/pilot_results/" + trial_name;

	ifstream raw_fs(directory + trial_name + "/" + "all.json");
	ifstream beacon_fs(directory + "pilot_anchors.json");

	json sensor_stream = json::parse(raw_fs);
	map<string, tracking> info; // Map of username to tracking information

	//get_gt_info(info, json::parse(gt_fs)); // fill user_info with gt_pose trajectory

	get_beacon_info(info, json::parse(beacon_fs)); // I think this is reading beacon positions in properly

	info.insert(pair<string, tracking>("2", tracking()));

	double dt = 1.0 / 200.0; // IMU gyro and accelerometer operate at 200Hz


	// --- Noise Models ---

	// VIO noise model

	double vio_ori_stdev = 0.175; 
	double vio_pos_stdev = 0.2;
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));

	// UWB noise model

	double uwb_stdev = 0.1;
	//double uwb_stdev = 1;
	// They set this to 100 or 1000 in this example: https://github.com/borglab/gtsam/blob/develop/examples/RangeISAMExample_plaza2.cpp
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	// GT noise model - (use to define pose prior)
	double gt_pos_stdev = 0.01;
	double gt_ori_stdev = 0.01;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));
	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.01);
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 0.01);


	// IMU noise model

	imuBias::ConstantBias prior_imu_bias; // Assumption of no prior IMU bias

	// Realsense Gyro is in radians / sec: https://support.intelrealsense.com/hc/en-us/community/posts/9489403831059-d435i-gyro-data-unit 

	// Hard coded from IMU comparison sheet
	double GYRO_NOISE = 0.014 * M_PI/180; // deg / s / sqrt(Hz) -> Since realsense gyro returns data in rad, GYRO_NOISE should also be given in rad
	double ACCEL_NOISE = 0.0014715; // m / s^2 / sqrt(Hz)
	Matrix33 accel_noise_cov = I_3x3 * pow(ACCEL_NOISE, 2);
	Matrix33 gyro_noise_cov = I_3x3 * pow(GYRO_NOISE, 2);
	Matrix33 noise_integration_cov = I_3x3 * 1e-8;  // error committed in integrating position from velocities

	// Hard coded from calibration.json
	Vector3 GYRO_BIAS(-0.00307518, 0.0003668, 0.00393268); // I sure hope these are in the same units as what GTSAM expects (Realsense doesn't label calibration output with units)
	Vector3 ACCEL_BIAS(-0.031682, -0.0617278, 0.02699346);
	Matrix33 accel_bias_cov;

	accel_bias_cov << pow(ACCEL_BIAS(0), 2), 0, 0,
					0, pow(ACCEL_BIAS(1), 2), 0,
					0, 0, pow(ACCEL_BIAS(2), 2);
								
	Matrix33 gyro_bias_cov;
	gyro_bias_cov << pow(GYRO_BIAS(0), 2), 0, 0,
				0, pow(GYRO_BIAS(1), 2), 0,
				0, 0, pow(GYRO_BIAS(2), 2);

	Matrix66 initial_bias_cov = I_6x6 * 1e-5; // 


	// Use our noise model to define the parameters of an IMU preintegrator
	// With these params 'MakeSharedU' gravity points along negative z axis, which is how I defined my global coordinate (aka navigation frame)

	boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = PreintegratedCombinedMeasurements::Params::MakeSharedU();
	imu_preintegration_params->accelerometerCovariance = accel_noise_cov;
	imu_preintegration_params->integrationCovariance = noise_integration_cov;
	imu_preintegration_params->gyroscopeCovariance = gyro_noise_cov;
	imu_preintegration_params->biasAccCovariance = accel_bias_cov;
	imu_preintegration_params->biasOmegaCovariance = gyro_bias_cov;
	imu_preintegration_params->biasAccOmegaInt = initial_bias_cov;

	Matrix33 negate_x_axis;
	negate_x_axis << -1, 0, 0,
					0, 1, 0,
					0, 0, 1;
	// 90 about x-axis, then negate x-axis
	Pose3 sensor_to_body_transform( (Rot3(negate_x_axis) * Rot3::AxisAngle(Point3(1, 0, 0), +M_PI / 2)).inverse(), Vector3(0, 0, 0));
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
	Pose3 gt_pose;

	int pose_num = 0;
	for (auto& [u, track] : info) {
		track.Ix = 0;
		track.Iv = 0;
		track.Ib = 0;

		if (track.is_beacon) { // Set nonlinearequality on anchors
			track.pose_key = MK_Anchor(u,0);
			Pose3 prior_beacon_pose(track.gt_poses[0]); // Position of beacon in U frame extracted from GT
			vals.insert(track.pose_key, prior_beacon_pose);
			graph->add(NonlinearEquality<Pose3>(track.pose_key, prior_beacon_pose));
			//graph->add(PriorFactor<Pose3>(track.pose_key, prior_beacon_pose, GT_noise_model));
		}
		else { // Since we only have one user, user 2.

			Point3 prior_position(0, 0, 1.3); // I was carrying laptop at about chest level ~130cm off the ground
			Pose3 start_pose(Rot3::Identity(), prior_position);
			gt_pose = start_pose;

			Vector3 prior_velocity(0, 0, 0);

			vals.insert(X(track.Ix), start_pose);
			vals.insert(V(track.Iv), prior_velocity);
			vals.insert(B(track.Ib), prior_imu_bias);

			graph->addPrior(X(track.Ix), start_pose, GT_noise_model);
			graph->addPrior(V(track.Iv), prior_velocity, prior_velocity_noise_model);
			graph->addPrior(B(track.Ib), prior_imu_bias, prior_bias_noise_model);

			track.est_poses.push_back(start_pose); // We'll take the estimate out of values and put it here.
			track.est_velocitys.push_back(prior_velocity);
			track.constant_bias = prior_imu_bias;

		}

	}

	vector<string> show_list = { "2" };


	// Use Preintegrator params, and bias prior, to create a new preintegrator object that we can use for an IMU factor.
	//PreintegrationType* imu_preintegrated = new PreintegratedCombinedMeasurements(imu_preintegration_params, prior_imu_bias);
	std::shared_ptr<PreintegrationType> imu_preintegrated = std::make_shared<PreintegratedImuMeasurements>(imu_preintegration_params, prior_imu_bias);


	ISAM2Params isam_params;
	isam_params.factorization = ISAM2Params::QR;
	//isam_params.relinearizeThreshold = 0.01;
	//isam_params.relinearizeSkip = 1;
	ISAM2DoglegParams dogleg;
	isam_params.optimizationParams = dogleg;
	ISAM2* isam = new ISAM2(isam_params);

	tracking& user = info.at("2");
	NavState prev_state(user.est_poses.back(), user.est_velocitys.back());
	imuBias::ConstantBias prev_bias = prior_imu_bias;


	double gt_velocity = 12.0 / 30.0;
	double middle_timestamp = 225271404.76314998;
	double dy = gt_velocity * dt;
	int T_CORRECTION = 200; // Every ~1 second. 200 IMU measurements, correct with GT.
	int T_UWB = 10; // Every 30 IMU measurements, generate 1 synthetic UWB measurement.
	int uwb_counter = 0;
	int GT_CORRECTION_COUNT = 0;

	int imu_counter = 0;
	int last_imu_counter = 0;
	bool initialization_complete = true;
	bool start_graph = false; 
	bool turn = false;
	// Setting a constraint that graph can only start on the first imu measurement
	// long string of uwb measurements leads to integration on nothing ~40 times.

	for (json mes : sensor_stream) {

		//if (GT_CORRECTION_COUNT > 2) break;

		if (mes["type"] == "imu") {

			// Add IMU measurement
			start_graph = true;
			Vector3 accel;
			Vector3 gyro;
			get_IMU(mes, accel, gyro);
			imu_preintegrated->integrateMeasurement(accel, gyro, dt);
			imu_counter++;

			if (float(mes["t"]) >= middle_timestamp && !turn) {
				gt_pose = gt_pose * Pose3(Rot3::AxisAngle(Point3(0, 0, 1), -M_PI), Vector3(0, 0, 0));
				turn = true;
			}

			Pose3 delta_pose(Rot3::Identity(), Vector3(0, dy, 0));
			gt_pose = gt_pose * delta_pose;
			user.gt_poses.push_back(gt_pose);

			// Periodically generate a GT correction
			if (imu_counter % T_CORRECTION == 0) {

				user.Ix++;
				user.Iv++;
				user.Ib++;

				//PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);
				//CombinedImuFactor imu_factor(X(user.Ix - 1), V(user.Iv - 1), X(user.Ix), V(user.Iv), B(user.Ib - 1), B(user.Ib), *current_imu_preintegration);
				//graph->add(imu_factor);

				auto preint_imu =
					dynamic_cast<const PreintegratedImuMeasurements&>(*imu_preintegrated);
				ImuFactor imu_factor(X(user.Ix - 1), V(user.Iv - 1),
					X(user.Ix), V(user.Iv),
					B(user.Ib - 1), preint_imu);
				graph->add(imu_factor);
				imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
				graph->add(BetweenFactor<imuBias::ConstantBias>(
					B(user.Ib - 1), B(user.Ib), zero_bias,
					prior_bias_noise_model));

				auto proposed = preint_imu.predict(prev_state, prev_bias);
				auto cov_matrix = preint_imu.preintMeasCov(); // COVARIANCE OF: [PreintROTATION PreintPOSITION PreintVELOCITY BiasAcc BiasOmega]
				Vector3 position_var(cov_matrix(3, 3), cov_matrix(4, 4), cov_matrix(5, 5));


				// GT correction (currently as GPS factor)
				auto correction_noise = noiseModel::Isotropic::Sigma(3, 0.1);
				graph->add(GPSFactor(X(user.Ix), gt_pose.translation(), correction_noise));

				vals.insert(X(user.Ix), proposed.pose());
				vals.insert(V(user.Iv), proposed.v());
				vals.insert(B(user.Ib), prev_bias);
				Values result; 

				try {
					isam->update(*graph, vals);
					result = isam->calculateEstimate();
					user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
					user.est_velocitys.push_back(result.at<Vector3>(V(user.Iv))); // Assuming V and X are on same index
					user.est_poses_error.push_back(position_var);

					prev_state = NavState(result.at<Pose3>(X(user.Ix)), result.at<Vector3>(V(user.Iv)));
					prev_bias = result.at<imuBias::ConstantBias>(B(user.Ib));
				}
				catch (const std::exception& e) {
					std::cerr << "Optimizer update failed: " << e.what() << std::endl;

					// Dump factor graph to .dot file
					std::ofstream os("/home/admitriev/Research/gtsam_test/pilot_factor_graphs/factor_graph.dot");
					graph->saveGraph(os, result); // Uses current result (could also pass an empty Values())
					os.close();

					PLOT_ANCHORS(info);
					PLOT_ESTIMATED_FOR_USERS(info, show_list);

					std::cerr << "Graph dumped to factor_graph.dot" << std::endl;
					throw; // rethrow after dumping
				}

				// Here, you need to re-insert the optimization results as the base of the next preintegration.
				graph->resize(0);
				vals.clear();

				imu_preintegrated->resetIntegrationAndSetBias(prev_bias); // Clear preintegrator
				GT_CORRECTION_COUNT++;
			}
		}
		else if (mes["type"] == "uwb" && start_graph) {
			double range;
			string src_user = "2";
			string dst_user;

			uwb_counter++;

			get_UWB(mes, src_user, dst_user, range);


				vector<string> anchors = { "1", "3", "4" };
				//string dst_user = anchors[UWB_ANCHOR_IDX % 3];

				user.Ix++;
				user.Iv++;
				user.Ib++;


				//draw_vector(gt_pose.translation(), info[dst_user].gt_poses[0].translation(), "black");

				//double true_range = distance3(info[dst_user].gt_poses[0].translation(), gt_pose.translation());
				////cout << "true_range " << true_range << " measured range " << range << endl;
				////cout << "true_range " << true_range << " to anchor " << dst_user << endl;

				cout << "Integrating on " << imu_counter - last_imu_counter << " imu measurements" << endl;
				last_imu_counter = imu_counter;

				double true_range = distance3(gt_pose.translation(), info[dst_user].gt_poses[0].translation());
				graph->add(RangeFactor<Pose3, Pose3, double>(X(info[src_user].Ix), info[dst_user].pose_key, true_range, UWB_noise_model));


				/*PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);
				CombinedImuFactor imu_factor(X(user.Ix - 1), V(user.Iv - 1), X(user.Ix), V(user.Iv), B(user.Ib - 1), B(user.Ib), *current_imu_preintegration);
				graph->add(imu_factor);*/

				auto preint_imu =
					dynamic_cast<const PreintegratedImuMeasurements&>(*imu_preintegrated);
				ImuFactor imu_factor(X(user.Ix - 1), V(user.Iv - 1),
					X(user.Ix), V(user.Iv),
					B(user.Ib - 1), preint_imu);
				graph->add(imu_factor);
				imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
				graph->add(BetweenFactor<imuBias::ConstantBias>(
					B(user.Ib - 1), B(user.Ib), zero_bias,
					prior_bias_noise_model));


				auto proposed = preint_imu.predict(prev_state, prev_bias);


				// So I think Navstate 1 is supposed to be the GT pose?
				NavState gt_navstate(gt_pose, Vector3(0, gt_velocity, 0));

				// Not sure what exactly the difference is between this covariance matrix is and the computeErrors are/
				auto cov_matrix = preint_imu.preintMeasCov(); // COVARIANCE OF: [PreintROTATION PreintPOSITION PreintVELOCITY BiasAcc BiasOmega]
				//Vector3 position_var(cov_matrix(3, 3), cov_matrix(4, 4), cov_matrix(5, 5));
				Vector9 integration_error = preint_imu.computeErrorAndJacobians(gt_navstate.pose(), gt_navstate.v(), proposed.pose(), proposed.v(), prev_bias);
				Vector3 position_var(integration_error(3), integration_error(4), integration_error(5)); // Going to guess its the middle 3 elements that correspond to pose?
				// Can the documentation please explain to me exactly what this Vector9 is that gets returned, what indices correspond to what variables?

				
				vals.insert(X(user.Ix), proposed.pose());
				vals.insert(V(user.Iv), proposed.v());
				vals.insert(B(user.Ib), prev_bias);
				Values result;

				try {
					isam->update(*graph, vals);
					result = isam->calculateEstimate();
					user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
					user.est_velocitys.push_back(result.at<Vector3>(V(user.Iv))); // Assuming V and X are on same index
					user.est_poses_error.push_back(position_var);

					prev_state = NavState(result.at<Pose3>(X(user.Ix)), result.at<Vector3>(V(user.Iv)));
					prev_bias = result.at<imuBias::ConstantBias>(B(user.Ib));
				}
				catch (const std::exception& e) {
					std::cerr << "Optimizer update failed: " << e.what() << std::endl;

					// Dump factor graph to .dot file
					std::ofstream os("/home/admitriev/Research/gtsam_test/pilot_factor_graphs/factor_graph.dot");
					graph->saveGraph(os, result); // Uses current result (could also pass an empty Values())
					os.close();

					PLOT_ANCHORS(info);
					PLOT_ESTIMATED_FOR_USERS(info, show_list);

					std::cerr << "Graph dumped to factor_graph.dot" << std::endl;
					throw; // rethrow after dumping
				}

				// Here, you need to re-insert the optimization results as the base of the next preintegration.
				graph->resize(0);
				vals.clear();
				// iSam internally caches past graph and vals states it was called on, 
				// so if you don't resize, you'll get duplicate keys

				imu_preintegrated->resetIntegrationAndSetBias(prev_bias); // Clear preintegrator
			}
		
		
		
	}


	PLOT_ANCHORS(info);
	PLOT_ESTIMATED_FOR_USERS(info, show_list);

	show();

	return 0;
}

int main(int argc, char* argv[]) {

	// run_euroc();
	run_cappella();

	return 0;

	//Save (drawing example for when you get confused)
	//hold(on);

	//Pose3 test1(Rot3::Identity(), Point3(1, 1, 0));
	//Pose3 test_rot = test1 * Pose3(Rot3::AxisAngle(Point3(0, 0, 1), M_PI / 2), Vector3(0, 0, 0));
	//// So to do a relative rotation with axis angle. New Pose = Current Pose Frame * Incremental Pose <- the incremental rotation.
	//// This makes sense

	////draw_vector(Vector3(0, 0, 0), Vector3(1, 0, 0), "red"); // X
	//draw_forward(test1, 1, "red");
	//draw_forward(test_rot, 1, "black");
	//draw_vector(Vector3(0, 0, 0), Vector3(1, 0, 0), "red"); // X
	//draw_vector(Vector3(0, 0, 0), Vector3(0, 1, 0), "blue"); // Y
	//draw_vector(Vector3(0, 0, 0), Vector3(0, 0, 1), "green"); // Z

	//hold(on);
	////show();
}
