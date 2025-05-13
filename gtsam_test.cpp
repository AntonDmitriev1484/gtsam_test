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

#define PLOT_ESTIMATED_FOR_USERS(INFO, SHOW_LIST) {                          \
    for (const auto& [user_name, user_info] : INFO) {                        \
        if (!user_info.is_beacon) {                                          \
            if (std::find(SHOW_LIST.begin(), SHOW_LIST.end(), user_name) != SHOW_LIST.end()) { \
                hold(on);                                                    \
                draw_trajectory(user_info.est_poses, "blue");                \
                hold(on);                                                  \
                draw_trajectory(user_info.gt_poses, "green");              \
                                                                             \
                xlabel("X (m)");                                             \
                ylabel("Z (m)");                                             \
                zlabel("Y (m)");                                             \
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
                ylabel("Z (m)");                                           \
                zlabel("Y (m)");                                           \
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

Rot3 rotFromDirection(const Point3& direction) {
	// Normalize direction vector
	Point3 z = direction / direction.norm();

	// Choose arbitrary up vector
	Point3 up(0, 0, 1);
	if (fabs(z.dot(up)) > 0.99) {
		up = Point3(0, 1, 0);
	}

	Point3 x = up.cross(z).normalized();
	Point3 y = z.cross(x);

	return Rot3(x, y, z);
}



int run_cappella() {

	string directory = "/home/admitriev/Datasets/UWBSLAM_pilot/";
	string trial_name = "pilot0";
	string out_directory = "/home/admitriev/Research/pilot_results/" + trial_name;

	ifstream raw_fs(directory + trial_name + "/" + "all.json");
	ifstream beacon_fs(directory + "pilot_anchors.json");

	json sensor_stream = json::parse(raw_fs);
	map<string, tracking> info; // Map of username to tracking information


	// TODO get_gt_info but with a faked trajectory
	//get_gt_info(info, json::parse(gt_fs)); // fill user_info with gt_pose trajectory

	get_beacon_info(info, json::parse(beacon_fs));

	// TODO add user to the map

	info.insert(pair<string, tracking>("2", tracking()));

	double dt = 1.0 / 200.0; // IMU gyro and accelerometer operate at 200Hz


	// --- Noise Models ---

	// VIO noise model

	double vio_ori_stdev = 0.175; // rad->~10degrees
	double vio_pos_stdev = 0.2;
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));

	// UWB noise model

	double uwb_stdev = 0.1;
	//double uwb_stdev = 1;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev); // Apparently this is the correct noise model for a range

	// GT noise model - (use to define pose prior)
	double gt_pos_stdev = 0.01;
	double gt_ori_stdev = 0.01;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));
	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.01);
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 0.01);


	// IMU noise model

	imuBias::ConstantBias prior_imu_bias(Vector6(1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4)); // Assumption of no prior IMU bias

	// Realsense Gyro is in radians / sec: https://support.intelrealsense.com/hc/en-us/community/posts/9489403831059-d435i-gyro-data-unit 

	// Hard coded from IMU comparison sheet
	double GYRO_NOISE = 0.014 * 3.14/180; // deg / s / sqrt(Hz) -> Since realsense gyro returns data in rad, GYRO_NOISE should also be given in rad
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
	boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
	imu_preintegration_params->accelerometerCovariance = accel_noise_cov;
	imu_preintegration_params->integrationCovariance = noise_integration_cov;
	imu_preintegration_params->gyroscopeCovariance = gyro_noise_cov;
	imu_preintegration_params->biasAccCovariance = accel_bias_cov;
	imu_preintegration_params->biasOmegaCovariance = gyro_bias_cov;
	imu_preintegration_params->biasAccOmegaInt = initial_bias_cov;


	NonlinearFactorGraph* graph = new NonlinearFactorGraph();

	// Beacon info is a string, 
	// Make Key
	const function<Key(string, int)> MK_Anchor = [](string name, int I) {
		return symbol('s', stoi(name));
	};


	// Establish and attach priors to keys

	Values vals;

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
		}
		else { // Since we only have one user, user 2.

			Point3 prior_position(0, 0, 1.3); // I was carrying laptop at about chest level ~130cm off the ground

			Vector3 start_pointing_direction(0, 1, 0);
			Rot3 prior_rotation(rotFromDirection(start_pointing_direction)); // Pointing forward about the y-axis. Vector3(0,1,0) -> turn this into a quat 
			Pose3 start_pose(prior_rotation, prior_position);

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

		//track.I++;
	}


	// Use Preintegrator params, and bias prior, to create a new preintegrator object that we can use for an IMU factor.
	PreintegrationType* imu_preintegrated = new PreintegratedCombinedMeasurements(imu_preintegration_params, prior_imu_bias);

	ISAM2Params isam_params;
	isam_params.factorization = ISAM2Params::CHOLESKY;
	isam_params.relinearizeThreshold = 0.01;
	isam_params.relinearizeSkip = 1;
	ISAM2DoglegParams dogleg;
	isam_params.optimizationParams = dogleg;
	ISAM2* isam = new ISAM2(isam_params);


	tracking user = info["2"];
	NavState prev_state(user.est_poses.back(), user.est_velocitys.back());

	double gt_velocity = 12.0 / 30.0;
	Pose3 gt_pose = user.est_poses[0];
	double middle_timestamp = 15000;

	int imu_counter = 0;
	bool initialization_complete = true;

	for (json mes : sensor_stream) {


		if (mes["type"] == "imu") {
			Vector3 accel;
			Vector3 gyro;
			get_IMU(mes, accel, gyro);
			imu_preintegrated->integrateMeasurement(accel, gyro, dt);
			imu_counter++;
			
			// Generate GT points
			// Assume around 15 seconds in we go the opposite way

			// Around 200 IMU measurements per second
			double dy = gt_velocity * dt;

			if (mes["t"] > middle_timestamp) { dy *= -1; }
			Pose3 motion( Rot3::Identity(), Vector3(0, dy, 0));
			gt_pose = gt_pose * motion;
			user.gt_poses.push_back(gt_pose);


		}
		else if (mes["type"] == "uwb") {
			double range;
			string src_user = "2";
			string dst_user;
			get_UWB(mes, src_user, dst_user, range);

			user.Ix++;
			user.Ib++;
			user.Iv++;

			//graph->add(RangeFactor<Pose3, Pose3, double>(X(info[src_user].Ix), MK_Anchor(dst_user, info[dst_user].Ix), range, UWB_noise_model));

			PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);
			CombinedImuFactor imu_factor(X(user.Ix - 1), V(user.Iv - 1), X(user.Ix), V(user.Iv), B(user.Ib - 1), B(user.Ib), *current_imu_preintegration);
			graph->add(imu_factor);



			//graph->addPrior(X(user.Ix), gt_pose, GT_noise_model); // Add the most recently added gt_pose as a prior -> Trying to avoid indeterminant linear system
			auto correction_noise = noiseModel::Isotropic::Sigma(3, 1.0);
			graph->add(GPSFactor(X(user.Ix), gt_pose.translation(), correction_noise));


			auto proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);

			// Note: In example, they don't add any priors at all here.
			////graph->addPrior(X(user.Ix), proposed.pose(), VIO_pose_noise_model);
			//graph->addPrior(V(user.Iv), proposed.velocity(), prior_velocity_noise_model);
			//graph->addPrior(B(user.Ib), user.constant_bias, prior_bias_noise_model);

			vals.insert(X(user.Ix), proposed.pose());
			vals.insert(V(user.Iv), proposed.v());
			vals.insert(B(user.Ib), user.constant_bias);

			Values result;
			try {
				//isam->update(*graph, vals);
				//Values result = isam->calculateEstimate();
				//Pose3 estimated_pose = result.at<Pose3>(X(user.Ix));
				//Vector3 estimated_velocity = result.at<Vector3>(V(user.Ix));

				//user.est_poses.push_back(estimated_pose);
				//user.est_velocitys.push_back(estimated_velocity);
				//prev_state = NavState(estimated_pose, estimated_velocity);


				LevenbergMarquardtParams params;
				LevenbergMarquardtOptimizer optimizer(*graph, vals, params);
				Values result = optimizer.optimize();

				info["2"].est_poses.clear();

				for (auto& [user, user_info] : info) { //Unpacks results
					for (int i = 0; i < user_info.Ix; i++) {
						if (!user_info.is_beacon) {
							Key k = X(i);
							Pose3 estimated_pose = result.at<Pose3>(k);
							user_info.est_poses.push_back(estimated_pose);
						}
					}
				}

			}
			catch (const std::exception& e) {
				std::cerr << "Optimizer update failed: " << e.what() << std::endl;

				// Dump factor graph to .dot file
				std::ofstream os("/home/admitriev/Research/gtsam_test/pilot_factor_graphs/factor_graph.dot");
				graph->saveGraph(os, result); // Uses current result (could also pass an empty Values())
				os.close();

				std::cerr << "Graph dumped to factor_graph.dot" << std::endl;
				//throw; // rethrow after dumping
			}


			//graph->resize(0); // Why these 2?
			//vals.clear();


			imu_preintegrated->resetIntegrationAndSetBias(user.constant_bias); // Clear preintegrator
		}

		//if (imu_counter % 50 == 0 && initialization_complete) {
			//PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);

		//	//They increment I up here

		//	user.Ix++;
		//	user.Ib++;
		//	user.Iv++;

		//	CombinedImuFactor imu_factor(X(user.Ix - 1), V(user.Iv - 1), X(user.Ix), V(user.Iv), B(user.Ib - 1), B(user.Ib), *current_imu_preintegration);
		//	graph->add(imu_factor);

		//	graph->addPrior(X(user.Ix), gt_pose, GT_noise_model); // Add the most recently added gt_pose as a prior -> Trying to avoid indeterminant linear system

		//	auto proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);

		//	//graph->addPrior(X(user.Ix), proposed.pose(), VIO_pose_noise_model);
		//	graph->addPrior(V(user.Iv), proposed.velocity(), prior_velocity_noise_model);
		//	graph->addPrior(B(user.Ib), user.constant_bias, prior_bias_noise_model);

		//	vals.insert(X(user.Ix), proposed.pose());
		//	vals.insert(V(user.Iv), proposed.v());
		//	vals.insert(B(user.Ib), user.constant_bias);


		//	isam->update(*graph, vals);
		//	Values result = isam->calculateEstimate();
		//	Pose3 estimated_pose = result.at<Pose3>(X(user.Ix));
		//	Vector3 estimated_velocity = result.at<Vector3>(V(user.Ix));

		//	user.est_poses.push_back(estimated_pose);
		//	user.est_velocitys.push_back(estimated_velocity);
		//	prev_state = NavState(estimated_pose, estimated_velocity);

		//	//graph->resize(0); // Why these 2?
		//	//vals.clear();
		//	imu_preintegrated->resetIntegrationAndSetBias(user.constant_bias); // Clear preintegrator


		//}

		//if (imu_counter == 200 * 10) {

		//	user.Ix++;

		//	PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);

		//	auto proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);
		//	vals.insert(X(user.Ix), proposed.pose());
		//	vals.insert(V(user.Ix), proposed.v());
		//	vals.insert(B(user.Ix), user.constant_bias);


		//	CombinedImuFactor imu_factor(X(user.Ix - 1), V(user.Ix - 1), X(user.Ix), V(user.Ix), B(user.Ix - 1), B(user.Ix), *current_imu_preintegration);
		//	graph->add(imu_factor);

		//	LevenbergMarquardtParams params;
		//	LevenbergMarquardtOptimizer lm(*graph, vals, params);
		//	Values result = lm.optimize();

		//	Pose3 estimated_pose = result.at<Pose3>(X(user.Ix));
		//	Vector3 estimated_velocity = result.at<Vector3>(V(user.Ix));

		//	isam->update(*graph, result); // This should either be result or vals?
		//	// Re-sizing vs not re-sizing the graph makes no difference in hitting indeterminant system
		//	//graph->resize(0); // Not sure if I'm initializing isam properly here... But still indeterminant
		//	vals.clear();

		//	user.est_poses.push_back(estimated_pose);
		//	user.est_velocitys.push_back(estimated_velocity);
		//	prev_state = NavState(estimated_pose, estimated_velocity);

		//	imu_preintegrated->resetIntegrationAndSetBias(user.constant_bias); // Clear preintegrator


		//	initialization_complete = true;
		//}



	}

	vector<string> show_list = { "2" };

	PLOT_ESTIMATED_FOR_USERS(info, show_list);

	show();

	return 0;
}

int main(int argc, char* argv[]) {

	// run_euroc();
	run_cappella();

	return 0;
}
