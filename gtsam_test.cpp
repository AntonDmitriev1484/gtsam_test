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

// Note: All plotting code is in macros because Matplot++ behaves weirdly if
// plotting code gets ran outside of the main file.

#define PLOT_ANCHORS(INFO) {\
	for (const auto& [user_name, user_info] : INFO) {                        \
        if (user_info.is_beacon) {                                          \
			hold(on);														\
            draw_points(user_info.gt_poses, "red");                      \
        }                                                                    \
    }																		\
}

// Plots only GT and estimated trajectory
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
				xlim({ -3.5,3.5 }); \
				ylim({ -1,6 }); \
				/*zlim({ 0,7 }); */ \
            }                                                                \
        }                                                                    \
    }                                                                        \
}

// Plots GT, Drifted, and Estimated trajectory
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

int main(int argc, char* argv[]) {
	string directory = "/home/admitriev/Datasets/UWBSLAM_pilot/";
	string trial_name = "pilot1";
	string out_directory = "/home/admitriev/Research/pilot_results/" + trial_name;

	string debug_dot_dump_directory = "/home/admitriev/Research/gtsam_test/pilot_factor_graphs/factor_graph.dot";

	ifstream raw_fs(directory + trial_name + "/" + "all.json");
	ifstream beacon_fs(directory + "pilot_anchors.json");

	json sensor_stream = json::parse(raw_fs);
	map<string, tracking> info; // Map of username to tracking information

	get_beacon_info(info, json::parse(beacon_fs)); // I think this is reading beacon positions in properly

	info.insert(pair<string, tracking>("2", tracking()));

	// So 'info' contains the poses of all anchors and users. 
	// To get a user's tracking information just .get() their id.
	// The only user in the pilot is user 2.


	// --- Noise Models ---

	// UWB noise model

	double uwb_stdev = 0.1;
	//double uwb_stdev = 1;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	// GT noise model - (use to define pose prior)
	double gt_pos_stdev = 0.01;
	double gt_ori_stdev = 0.01;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));
	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.01);
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 0.01);


	// IMU noise model


	double dt = 1.0 / 200.0; // IMU gyro and accelerometer operate at 200Hz
	imuBias::ConstantBias prior_imu_bias; // Assumption of no prior IMU bias

	// Hard coded from IMU comparison sheet: https://docs.google.com/spreadsheets/d/1KIv1S0s0iet4aIrxCmkeSnDw0nlNK1Se2YtJA5IopXc/edit?usp=sharing
	double GYRO_NOISE = 0.014 * M_PI / 180; // deg / s / sqrt(Hz) -> Since realsense gyro returns data in rad, GYRO_NOISE should also be given in rad
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

	// Transform I was originally running with
	Matrix33 negate_x_axis;
	negate_x_axis << -1, 0, 0,
					0, 1, 0,
					0, 0, 1;
	// 90 about x-axis, then negate x-axis
	Pose3 sensor_to_body_transform( (Rot3(negate_x_axis) * Rot3::AxisAngle(Point3(1, 0, 0), +M_PI / 2)).inverse(), Vector3(0, 0, 0));

	// Transform that Jose calculated
	//Matrix33 transform;
	//transform << 1, 0, 0,
	//	0, 0, 1,
	//	0, -1, 0;
	//Pose3 sensor_to_body_transform(Rot3(transform), Vector3(0, 0, 0));
	// body_P_sensor : "pose of sensor frame w.r.t body frame"
	imu_preintegration_params->body_P_sensor = sensor_to_body_transform;


	NonlinearFactorGraph* graph = new NonlinearFactorGraph();

	// Lambda to make a key for UWB anchors.
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
			track.pose_key = MK_Anchor(u, 0);
			Pose3 prior_beacon_pose(track.gt_poses[0]);
			vals.insert(track.pose_key, prior_beacon_pose);
			graph->add(NonlinearEquality<Pose3>(track.pose_key, prior_beacon_pose));
			//graph->add(PriorFactor<Pose3>(track.pose_key, prior_beacon_pose, GT_noise_model));
		}
		else { // Since we only have one user, user 2.

			Point3 prior_position(-1.7, 2.35, 1.3); // I was carrying laptop at about chest level ~130cm off the ground
			Pose3 start_pose(Rot3::AxisAngle(Point3(0, 0, 1), -M_PI / 2) * Rot3::Identity(), prior_position); // Body frame aligned with the +y axis of my world frame
			gt_pose = start_pose;

			Vector3 prior_velocity(0, 0, 0);

			vals.insert(X(track.Ix), start_pose);
			vals.insert(V(track.Iv), prior_velocity);
			vals.insert(B(track.Ib), prior_imu_bias);

			graph->addPrior(X(track.Ix), start_pose, GT_noise_model);
			graph->addPrior(V(track.Iv), prior_velocity, prior_velocity_noise_model);
			graph->addPrior(B(track.Ib), prior_imu_bias, prior_bias_noise_model);

			track.gt_poses.push_back(gt_pose);
			track.est_poses.push_back(start_pose); // We'll take the estimate out of values and put it here.
			track.est_velocities.push_back(prior_velocity);
			track.constant_bias = prior_imu_bias;

		}

	}

	// For plotting trajectories, only show user 2.
	vector<string> show_list = { "2" };

	// Use Preintegrator params, and bias prior, to create a new preintegrator object that we can use for an IMU factor.
	std::shared_ptr<PreintegrationType> imu_preintegrated = std::make_shared<PreintegratedImuMeasurements>(imu_preintegration_params, prior_imu_bias);

	// Define optimizer
	ISAM2Params isam_params; // Suggested per example: https://github.com/borglab/gtsam/blob/develop/examples/IMUKittiExampleGPS.cpp
	isam_params.factorization = ISAM2Params::QR;
	isam_params.relinearizeThreshold = 0.01;
	isam_params.relinearizeSkip = 1;
	ISAM2DoglegParams dogleg; // Dogleg optimizer suggested here: https://faculty.cc.gatech.edu/~dhekne/Robust_Indoor_Localization_with_Ranging_IMU_Fusion_ICRA_2024.pdf
	isam_params.optimizationParams = dogleg;
	ISAM2* isam = new ISAM2(isam_params);


	// Parameters for generating GT
	double gt_velocity = 0.69777;
	bool on_side1 = true; // start by walking side1
	double side1 = 2.63;
	double side2 = 3.65;
	double total_distance_walked = 31.4;
	double distance_walked = 0;
	double distance_walked_at_last_turn = 0;
	double dy = gt_velocity * dt;

	// Synthetic GT and UWB frequency controls:
	int T_UWB = 10; // Every T_UWB IMU measurements, generate 1 synthetic UWB measurement.
	int T_CORRECTION = 200; // Every T_CORRECTION IMU measurements correct with GT. 200 IMU measurements per second.

	// Counters
	int GT_CORRECTION_COUNT = 0;
	bool USE_UWB = true;
	int UWB_COUNT = 0;
	int IMU_COUNT = 0;
	int last_imu_counter = 0;
	bool start_graph = false;


	// Variables for tracking estimated pose.
	tracking& user = info.at("2"); // The user in pilot0 and 1 had anchor id #2
	NavState prev_state(user.est_poses.back(), user.est_velocities.back());
	imuBias::ConstantBias prev_bias = prior_imu_bias;


	for (json mes : sensor_stream) {

		if (mes["type"] == "imu") {

			// Add IMU measurement
			start_graph = true;
			Vector3 accel;
			Vector3 gyro;
			get_IMU(mes, accel, gyro);
			imu_preintegrated->integrateMeasurement(accel, gyro, dt);
			IMU_COUNT++;

			// GT generation
			if (on_side1) {
				if (distance_walked - distance_walked_at_last_turn >= side1) {
					gt_pose = gt_pose * Pose3(Rot3::AxisAngle(Point3(0, 0, 1), M_PI / 2), Vector3(0, 0, 0));
					distance_walked_at_last_turn = distance_walked;
					on_side1 = !on_side1;
				}
			}
			else {
				if (distance_walked - distance_walked_at_last_turn >= side2) {
					gt_pose = gt_pose * Pose3(Rot3::AxisAngle(Point3(0, 0, 1), M_PI / 2), Vector3(0, 0, 0));
					distance_walked_at_last_turn = distance_walked;
					on_side1 = !on_side1;
				}
			}
			Pose3 delta_pose(Rot3::Identity(), Vector3(0, dy, 0));
			gt_pose = gt_pose * delta_pose;
			user.gt_poses.push_back(gt_pose);
			distance_walked += dy;


			// Periodically generate a GT correction
			if (IMU_COUNT % T_CORRECTION == 0) {

				user.Ix++;
				user.Iv++;
				user.Ib++;


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
				//graph->add(PriorFactor(X(user.Ix), gt_pose, GT_noise_model));
				GT_CORRECTION_COUNT++;

				draw_frame(proposed.pose(), 0.1, "black"); // Draw the coordinate frame axes of a pose for debugging
				vals.insert(X(user.Ix), proposed.pose());
				vals.insert(V(user.Iv), proposed.v());
				vals.insert(B(user.Ib), prev_bias);
				Values result;

				try {
					isam->update(*graph, vals);
					result = isam->calculateEstimate();
					user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
					user.est_velocities.push_back(result.at<Vector3>(V(user.Iv))); // Assuming V and X are on same index
					user.est_poses_error.push_back(position_var);

					prev_state = NavState(result.at<Pose3>(X(user.Ix)), result.at<Vector3>(V(user.Iv)));
					prev_bias = result.at<imuBias::ConstantBias>(B(user.Ib));

					// Here, you need to re-insert the optimization results as the base of the next preintegration.
					graph->resize(0);
					vals.clear();
					imu_preintegrated->resetIntegrationAndSetBias(prev_bias); // Clear preintegrator

				}
				catch (const std::exception& e) {
					std::cerr << "Optimizer update failed: " << e.what() << std::endl;

					// Dump factor graph to .dot file
					std::ofstream os(debug_dot_dump_directory);
					graph->saveGraph(os, result); // Uses current result (could also pass an empty Values())
					os.close();

					PLOT_ANCHORS(info);
					PLOT_ESTIMATED_FOR_USERS(info, show_list);

					std::cerr << "Graph dumped to factor_graph.dot" << std::endl;
					throw; // rethrow after dumping
				}
			}
		}
		else if (USE_UWB && mes["type"] == "uwb" && start_graph) {
			double range;
			string src_user = "2";
			string dst_user;

			get_UWB(mes, src_user, dst_user, range);

			vector<string> anchors = { "1", "3", "4" };

			user.Ix++;
			user.Iv++;
			user.Ib++;

			last_imu_counter = IMU_COUNT;

			// Code that generates a synthetic range:
			//double true_range = distance3(gt_pose.translation(), info[dst_user].gt_poses[0].translation());
			
			//graph->add(RangeFactor<Pose3, Pose3, double>(X(info[src_user].Ix), info[dst_user].pose_key, true_range, UWB_noise_model));
			//UWB_COUNT++;

			auto preint_imu = dynamic_cast<const PreintegratedImuMeasurements&>(*imu_preintegrated);
			ImuFactor imu_factor(X(user.Ix - 1), V(user.Iv - 1),
				X(user.Ix), V(user.Iv),
				B(user.Ib - 1), preint_imu);
			graph->add(imu_factor);


			imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
			graph->add(BetweenFactor<imuBias::ConstantBias>(
				B(user.Ib - 1), B(user.Ib), zero_bias,
				prior_bias_noise_model));


			auto proposed = preint_imu.predict(prev_state, prev_bias);

			vals.insert(X(user.Ix), proposed.pose());
			vals.insert(V(user.Iv), proposed.v());
			vals.insert(B(user.Ib), prev_bias);
			Values result;

			try {
				isam->update(*graph, vals);
				result = isam->calculateEstimate();
				user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
				user.est_velocities.push_back(result.at<Vector3>(V(user.Iv))); // Assuming V and X are on same index

				prev_state = NavState(result.at<Pose3>(X(user.Ix)), result.at<Vector3>(V(user.Iv)));
				prev_bias = result.at<imuBias::ConstantBias>(B(user.Ib));

				graph->resize(0);
				vals.clear();
				imu_preintegrated->resetIntegrationAndSetBias(prev_bias);


			}
			catch (const std::exception& e) {
				std::cerr << "Optimizer update failed: " << e.what() << std::endl;

				// Dump factor graph to .dot file
				std::ofstream os(debug_dot_dump_directory);
				graph->saveGraph(os, result); // Uses current result (could also pass an empty Values())
				os.close();

				PLOT_ANCHORS(info);
				PLOT_ESTIMATED_FOR_USERS(info, show_list);

				std::cerr << "Graph dumped to factor_graph.dot" << std::endl;
				throw; // rethrow after dumping
			}
		}

	}


	PLOT_ANCHORS(info);
	PLOT_ESTIMATED_FOR_USERS(info, show_list);

	show();

	return 0;
}
