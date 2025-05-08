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

//#define PLOT_FOR_USERS(INFO, SHOW_LIST) {						           \
//    for (const auto& [user_name, user_info] : INFO) {                      \
//        if (!user_info.is_beacon) {                                        \
//            if (find(SHOW_LIST.begin(), SHOW_LIST.end(), user_name) != SHOW_LIST.end()) { \
//                                                                           \
//                hold(on);                                                  \
//                draw_trajectory(user_info.vio_poses, "red");               \
//                hold(on);                                                  \
//                draw_trajectory(user_info.gt_poses, "green");              \
//                hold(on);                                                  \
//                draw_trajectory(user_info.est_poses, "blue");              \
//                                                                           \
//                xlabel("X (m)");                                           \
//                ylabel("Z (m)");                                           \
//                zlabel("Y (m)");                                           \
//                                                                           \
//            }                                                              \
//        }                                                                  \
//    }                                                                      \
//}

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
	ifstream beacon_fs(directory + trial_name + "/" + "anchors.json");

	json sensor_stream = json::parse(raw_fs);
	map<string, tracking> info; // Map of username to tracking information


	// TODO get_gt_info but with a faked trajectory
	//get_gt_info(info, json::parse(gt_fs)); // fill user_info with gt_pose trajectory

	get_beacon_info(info, json::parse(beacon_fs));


	// --- Noise Models ---

	// VIO noise model

	double vio_ori_stdev = 0.175; // rad->~10degrees
	double vio_pos_stdev = 0.2;
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));

	// UWB noise model

	//double uwb_stdev = 0.1;
	double uwb_stdev = 1;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev); // Apparently this is the correct noise model for a range

	// GT noise model - (use to define pose prior)
	double gt_pos_stdev = 0.01;
	double gt_ori_stdev = 0.0174533;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));
	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

	// TODO: GT_velocity, and bias noise model.
	// Velocity noise model - used on prior - Noise in velocity updates from preintegration

	// Bias noise model - used on prior. - Noise in bias updates from preintegration


	// IMU noise model

	imuBias::ConstantBias prior_imu_bias; // Assumption of no prior IMU bias

	// Hard coded from IMU comparison sheet
	double GYRO_NOISE = 0.014; // deg / s / sqrt(Hz) <- should this be rad?
	double ACCEL_NOISE = 0.0014715; // m / s^2 / sqrt(Hz)

	// Hard coded from calibration.json
	Matrix33 accel_noise_cov = I_3x3 * pow(ACCEL_NOISE, 2);
	Matrix33 gyro_noise_cov = I_3x3 * pow(GYRO_NOISE, 2);
	Matrix33 noise_integration_cov = I_3x3 * 1e-8;  // error committed in integrating position from velocities
	
	Vector3 GYRO_BIAS(-0.00307518, 0.0003668, 0.00393268); // I sure hope these are in the same units as what GTSAM expects (Realsense doesn't label calibration output with units)
	Vector3 ACCEL_BIAS(-0.031682, -0.0617278, 0.02699346);
	Matrix33 accel_bias_cov = I_3x3 * Vector3(ACCEL_BIAS.array().square()); // square all elements along the diagonal
	Matrix33 gyro_bias_cov = I_3x3 * Vector3(GYRO_BIAS.array().square());
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

	// In the example they use 'c', this is just track.I for user 2.

	int pose_num = 0;
	Values vals;
	for (auto& [u, track] : info) {
		track.I = 0;

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

			Vector3 prior_velocity(0, 0, 0);

			graph->addPrior(X(track.I), Pose3(prior_rotation, prior_position), GT_noise_model);
			graph->addPrior(V(track.I), prior_velocity, prior_velocity_noise_model);
			graph->addPrior(B(track.I), prior_imu_bias, prior_bias_noise_model);
		}
	}


	// Use Preintegrator params, and bias prior, to create a new preintegrator object that we can use for an IMU factor.
	PreintegrationType* imu_preintegrated = new PreintegratedCombinedMeasurements(imu_preintegration_params, prior_imu_bias);

	//ISAM2Params isam_params;
	//isam_params.factorization = ISAM2Params::CHOLESKY;
	//isam_params.relinearizeSkip = 10;
	//ISAM2DoglegParams dogleg;
	//isam_params.optimizationParams = dogleg;
	//ISAM2* isam = new ISAM2(isam_params);

	double uwb_error = 0;
	double n_uwb_mes = 0;

	for (json mes : sensor_stream) {


		if (mes["type"] == "imu") {


		}
		else if (mes["type"] == "uwb") {
			double range;
			string src_user = "2";
			string dst_user;

			get_UWB(mes, src_user, dst_user, range);

			graph->add(RangeFactor<Pose3, Pose3, double>(X(info[src_user].I), MK_Anchor(dst_user, info[dst_user].I), range, UWB_noise_model));

		}

		//if (VIO_measurements > max_VIO_measurements) break; // TO keep the graph small and visualizable

		//isam->update(*graph, vals);
		//graph->resize(0); // According to example
		//vals.clear(); // Still don't quite get why we need this vals.clear();

	}

	double avg_uwb_error = uwb_error / n_uwb_mes;
	cout << " Average dataset UWB error (m) " << avg_uwb_error << endl;


	//LM_lambda_search(graph, vals, info);

	vector<string> show_plots_for = { "nuno", "elahe" };

	//unpack_results(isam->calculateBestEstimate(), MK, info);
	//cout << info["elahe"].gt_poses.size() << " " << info["elahe"].vio_poses.size() << " " << info["elahe"].est_poses.size() << endl;
	//PLOT_FOR_USERS(info, show_plots_for);
	//show();

	// Once graph is complete, optimize it offline


	GraphvizFormatting vizp; // To figure out the variable misalignment issue
	vizp.plotFactorPoints = true;
	//vizp.mergeSimilarFactors = true;
	vizp.binaryEdges = true;
	graph->saveGraph("/home/admitriev/Research/gtsam_test/factor_graphs/factor_graph.dot", vals, vizp);

	LevenbergMarquardtParams params;
	LevenbergMarquardtOptimizer optimizer(*graph, vals, params);

	////GaussNewtonParams params;
	////GaussNewtonOptimizer optimizer(*graph, vals, params);

	double last_error;
	do {
		last_error = optimizer.error();
		optimizer.iterate();

		//unpack_results(optimizer.values(), MK, info);
		//PLOT_FOR_USERS(info, show_plots_for);
		//clear_results(info); // clear Est_poses trajectory

	} while (!checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol, last_error, optimizer.error()));

	unpack_results(optimizer.values(), MK, info);
	PLOT_FOR_USERS(info, show_plots_for);
	clear_results(info); // clear Est_poses trajectory

	show();

	cout << " Converged in " << optimizer.iterations() << " iterations, with " << optimizer.error() << " final error." << endl; // Currently doing 4 iterations

	// TODO: Even trajectories
	// TODO: Output looks wrong:
	// HMT format last row should be 0 0 0 1
	//0.216009 - 0.671835 0.708532 - 0.81528 - 0.229042 0.670536 0.705699 5.541 - 0.949113 - 0.314649 - 0.00904673 - 13.68030001
	//0.216202 - 0.6718 0.708507 - 0.815325 - 0.228938 0.670542 0.705727 5.54088 - 0.949095 - 0.314712 - 0.00883794 - 13.68040001

	//ofstream out_gt_fs(out_directory + filename + "_out_gt.txt");
	//ofstream out_vio_fs(out_directory + filename + "_out_vio.txt");
	//ofstream out_est_fs(out_directory + filename + "_out_estimate.txt");

	//cout << "Size check " << info["nuno"].gt_poses.size() << " "
	//	<< info["nuno"].vio_poses.size() << " "
	//	<< info["nuno"].est_poses.size() << endl;

	//info["nuno"].vio_poses.pop_back();
	//write_trajectory_KITTI_format(info["nuno"].gt_poses, out_gt_fs);
	//write_trajectory_KITTI_format(info["nuno"].vio_poses, out_vio_fs);
	//write_trajectory_KITTI_format(info["nuno"].est_poses, out_est_fs);


	//out_gt_fs.flush();
	//out_gt_fs.close();	
	//out_vio_fs.flush();
	//out_vio_fs.close();	
	//out_est_fs.flush();
	//out_est_fs.close();



	return 0;
}

int main(int argc, char* argv[]) {

	// run_euroc();
	run_cappella();

	return 0;
}
