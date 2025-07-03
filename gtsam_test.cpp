#include "gtsam_test.h"
#include "data_tools.h"
#include "utils.h"
#include "cmath"
#include <regex>

using PreintegrationType = gtsam::PreintegrationBase;
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

		isam->update(*graph, vals);
		result = isam->calculateEstimate();

		user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
		user.est_velocitys.push_back(result.at<Vector3>(V(user.Iv)));
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
	Vector3 last_velocity = user.est_velocitys.back();

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
		isam->update(*graph, vals);
		result = isam->calculateEstimate();

		user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
		user.est_velocitys.push_back(result.at<Vector3>(V(user.Iv)));

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
	json gt_mes,
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
	string debug_dump_directory)
{
	cout << "Processing synthetic range for : " << gt_mes["t"] << endl;
	uwb_counter++;

	vector<string> users = {"2", "3", "5"};
	string dst_user = users[uwb_counter % 3];

	tracking& user = info[src_user];
	tracking& dst = info[dst_user];

		// Extract GT pose
	Matrix44 gt_pose_slam;
	string usrname;
	get_pose_matrix(gt_mes, usrname, gt_pose_slam);
	Pose3 gt_pose = Pose3(gt_pose_slam) * T_imu_body.inverse(); // Transform pose to the body frame.
	user.gt_poses.push_back(gt_pose);


	user.Ix++;
	user.Iv++;
	user.Ib++;

	// Ground truth range from GT poses
	double true_range = distance3(
		gt_pose.translation(),
		info[dst_user].gt_poses.back().translation());

	// Add UWB (range) factor — use ground truth here for stability
	graph->add(RangeFactor<Pose3, Pose3, double>(
		X(user.Ix), dst.pose_key, true_range, UWB_noise_model));
	cout << "Added Range factor " << graph->size() - 1 << endl;

	// Add fake GT-style prior to help stabilize
	graph->add(PriorFactor<Pose3>(X(user.Ix), gt_pose, FakePrior_noise_model));
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
		isam->update(*graph, vals);
		result = isam->calculateEstimate();

		user.est_poses.push_back(result.at<Pose3>(X(user.Ix)));
		user.est_velocitys.push_back(result.at<Vector3>(V(user.Iv)));

		cout << "Successful estimate on UWB factor" << endl;
	}
	catch (const std::exception& e) {
		cerr << "Optimizer update failed: " << e.what() << endl;
		cerr << "Data timestamp is " << gt_mes["t"] << endl;
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


int main(int argc, char* argv[]) {


	if (argc != 5) {
		// ex. stereoi_circle2 synthetic_20_60 uwb true
        std::cerr << "Usage: " << argv[0] << " <trial_name> <synthetic_trial_name or 'none'> <'uwb' or 'no_uwb'> <dump (true|false)>" << std::endl;
        return 1;
    }
	std::string trial_name = argv[1];
    std::string synthetic_trial_name = argv[2];
    std::string dump_str = argv[4];
	std::string uwb_str = argv[3];

    bool log_dump = (dump_str == "true");
	bool use_uwb = (uwb_str == "uwb");
	bool use_gt = true;
	bool synthetic = synthetic_trial_name != "none";


	string data_dir = "/home/antond2/ws/post/out/"+trial_name+"_post";
	string out_dir = "/home/antond2/Desktop/Research/gtsam_test/out_results/"+trial_name;
	if (synthetic) {
		data_dir += "/synthetic";
		out_dir += "/"+synthetic_trial_name;
		if (use_uwb) { out_dir += "_uwb";}
	}
	string debug_dir = out_dir+"/debug";

	ifstream raw_fs;
	if (synthetic) {
		raw_fs = ifstream(data_dir + "/all_" + synthetic_trial_name +".json");
	}
	else {
		raw_fs = ifstream(data_dir + "/all.json");
	}
	ifstream beacon_fs("/home/antond2/ws/post/out/"+trial_name+"_post" + "/anchors.json");
	ifstream transform_fs("/home/antond2/ws/post/out/"+trial_name+"_post" + "/transforms.json");

	ofstream estimated_trajectory_fs(out_dir + "/est.txt");
	ofstream slam_trajectory_fs(out_dir+"/slam.txt");
	ofstream log_dump_fs(out_dir + "/log_dump.txt");

	vector<string> paths = {out_dir, debug_dir};
	for (string path: paths) {
		if (!std::filesystem::exists(path)) {
				std::filesystem::create_directories(path);
				std::cout << "Directory created: " << path << std::endl;
		}
	}

	// If log_dump. Redirect stdout output to a text file.
	if (log_dump) {
		std::cout.rdbuf(log_dump_fs.rdbuf()); // redirect cout to file
	}

	cout << "In path " << data_dir << endl;
	cout << "Out path " << out_dir << endl;



	json sensor_stream = json::parse(raw_fs);
	map<string, tracking> info; // Map of username to tracking information

	get_beacon_info(info, json::parse(beacon_fs));
	info.insert(pair<string, tracking>("1", tracking()));

	double dt = 1.0 / 200.0; // IMU gyro and accelerometer operate at 200Hz


	// --- Noise Models ---

	// VIO noise model

	double vio_ori_stdev = 0.175;
	double vio_pos_stdev = 0.2;
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));

	// UWB noise model

	double uwb_stdev = 0.1;
	// double uwb_stdev = 0.5;
	// They set this to 100 or 1000 in this example: https://github.com/borglab/gtsam/blob/develop/examples/RangeISAMExample_plaza2.cpp
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	// GT noise model - (use to define pose prior)
	double gt_pos_stdev = 1e-2;
	double gt_ori_stdev = 1e-2;
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


	std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = PreintegratedCombinedMeasurements::Params::MakeSharedU();
	// std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = std::make_shared<PreintegratedCombinedMeasurements::Params>(Vector3(0, 9.81, 0));
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
	Pose3 T_imu_body(Rot3(transform), Vector3(0, 0, 0));
	// body_P_sensor : "pose of sensor frame w.r.t body frame"

	// auto trans = json::parse(transform_fs);
	// Matrix44 pose_matrix;
	// int i = 0;
	// int j = 0;
	// for (const auto& row : trans["T_slam_world"]) {
	// 	if (row.is_array()) {
	// 		for (const double& element : row) {
	// 			//HTM_L_G(i, j) = static_cast<double>(element.get<float>());
	// 			pose_matrix(i, j) = element;
	// 			j++;
	// 		}
	// 	}
	// 	j = 0;
	// 	i++;
	// }
	// Pose3 T_imu_world(pose_matrix);
	// Pose3 T_imu_body(T_imu_world.rotation(), Vector3(0, 0, 0));

	imu_preintegration_params->body_P_sensor = T_imu_body;

	NonlinearFactorGraph* graph = new NonlinearFactorGraph();

	// Beacon info is a string, 
	// Make Key
	const function<Key(string, int)> MK_Anchor = [](string name, int I) {
		return symbol('s', stoi(name));
	};

	// Establish and attach priors to keys

	Values vals;
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
				if (mes["type"] == "slam_pose") {
					Matrix44 gt_pose_world;
					string usrname;
					get_pose_matrix(mes, usrname, gt_pose_world);

					//Pose3 gt_pose = slam_to_world * gt_pose_slam;
					Pose3 gt_pose(gt_pose_world);

					find_first_gt_pose = gt_pose;
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
			track.constant_bias = prior_imu_bias;

		}

	}

	vector<string> show_list = { "1" };
	vector<string> anchors = { "2", "3", "5" };

	// Use Preintegrator params, and bias prior, to create a new preintegrator object that we can use for an IMU factor.
	PreintegrationType* imu_preintegrated = new PreintegratedCombinedMeasurements(imu_preintegration_params, prior_imu_bias);

	ISAM2Params isam_params;
	// Must be some way to set up a verbose option.
	isam_params.factorization = ISAM2Params::QR;
	isam_params.relinearizeThreshold = 0.01;
	isam_params.relinearizeSkip = 1; // More informed optimization at the cost of more computing.
	isam_params.enableDetailedResults = true;
	ISAM2DoglegParams dogleg;
	isam_params.optimizationParams = dogleg;
	ISAM2* isam = new ISAM2(isam_params);
	isam->update(*graph, vals);

	graph->resize(0);
	vals.clear();

	tracking& user = info.at("1");
	NavState prev_state(user.est_poses.back(), user.est_velocitys.back());


	int imu_counter = 0;
	int last_imu_counter = 0;
	bool initialization_complete = true;
	bool start_graph = false;

	int T_UWB = 10; // Every X IMU measurements, generate 1 synthetic UWB measurement.
	int uwb_counter = 0;
	
	int gt_correction_hz_max = 200; // This will be the frequency that you interpolated to, 20 by default
	double gt_correction_hz = 20;
	int T_GT = (int) (gt_correction_hz_max / gt_correction_hz); // Every X SLAM measurements, generate 1 GT correction
	int gt_counter = 0;


	int imu_count_at_last_correction = 0;
	int imu_count_at_last_imu_factor = 0;
	int factor_counter = 0;
	int gt_skipped = 0;

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

			cout << "Preintegration on at " << mes["t"] << " a: " << accel.x() << " " << accel.y() << " " << accel.z() << ", g: " << gyro.x() << " " << gyro.y() << " " << gyro.z() << endl;

			// Just for plotting
			PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated);
			auto proposed = current_imu_preintegration->predict(prev_state, user.constant_bias);
			user.est_poses.push_back(proposed.pose());

			for (json mes : gt_pose_buffer) {
				processGT(mes, user, graph, vals, isam, imu_preintegrated, prev_state, GT_noise_model, T_imu_body, debug_dir);

				imu_count_at_last_correction = imu_counter;
				imu_count_at_last_imu_factor = imu_counter;
				gt_counter++;
			}
			gt_pose_buffer.clear();

			for (json mes : range_buffer) {

				if (synthetic) {
					processSyntheticUWB(mes, "1", info, graph, vals, isam, imu_preintegrated, prev_state,
					UWB_noise_model, GT_noise_model, GT_noise_model,
					imu_counter, imu_count_at_last_correction, dt, uwb_counter, T_imu_body, debug_dir);

					uwb_counter++;
				}
				else {
					processUWB(mes, "1", info, graph, vals, isam, imu_preintegrated, prev_state, 
						UWB_noise_model, GT_noise_model,
						imu_counter, imu_count_at_last_correction, dt, uwb_counter, debug_dir);
				}

			}
			range_buffer.clear();
		}
		else if (use_gt && mes["type"] == "slam_pose" && start_graph) {

			if (imu_counter == imu_count_at_last_imu_factor) {
				// Pass this measurement and buffer it until the next IMU becomes available
				cout << " Skipped SLAM pose " << endl;
				gt_skipped ++; 
				gt_pose_buffer.push_back(mes);
				continue;
			}
			processGT(mes, user, graph, vals, isam, imu_preintegrated, prev_state, GT_noise_model, T_imu_body, debug_dir);

			imu_count_at_last_correction = imu_counter;
			imu_count_at_last_imu_factor = imu_counter;

			gt_counter++;
			
		}
		else if (use_uwb && mes["type"] == "uwb" && start_graph) {

			if (imu_counter == imu_count_at_last_correction) { continue; }
			if (imu_counter == imu_count_at_last_imu_factor) {
				// Pass this measurement and buffer it until the next IMU becomes available
				range_buffer.push_back(mes);
				continue;
			}
			else {
				processUWB(mes, "1", info, graph, vals, isam, imu_preintegrated, prev_state,
					UWB_noise_model, GT_noise_model,
					imu_counter, imu_count_at_last_correction, dt, uwb_counter, debug_dir);
				uwb_counter++;
			}

			imu_count_at_last_imu_factor = imu_counter;
		}
		else if (synthetic && use_uwb && mes["type"] == "synthetic_uwb" && start_graph) {


			if (imu_counter == imu_count_at_last_correction) { continue; }
			if (imu_counter == imu_count_at_last_imu_factor) {
				// Pass this measurement and buffer it until the next IMU becomes available
				range_buffer.push_back(mes);
				continue;
			}
			else {
				processSyntheticUWB(mes, "1", info, graph, vals, isam, imu_preintegrated, prev_state,
					UWB_noise_model, GT_noise_model, GT_noise_model,
					imu_counter, imu_count_at_last_correction, dt, uwb_counter, T_imu_body, debug_dir);
			}

			imu_count_at_last_imu_factor = imu_counter;
		}
		
	}

	write_trajectory_TUM_format( user.est_poses, estimated_trajectory_fs);
	estimated_trajectory_fs.close();

	write_trajectory_TUM_format( user.gt_poses, slam_trajectory_fs);
	slam_trajectory_fs.close();

	cout << " Applied " << uwb_counter << " uwb measurements for 45 seconds of data " << endl;
	double fuwb = uwb_counter /45.0;
	cout << " UWB frequency in the graph is " << fuwb << endl;

	cout << " Applied " << gt_counter << " slam measurements for 45 seconds of data " << endl;
	double fgt = gt_counter /45.0;
	cout << " GT frequency in the graph is " << fgt << endl;
	cout << " GT skipped " << gt_skipped << endl;

	return 0;
}
