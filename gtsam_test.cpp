#include "gtsam_test.h"
#include "data_tools.h"
#include "utils.h"
#include "cmath"
#include "tracker.h"
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


int main(int argc, char* argv[]) {


	if (argc != 6) {
		// ex. stereoi_circle2 synthetic_20_60 uwb true
        std::cerr << "Usage: " << argv[0] << " <trial_name> <synthetic_trial_name or 'none'> <'uwb' or 'no_uwb'> <uwb_noise> <dump (true|false)>" << std::endl;
        return 1;
    }
	std::string trial_name = argv[1];
    std::string synthetic_trial_name = argv[2];
	std::string uwb_str = argv[3];
	std::string uwb_noise_str = argv[4];
	std::string dump_str = argv[5];

    bool log_dump = (dump_str == "true");
	bool use_uwb = (uwb_str == "uwb");
	bool use_gt = true;
	bool synthetic = synthetic_trial_name != "none";
	double uwb_synth_stdev = stod(uwb_noise_str);


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
	ofstream estimtated_timestamp_fs(out_dir+"/est_timestamps.txt");
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

	double dt = 1.0 / 200.0; // IMU gyro and accelerometer operate at 200Hz

	// --- Noise Models ---

	// VIO noise model

	double vio_ori_stdev = 0.175;
	double vio_pos_stdev = 0.2;
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));

	// UWB noise model

	// double uwb_stdev = 1e-3;
	double uwb_stdev = 0.1;
	// double uwb_stdev = 0.2;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	// GT noise model - (use to define pose prior)
	double gt_pos_stdev = 1e-2;
	double gt_ori_stdev = 1e-2;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));
	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 1e-2);
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-2);


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

	imu_preintegration_params->body_P_sensor = T_imu_body;

	const string id = "1";
	Tracker t(
		id, T_imu_body, dt,
		GT_noise_model, UWB_noise_model, VIO_pose_noise_model, 
		prior_velocity_noise_model, prior_bias_noise_model,
		imu_preintegration_params, prior_imu_bias,
		debug_dir);

	t.init_anchors(json::parse(beacon_fs));
	// t.init(sensor_stream);


	int imu_counter = 0;
	int last_imu_counter = 0;

	bool start_graph = false;

	int uwb_counter = 0;
	int gt_counter = 0;

	int imu_count_at_last_correction = 0;
	int imu_count_at_last_imu_factor = 0;
	int factor_counter = 0;
	int gt_skipped = 0;

	vector<json> gt_pose_buffer;
	vector<json> range_buffer;

	int mes_idx = 0;

	for (json mes : sensor_stream) {

		if (start_graph && mes["type"] == "imu") {

			// Add IMU measurement
			Vector3 accel;
			Vector3 gyro;
			get_IMU(mes, accel, gyro);
			t.imu_preintegrated->integrateMeasurement(accel, gyro, dt);
			imu_counter++;

			cout << "Preintegration on at " << mes["t"] << " a: " << accel.x() << " " << accel.y() << " " << accel.z() << ", g: " << gyro.x() << " " << gyro.y() << " " << gyro.z() << endl;

			// Just for plotting at IMU frequency
			PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(t.imu_preintegrated);
			auto proposed = current_imu_preintegration->predict(t.prev_state, t.track.constant_bias);
			t.track.est_poses.push_back(proposed.pose());
			t.track.est_timestamps.push_back((double)mes["t"]);

			for (json mes : gt_pose_buffer) {
				t.processSLAM(mes);
				imu_count_at_last_correction = imu_counter;
				imu_count_at_last_imu_factor = imu_counter;
				gt_counter++;
			}
			gt_pose_buffer.clear();

			for (json mes : range_buffer) {
				if (synthetic) {
					t.processSUWB(mes, uwb_counter, uwb_stdev);
					uwb_counter++; // TODO check to make sure I'm not double counting.
				}
				else {
					//t.processUWB
				}

			}
			range_buffer.clear();
		}
		else if (use_gt && mes["type"] == "slam_pose" && !start_graph) {
			// Skip all measurements until we find a slam pose and velocity that we can use
			// to set up priors
			t.init_state(mes);
			start_graph = true;
			
		}
		else if (use_gt && mes["type"] == "slam_pose" && start_graph) {

			if (imu_counter == imu_count_at_last_imu_factor) {
				// Pass this measurement and buffer it until the next IMU becomes available
				cout << " Skipped SLAM pose " << endl;
				gt_skipped ++; 
				gt_pose_buffer.push_back(mes);
				continue;
			}
			t.processSLAM(mes);
			imu_count_at_last_correction = imu_counter;
			imu_count_at_last_imu_factor = imu_counter;
			gt_counter++;
			
		}
		else if (synthetic && use_uwb && mes["type"] == "synthetic_uwb" && start_graph) {
			if (imu_counter == imu_count_at_last_correction) { continue; }
			else {
				t.processSUWB(mes, uwb_counter, uwb_stdev);
				uwb_counter++;
			}
			imu_count_at_last_imu_factor = imu_counter;
			imu_count_at_last_imu_factor = imu_counter;
		}
		// else if (use_uwb && mes["type"] == "uwb" && start_graph) {

		// }
		mes_idx ++;
	}

	write_trajectory_TUM_format( t.track.est_poses, t.track.est_timestamps, estimated_trajectory_fs, T_imu_body);
	estimated_trajectory_fs.close();

	write_trajectory_TUM_format( t.track.gt_poses, t.track.gt_timestamps, slam_trajectory_fs, T_imu_body);
	slam_trajectory_fs.close();

	write_timestamps( t.track.est_poses, t.track.est_timestamps, estimtated_timestamp_fs);
	estimtated_timestamp_fs.close();


	cout << "Dumping magnetometer vectors for visual debug" << endl;
	ofstream suwb_base_poses_fs(out_dir + "/suwb_base_poses.txt");
	write_trajectory_KITTI_format( t.suwb_base_poses, suwb_base_poses_fs);
	suwb_base_poses_fs.close();

	ofstream mag_vectors_fs(out_dir + "/mag_vectors_fs.txt");
	write_trajectory_KITTI_format( t.mag_vectors, mag_vectors_fs);
	mag_vectors_fs.close();




	cout << " Applied " << uwb_counter << " uwb measurements for 45 seconds of data " << endl;
	double fuwb = uwb_counter /45.0;
	cout << " UWB frequency in the graph is " << fuwb << endl;

	cout << " Applied " << gt_counter << " slam measurements for 45 seconds of data " << endl;
	double fgt = gt_counter /45.0;
	cout << " GT frequency in the graph is " << fgt << endl;
	cout << " GT skipped " << gt_skipped << endl;


	return 0;
}
