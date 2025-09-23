#include "gtsam_test.h"
#include "data_tools.h"
#include "cmath"
#include "tracker.h"
#include <regex>
#include "/home/antond2/gnuplot-iostream/gnuplot-iostream.h"
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>


using PreintegrationType = gtsam::PreintegrationBase;
using namespace gtsam;
using namespace std;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)


int main(int argc, char* argv[]) {

	if (argc != 6) {
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
	bool use_synthetic_uwb = uwb_synth_stdev > 1e-5;

	string DATA_INPUT_DIR = "/home/antond2/ws/post/out/";
	string GRAPH_REPO_DIR = "/home/antond2/Desktop/Research/";

	string data_dir = DATA_INPUT_DIR+trial_name+"_post";
	string out_dir = GRAPH_REPO_DIR+"gtsam_test/results/out/"+trial_name;

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

	ifstream beacon_fs(DATA_INPUT_DIR+trial_name+"_post" + "/anchors.json");
	ifstream transform_fs(DATA_INPUT_DIR+trial_name+"_post" + "/transforms.json");

	ofstream estimated_trajectory_fs(out_dir + "/est.txt");
	ofstream slam_trajectory_fs(out_dir+"/slam.txt");
	ofstream estimated_timestamp_fs(out_dir+"/est_timestamps.txt");
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
		std::cout.rdbuf(log_dump_fs.rdbuf());
	}

	cout << "In path " << data_dir << endl;
	cout << "Out path " << out_dir << endl;


	json sensor_stream = json::parse(raw_fs);
	json transforms = json::parse(transform_fs);
	map<string, tracking> info; // Map of username to tracking information

	double dt = 1.0 / 200.0; // IMU gyro and accelerometer operate at 200Hz

	// --- Noise Models ---

	// VIO noise model

	double vio_ori_stdev = 0.175;
	double vio_pos_stdev = 0.2;
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));

	// UWB noise model

	// double uwb_stdev = 1e-2;
	double uwb_stdev = 0.05;
	// double uwb_stdev = 0.2;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	// GT noise model - (use to define pose prior)
	double gt_pos_stdev = 1e-2;
	double gt_ori_stdev = 1e-2;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));
	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 1e-2);
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);


	//// IMU noise model

	// IMU initialization code

	std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = get_imu_preintegration_params(10, 10);
	imuBias::ConstantBias prior_imu_bias(Vector3(-0.03, -0.05, 0.23), Vector3(-0.0048, -0.00445, -0.0015)); 
	// First accelerometer bias, then gyro bias.
	
	Pose3 T_imu_to_body;
	get_pose_from_HTM(transforms["T_imu_to_body"], T_imu_to_body);

	// // Judging from this transform from earlier
	// Matrix33 transform;
	// transform << 1, 0, 0,
	// 			0, 0, 1,
	// 			0, -1, 0;
	// // we're dealing with a y-forward, x-right body frame.
	// Pose3 T_imu_to_body(Rot3(transform), Vector3::Zero());
	// Pose3 T_body_to_imu = T_imu_to_body.inverse();

	Pose3 T_body_to_imu = T_imu_to_body.inverse();

	//https://gtsam.org/doxygen/4.0.0/a03531.html
	// "Pose of the sensor in the body frame" - this should be T IMU to body.
	// but T_body_to_imu seems to give a more reasonable looking (but still incorrect) result
	imu_preintegration_params->setBodyPSensor(T_body_to_imu);




	Pose3 T_body_to_decawave;
	get_pose_from_HTM(transforms["T_body_to_decawave"], T_body_to_decawave);

	// "pose of sensor in body frame"

	const string id = "1";
	const int smoother_lag = 1;
	const bool use_smoother = false;
	const bool use_filter = false;


	Tracker t(
		id, T_body_to_imu, T_body_to_decawave, 
		dt, smoother_lag, use_smoother, use_filter, 
		uwb_synth_stdev, GT_noise_model, UWB_noise_model, VIO_pose_noise_model, 
		prior_velocity_noise_model, prior_bias_noise_model,
		imu_preintegration_params, prior_imu_bias,
		debug_dir);
	
	t.estimated_trajectory_fs = &estimated_trajectory_fs;
	t.slam_trajectory_fs = &slam_trajectory_fs;


	bool start_graph = false;

	// TODO: Counters can all be internal to tracker.
	int imu_counter=0, uwb_counter = 0, gt_counter=0,
	imu_count_at_last_correction = 0, imu_count_at_last_imu_factor = 0;

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


			cout << "Preintegration at " << mes["t"] << " a: " << accel.x() << " " << accel.y() << " " << accel.z() << ", g: " << gyro.x() << " " << gyro.y() << " " << gyro.z() << endl;

			// Just for plotting at IMU frequency
			PreintegratedCombinedMeasurements* current_imu_preintegration = dynamic_cast<PreintegratedCombinedMeasurements*>(t.imu_preintegrated);
			NavState proposed = current_imu_preintegration->predict(t.prev_state, t.track.changing_bias);

			t.report_estimate(proposed.pose(), mes["t"]);

		}

		else if (use_gt && mes["type"] == "vicon_pose" && !start_graph) {
			// Skip all measurements until we find a slam pose and velocity that we can use to set up priors
			t.init_state(mes);
			start_graph = true;
		}

		else if (use_gt && mes["type"] == "vicon_pose" && start_graph) { 
			// Doesn't contribute to the estimated trajectory at all on this branch

			Pose3 T_world_to_body;
			string usrname;

			// Notation; T_body_world = T_world_to_body (as tracked by Vicon)
			get_pose_from_HTM(mes["T_body_world"],T_world_to_body);

			Pose3 gt_pose = T_world_to_body;
			t.track.gt_poses.push_back(gt_pose);
			t.track.gt_timestamps.push_back(mes["t"]);
			gt_counter++;
		}

		mes_idx ++;
	}

	// Before writing files for evluation, need to be able to transform all
	// body poses in world frame to slambody (cam1) poses in world frame.

	Pose3 T_imu_to_cam1;
	get_pose_from_HTM(transforms["T_imu_to_cam1"], T_imu_to_cam1);

	// Pose3 T_body_to_sbody_in_world = T_body_to_imu.compose(T_imu_to_cam1);
	// Pose3& out_transform = T_body_to_sbody_in_world;
	Pose3 iden = Pose3::Identity();
	Pose3& out_transform = iden;

	write_trajectory_TUM_format( t.track.est_poses, t.track.est_timestamps, estimated_trajectory_fs, out_transform);
	estimated_trajectory_fs.close();

	write_trajectory_TUM_format( t.track.gt_poses, t.track.gt_timestamps, slam_trajectory_fs, out_transform);
	slam_trajectory_fs.close();

	write_timestamps( t.track.est_poses, t.track.est_timestamps, estimated_timestamp_fs);
	estimated_timestamp_fs.close();


	cout << "Dumping magnetometer and velocity vectors for visual debug" << endl;
	
	ofstream suwb_base_poses_fs(out_dir + "/suwb_base_poses.txt");
	write_trajectory_KITTI_format( t.suwb_base_poses, suwb_base_poses_fs);
	suwb_base_poses_fs.close();

	ofstream gt_base_poses_fs(out_dir + "/gt_base_poses.txt");
	write_trajectory_KITTI_format( t.track.gt_poses, gt_base_poses_fs);
	gt_base_poses_fs.close();

	ofstream mag_vectors_fs(out_dir + "/mag_vectors_fs.txt");
	write_trajectory_KITTI_format( t.mag_vectors, mag_vectors_fs);
	mag_vectors_fs.close();

	ofstream postproc_velocity_fs(out_dir + "/vel_vectors.txt");
	write_trajectory_KITTI_format( t.postproc_velocity_vectors, postproc_velocity_fs);
	postproc_velocity_fs.close();


	return 0;
}
