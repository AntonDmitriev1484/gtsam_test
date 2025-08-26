#include "gtsam_test.h"
#include "data_tools.h"
#include "utils.h"
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

	if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <trial_name> <synthetic_trial_name or 'none'> <'uwb' or 'no_uwb'> <uwb_noise> <dump (true|false)>" << std::endl;
        return 1;
    }
	std::string trial_name = argv[1];
	std::string dump_str = argv[5];

    bool log_dump = (dump_str == "true");

	string data_dir = "/home/antond2/ws/post/out/"+trial_name+"_post";
	string out_dir = "/home/antond2/Desktop/Research/gtsam_test/out_results/"+trial_name;
	string debug_dir = out_dir+"/debug";

	ifstream raw_fs;
	raw_fs = ifstream(data_dir + "/all.json");


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
	map<string, tracking> info; // Map of username to tracking information

	double dt = 1.0 / 200.0; // IMU gyro and accelerometer operate at 200Hz

	// --- Noise Models ---

	// VIO noise model

	double vio_ori_stdev = 0.175;
	double vio_pos_stdev = 0.2;
	noiseModel::Diagonal::shared_ptr DeltaGT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));

	// UWB noise model

	// double uwb_stdev = 1e-2;
	double uwb_stdev = 0.05;
	// double uwb_stdev = 0.2;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	// GT noise model - (use to define pose prior)
	double gt_pos_stdev = 1e-3;
	double gt_ori_stdev = 1e-3;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));
	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 1e-2);
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-2);


	//// IMU noise model

	std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = get_imu_preintegration_params(1, 10);
	imuBias::ConstantBias prior_imu_bias;

	Pose3 T_body_to_imu;

	Pose3 T_body_to_decawave;

	imu_preintegration_params->setBodyPSensor(T_body_to_imu);
	

	const string id = "1";
	const int smoother_lag = 1;
	const bool use_smoother = false;
	const bool use_filter = true;

	Tracker t(
		id, T_body_to_imu, T_body_to_decawave, 
		dt, smoother_lag, use_smoother, use_filter, 
		0, GT_noise_model, UWB_noise_model, DeltaGT_noise_model, 
		prior_velocity_noise_model, prior_bias_noise_model,
		imu_preintegration_params, prior_imu_bias,
		debug_dir);
	
	t.estimated_trajectory_fs = &estimated_trajectory_fs;
	t.slam_trajectory_fs = &slam_trajectory_fs;

	// t.init_anchors(json::parse(beacon_fs));


	bool start_graph = false;

	// TODO: Counters can all be internal to tracker.
	int imu_counter=0, uwb_counter = 0, gt_counter=0,
	imu_count_at_last_correction = 0, imu_count_at_last_imu_factor = 0;

	vector<json> gt_pose_buffer;
	vector<json> range_buffer;

	int mes_idx = 0;

	for (json mes : sensor_stream) {

		if (mes["type"] == "slam_pose" && !start_graph) {
			// Skip all measurements until we find a slam pose and velocity that we can use to set up priors
			t.init_state(mes); // TODO: Modify this to only involve gt pose no IMU nonsense
			start_graph = true;
		}
		else if (mes["type"] == "slam_pose" && start_graph) {

			t.processSLAM(mes);
			imu_count_at_last_correction = imu_counter;
			imu_count_at_last_imu_factor = imu_counter;
			gt_counter++;
		}

		mes_idx ++;
	}

	//TODO Make sure to dump all LM results into est_poses

	LevenbergMarquardtParams params;
	LevenbergMarquardtOptimizer optimizer(*t.graph, t.vals, params);
	Values results = optimizer.optimize();

	// void unpack_results(Values results, const function<Key(string, int)>& MK, map<string, tracking_info>& info) {

	for (int i = 0 ; i < t.track.Ix; i++){
		Pose3 estimated_pose = results.at<Pose3>(X(i));
		t.track.est_poses.push_back(estimated_pose);
	}


	// Before writing files for evluation, need to be able to transform all
	// body poses in world frame to slambody (cam1) poses in world frame.

	Pose3 oop = Pose3::Identity();
	Pose3& out_transform = oop;

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

	

	// NOTE: THIS WILL CHANGE FOR EACH DATASET duration

	double duration_s = 45;
	cout << " Applied " << uwb_counter << " uwb measurements for "<< duration_s<< " seconds of data " << endl;
	double f_uwb = uwb_counter /duration_s;
	cout << " UWB frequency in the graph is " << f_uwb << endl;

	cout << " Applied " << gt_counter << " slam measurements for "<< duration_s<< " seconds of data " << endl;
	double f_gt = gt_counter /duration_s;
	cout << " GT frequency in the graph is " << f_gt << endl;

	return 0;
}
