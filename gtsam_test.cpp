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

	string data_dir = "/home/antond2/Desktop/Research/MultiXR-Post/2/post/"+trial_name+"_post";
	string out_dir = "/home/antond2/Desktop/Research/gtsam_test/results/out/"+trial_name;
	string debug_dir = out_dir+"/debug";

	ifstream raw_fs(data_dir + "/all.json");
	ifstream anchor_fs(data_dir + "/anchors.json");
	ifstream transform_fs(data_dir + "/transforms.json");

	ofstream estimated_trajectory_htm_json_fs(out_dir + "/est_"+uwb_str+".json");
	ofstream estimated_trajectory_fs(out_dir + "/est_"+uwb_str+".txt");
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

	// --- Noise Models ---

	// TODO: Move within tracker

	// UWB noise model

	double uwb_stdev = 0.2;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);

	// GT noise model - (use to define pose prior)
	double gt_pos_stdev = 1e-2;
	double gt_ori_stdev = 1e-2;
	noiseModel::Diagonal::shared_ptr SLAM_noise_model = noiseModel::Diagonal::Sigmas(Vector6(gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));

	//// IMU noise model

	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 1e-2);
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-2);

	Vector3 prior_velocity(0,0,0);
	Vector6 prior_imu_bias(0,0,0, 0, 0, 0);

	Pose3 T_inertial_to_world;
	get_pose_from_HTM(transforms["T_inertial_to_world"], T_inertial_to_world);
	std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = get_imu_preintegration_params(10, 1, T_inertial_to_world);

	// Other transforms
	
	Pose3 T_body_to_imu;
	get_pose_from_HTM(transforms["T_body_to_imu"], T_body_to_imu);

	Pose3 T_body_to_decawave;
	get_pose_from_HTM(transforms["T_body_to_decawave"], T_body_to_decawave);

	imu_preintegration_params->setBodyPSensor(T_body_to_imu);

	const string id = "2";
	const int smoother_lag = 1;
	const bool use_smoother = false;
	const bool use_filter = false;

	Tracker t(
		id, 
		T_body_to_imu, 
		T_body_to_decawave, 
		smoother_lag, 
		use_smoother, 
		use_filter,
		use_uwb, 
		SLAM_noise_model, 
		UWB_noise_model, 
		prior_velocity_noise_model,
		prior_bias_noise_model,
		imu_preintegration_params,
		prior_imu_bias,
		prior_velocity,
		debug_dir);
	
	t.estimated_trajectory_fs = &estimated_trajectory_fs;
	t.slam_trajectory_fs = &slam_trajectory_fs;

	t.init_anchors(json::parse(anchor_fs));
	t.init_state(sensor_stream); 
	// Initialize with no calibration phase and no priors

	// Main emulation loop!
	for (json mes : sensor_stream) {
		t.processSensor(mes);
	}

	// Before writing files for evluation, need to be able to transform all
	// body poses in world frame to slambody (cam1) poses in world frame.

	Pose3 T_imu_to_cam1;
	get_pose_from_HTM(transforms["T_imu_to_cam1"], T_imu_to_cam1);

	Pose3 T_body_to_sbody_in_world = T_body_to_imu.compose(T_imu_to_cam1);
	Pose3& out_transform = T_body_to_sbody_in_world;

	write_trajectory_TUM_format( t.track.est_poses, t.track.est_timestamps, estimated_trajectory_fs);
	estimated_trajectory_fs.close();

	write_trajectory_TUM_format( t.track.slam_poses, t.track.slam_timestamps, slam_trajectory_fs);
	slam_trajectory_fs.close();

	write_timestamps( t.track.est_poses, t.track.est_timestamps, estimated_timestamp_fs);
	estimated_timestamp_fs.close();

	// Write estimated poses to a json so they can be plotted
	write_trajectory_HTM_JSON_format (t.track.est_poses, t.track.est_timestamps, estimated_trajectory_htm_json_fs, "est_pose");
	estimated_trajectory_htm_json_fs.close();

	ofstream gt_base_poses_fs(out_dir + "/gt_base_poses.txt");
	write_trajectory_KITTI_format( t.track.slam_poses, gt_base_poses_fs);
	gt_base_poses_fs.close();
	

	return 0;
}
