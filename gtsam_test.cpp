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
    bool log_dump = (dump_str == "true"); // We ignore this and dump anyways
	bool use_uwb = (uwb_str == "uwb");
	bool use_gt = true;
	bool synthetic = synthetic_trial_name != "none";


	vector<int> users {2,3,4};
	map<int, Tracker> trackers;
	map<int, Tracker>& trackers_ref = trackers;

	string data_dir = "/home/antond2/Desktop/Research/MultiXR-Post/merged/"+trial_name+"_merged/";
	ifstream raw_fs(data_dir + "/all.json");
	ifstream anchor_fs(data_dir + "/anchors.json");
	json sensor_stream = json::parse(raw_fs); // Sensor stream contains all measurements.
	json anchor_stream = json::parse(anchor_fs);

	for (int user: users) {

		string out_dir = "/home/antond2/Desktop/Research/gtsam_test/results/out/multi/"+to_string(user)+"/"+trial_name;
		string debug_dir = out_dir+"/debug";
		vector<string> paths = {out_dir, debug_dir};
		for (string path: paths) {
			if (!std::filesystem::exists(path)) {
					std::filesystem::create_directories(path);
					std::cout << "Directory created: " << path << std::endl;
			}
		}
		cout << "In path " << data_dir << endl;
		cout << "Out path " << out_dir << endl;


		ifstream transform_fs(data_dir + "/transforms"+to_string(user)+".json"); // possibly unique transforms per user
	
		json transforms = json::parse(transform_fs);

		// UWB noise model
		double uwb_stdev = 0.1;
		noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev);
		// SLAM noise model - (use to define pose prior)
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

		const int smoother_lag = 1;
		const bool use_smoother = false;
		const bool use_filter = false;


		Tracker t(
			to_string(user), 
			trackers_ref,
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
			out_dir,
			debug_dir);

		t.init_anchors(anchor_stream);
		t.init_state(sensor_stream); 
		trackers.emplace(user, std::move(t)); 
	}

	// Main emulation loop!
	for (json mes : sensor_stream) {
		try{
			Tracker& t = trackers.at((int)mes["src"]);
			t.processSensor(mes);
		}
		catch (const std::exception& e) {
			// Sometimes ranges may have "src" as 5 or 1 when I'm mirroring UWB ranges
			// Just skip those, they aren't flock nodes
			continue;
		}
	}

	for (auto& [user, t]: trackers) {
		// Dump all tracker trajectories

		write_trajectory_TUM_format( t.track.est_poses, t.track.est_timestamps, t.estimated_trajectory_fs);
		t.estimated_trajectory_fs.close();

		// Write estimated poses to a json so they can be plotted
		write_trajectory_HTM_JSON_format (t.track.est_poses, t.track.est_timestamps, t.estimated_trajectory_htm_json_fs, "est_pose");
		t.estimated_trajectory_htm_json_fs.close();
	}

	

	return 0;
}
