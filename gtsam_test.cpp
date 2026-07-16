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

	if (argc != 7) {
        std::cerr << "Usage: " << argv[0] 
		<< " <trial_name> <synthetic_trial_name or 'none'> <'uwb' or 'no_uwb'> <uwb_noise> <dump (true|false)>" 
		<< " <anchor_loc>" << std::endl;
        return 1;
    }
	std::string trial_name = argv[1];
    std::string synthetic_trial_name = argv[2];
	std::string uwb_str = argv[3];
	std::string uwb_noise_str = argv[4];
	std::string dump_str = argv[5];
	string anchor_loc_strategy = argv[6]; // should be, none, self-loc, pre-loc.

    bool log_dump = (dump_str == "true"); // We ignore this and dump anyways
	bool use_uwb = (uwb_str == "uwb");
	bool synth_live_slam_mode = (uwb_str == "live-slam-integration");

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
		noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, 1e-1);
		noiseModel::Isotropic::shared_ptr selfloc_UWB_noise_model = noiseModel::Isotropic::Sigma(1, 4e-1);
		// Anchor noise model - anchor pose prior when anchor_loc_strategy == self-loc
		double anchor_pos_stdev = 1e-1;
		double anchor_ori_stdev = 1e-1;
		noiseModel::Diagonal::shared_ptr selfloc_Anchor_noise_model = noiseModel::Diagonal::Sigmas(
			Vector6(1e-5, 1e-5, 1e-5, 
				anchor_pos_stdev, anchor_pos_stdev, 1e-5));
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
		const bool use_smoother = true;
		const bool use_filter = !(synth_live_slam_mode); 
		// const bool use_filter = true;
		// don't use filter when we're synthesizing a live slam by running integration


		Tracker t(
			to_string(user), 
			trackers_ref,
			T_body_to_imu, 
			T_body_to_decawave, 
			smoother_lag, 
			use_smoother, 
			use_filter,
			use_uwb,
			synth_live_slam_mode,
			anchor_loc_strategy,
			SLAM_noise_model, 
			UWB_noise_model, 
			prior_velocity_noise_model,
			prior_bias_noise_model,
			selfloc_UWB_noise_model,
			selfloc_Anchor_noise_model,
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
		int src = (int)mes["src"];
		if (src != 1 && src != 5) {
			for (auto& [user, t]: trackers) {
				t.processSensor(mes);
				// Let every tracker have a chance to process the measurement
				// Sometimes tracker 2 might need a range from user 4->1 in order to self localize anchor 1
			}
		}
	}

	for (auto& [user, t]: trackers) {
		// Dump all tracker trajectories

		if (synth_live_slam_mode) {
			ofstream result_trajectory_fs("/home/antond2/Desktop/Research/gtsam_test/results/out/multi/"+to_string(user)+"/"+trial_name+"/aligned_live_slam.txt");
			ofstream result_trajectory_htm_json_fs("/home/antond2/Desktop/Research/gtsam_test/results/out/multi/"+to_string(user)+"/"+trial_name+"/aligned_live_slam.json");
			
			// Write estimated poses to TUM so we can use in evaluation.
			write_trajectory_TUM_format( t.track.slam_poses, t.track.slam_timestamps, result_trajectory_fs);
			result_trajectory_fs.close();

			// Write estimated poses to a json so they can be plotted
			write_trajectory_HTM_JSON_format (t.track.slam_poses, t.track.slam_timestamps, result_trajectory_htm_json_fs, 
				"aligned_live_slam_pose", t.start, t.init_newmap, t.end);
			result_trajectory_htm_json_fs.close();
		}
		else {
			// Write estimated trajectory as TUM for EVO
			write_trajectory_TUM_format( t.track.est_poses, t.track.est_timestamps, t.estimated_trajectory_fs);
			t.estimated_trajectory_fs.close();

			// Write estimated poses to a json so they can be plotted with plot_all
			write_trajectory_HTM_JSON_format (t.track.est_poses, t.track.est_timestamps, t.estimated_trajectory_htm_json_fs, 
				"est_pose", t.start, t.init_newmap, t.end);
			t.estimated_trajectory_htm_json_fs.close();

			// Write estimated anchor trajectories as TUM

			// Dump the full trajectory for each anchor
			for (auto& [id, anchor_track]: t.anchors){

				// This can be used for plotting with plot all
				ofstream anchor_optimization_trajectory_htm_json_fs(t.out_dir + "/anchor_"+id+"_optimization.json");
				write_trajectory_HTM_JSON_format(anchor_track.est_poses, anchor_track.est_timestamps, anchor_optimization_trajectory_htm_json_fs,
					"est_pose", 0, 0, 0);
				anchor_optimization_trajectory_htm_json_fs.close();

			}

			// Output anchor translation in world frame
			ofstream anchor_positions_fs(t.out_dir + "/anchors_estimate.json");
			write_anchor_positions(t.anchors, anchor_positions_fs);
			anchor_positions_fs.close();

		}

	}

	

	return 0;
}
