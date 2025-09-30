#include "gtsam_test.h"
#include "data_tools.h"
#include "utils.h"
#include "cmath"
#include "tracker.h"
#include "central_tracker.h"
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

	ifstream beacon_fs(data_dir + "/anchors.json");
	ifstream transform_fs(data_dir + "/transforms.json");

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
	json transform_json = json::parse(transform_fs);
	json anchor_json = json::parse(beacon_fs);

	map<string, tracking> info; // Map of username to tracking information


	const string id = "1";
	const int smoother_lag = 1;
	const bool use_smoother = false;
	const bool use_filter = true;


	CentralTracker c(use_smoother, data_dir, out_dir, uwb_synth_stdev);

	bool start_graph = false;

	int mes_idx = 0;

	for (json mes : sensor_stream) {

		string id = mes["src"];
		Tracker& t = c.users.at(id);

		if (start_graph && mes["type"] == "imu") {
			t.processIMU(mes);
		}
		else if (use_gt && mes["type"] == "slam_pose" && !start_graph) {
			// Skip all measurements until we find a slam pose and velocity that we can use to set up priors
			t.init_state(mes);
			start_graph = true;
		}
		else if (use_gt && mes["type"] == "slam_pose" && start_graph) {
			t.processSLAM(mes);
		}
		else if (!use_synthetic_uwb && use_uwb && mes["type"] == "assisted_uwb" && start_graph) {
			// For the pilot4 case, where we aren't generating synthetic ranges, 
			// but still need synthetic orientations from post processed interpolation
			t.processAssistedUWB(mes);

		}

		mes_idx ++;
	}

	return 0;
}
