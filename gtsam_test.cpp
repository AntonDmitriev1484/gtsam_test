#include "gtsam_test.h"
#include "data_tools.h"
#include "utils.h"

#include "cmath"
#include <regex>

using namespace gtsam;
using namespace std;

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


int run_cappella() {
	string raw_filename = "/home/admitriev/Datasets/cappella_data/set_1/bigtest-1floor_sorted.json";
	string gt_reconstructed_filename = "/home/admitriev/Datasets/cappella_data/set_1/bigtest-1floor_gt_reconstructed_sorted.json";

	ifstream raw_fs(raw_filename);
	ifstream gt_fs(gt_reconstructed_filename);

	json sensor_stream = json::parse(raw_fs); 
	// I think this reads the entire filestream in at once, and stores it in memory
	// So iterating over sensor_stream partially, then iterating again, the second iteration should start back at 0.
	// So json::parse uses up the entire iterator, and reads everything into a program memory array ezpz
	map<string, tracking_info> info;
	get_info(sensor_stream, info);
	// Isn't sensor stream already partially read? Wouldn't this cause problems with running it in the main iterator?
	get_gt_info(info, json::parse(gt_fs)); // fill user_info with gt_pose trajectory
	//get_info2(sensor_stream, json::parse(gt_fs), info);

	// FOR THE UWB RANGES, make sure to double check that distance is preserved after I transform to the universal frame...
	// I'm 99% sure it is...


	// Data is collected with Y as the up-axis, adjust data for Z to be on the up-axis

	// --- Noise Models ---

	// VIO noise model

	double vio_ori_stdev = 0.175; // rad->~10degrees
	double vio_pos_stdev = 0.2;
	noiseModel::Diagonal::shared_ptr VIO_pose_noise_model = noiseModel::Diagonal::Sigmas(Vector6(vio_pos_stdev, vio_pos_stdev, vio_pos_stdev, vio_ori_stdev, vio_ori_stdev, vio_ori_stdev));

	// UWB noise model

	double uwb_stdev = 0.1;
	noiseModel::Isotropic::shared_ptr UWB_noise_model = noiseModel::Isotropic::Sigma(1, uwb_stdev); // Apparently this is the correct noise model for a range

	// GT noise model

	double gt_pos_stdev = 0.01;
	double gt_ori_stdev = 0.0174533;
	noiseModel::Diagonal::shared_ptr GT_noise_model = noiseModel::Diagonal::Sigmas(Vector6( gt_pos_stdev, gt_pos_stdev, gt_pos_stdev, gt_ori_stdev, gt_ori_stdev, gt_ori_stdev));

	NonlinearFactorGraph* graph = new NonlinearFactorGraph();

	// Make Key
	const function<Key(string,int)> MK = [](string username, int I) {
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

	Values vals; // a, e, f (jeff), j, n, s (static)
	for (auto& [username, userinfo] : info) {
		if (userinfo.is_beacon) {
			userinfo.pose_key = MK(username, userinfo.I);

			Pose3 prior_beacon_pose(userinfo.last_HTM_L_U); // Position of beacon in U frame extracted from GT

			vals.insert(userinfo.pose_key, prior_beacon_pose);
			graph->addPrior(userinfo.pose_key, prior_beacon_pose, GT_noise_model);
		}
		else {
			userinfo.pose_key = MK(username, userinfo.I);

			Pose3 prior_VIO_pose(userinfo.gt_poses[0]);

			// You would think that in addition to setting a prior on the first gt_pose
			// You would also have to set the M_G_U of each user to be their first GT pose.
			// Because that gives them a proper starting point
			// Whereas by default get_info is still seeking out the drifted starting point.
			// I really need to re-vise my parsing code, it's atrocious

			// Really thing I should be setting this here:

			userinfo.last_HTM_L_G = Matrix44::Identity();
			userinfo.last_HTM_G_U = userinfo.gt_poses[0].matrix();
			userinfo.last_HTM_L_U = userinfo.gt_poses[0].matrix();
			// Adding this seems like its applying a double rotation.
			// Could it be because of the filstream wrapped in json::parse()
			// 
			//maybe just add a separate field ... instead of last_HTM_G_U change verything to be around M_G_U.
			// M_G_U should be our first GT pose. im sure of it

			userinfo.vio_poses.push_back(prior_VIO_pose);
			vals.insert(userinfo.pose_key, prior_VIO_pose);
			graph->addPrior(userinfo.pose_key, prior_VIO_pose, VIO_pose_noise_model);
		}

	}

	//ISAM2Params isam_params;
	//isam_params.factorization = ISAM2Params::CHOLESKY;
	//isam_params.relinearizeSkip = 10;
	//ISAM2DoglegParams dogleg;
	//isam_params.optimizationParams = dogleg;
	//ISAM2* isam = new ISAM2(isam_params);

	int max_VIO_measurements = 1000*5;
	int VIO_measurements = 0;

	int VIO_show = 1000;
	for (json mes : sensor_stream) {

		chrono::system_clock::time_point tp = iso_string_to_time(mes["timestamp"]);
		unsigned long timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();

		if (mes["type"] == "vio") {
			Matrix44 HTM_L_G;
			string user;
			get_pose_matrix(mes, user, HTM_L_G);

			tracking_info& u = info.at(user);
			u.I++;

			u.last_HTM_L_G = HTM_L_G;
			Matrix44 pose_matrix_U = u.last_HTM_G_U * HTM_L_G;

			Pose3 pose(pose_matrix_U);
			Pose3 last_pose = u.vio_poses.back(); // segfault
			u.vio_poses.push_back(pose);

			Pose3 odometry = last_pose.between(pose);
			graph->add(BetweenFactor<Pose3>(MK(user, u.I - 1), MK(user, u.I), odometry, VIO_pose_noise_model));

			vals.insert(MK(user, u.I), pose); // vio pose gets bound as the initial estimate to this key.

			cout << "Added Key " << user << " " << u.I << endl;

			VIO_measurements++;

		}
		else if (mes["type"] == "uwb") {
			double range;
			string src_user, dst_user;
			get_UWB(mes, src_user, dst_user, range);

			//graph->add(RangeFactor<Pose3, Pose3, double>(MK(src_user, info[src_user].I), MK(dst_user, info[dst_user].I), range, UWB_noise_model));

		}
		else if (mes["type"] == "gt") {

		}

		//isam->update(*graph, vals);
		//graph->resize(0); // According to example
		//vals.clear(); // Still don't quite get why we need this vals.clear();

	}


	vector<string> show_plots_for = { "nuno" };

	//unpack_results(isam->calculateBestEstimate(), MK, info);
	//cout << info["elahe"].gt_poses.size() << " " << info["elahe"].vio_poses.size() << " " << info["elahe"].est_poses.size() << endl;
	//PLOT_FOR_USERS(info, show_plots_for);
	//show();



	// Once graph is complete, optimize it offline

	////ConjugateGradientParameters params;
	////NonlinearConjugateGradientOptimizer optimizer(*graph, vals);
	////optimizer.optimize(); // Can't iterate over this one w/ the same code
	////unpack_results_and_plot(optimizer.values(), MK, info, show_plots_for);

	LevenbergMarquardtParams params;
	LevenbergMarquardtOptimizer optimizer(*graph, vals, params);

	////GaussNewtonParams params;
	////GaussNewtonOptimizer optimizer(*graph, vals, params);

	double last_error;
	do {
		last_error = optimizer.error();
		optimizer.iterate();     

		unpack_results(optimizer.values(), MK, info);
		PLOT_FOR_USERS(info, show_plots_for);
		clear_results(info); // clear Est_poses trajectory

	} while (!checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol, last_error, optimizer.error()));

	show();

	cout << " Converged in " << optimizer.iterations() << " iterations, with " << optimizer.error() << " final error." << endl; // Currently doing 4 iterations

	
	//GraphvizFormatting vizp;
	//vizp.plotFactorPoints = true;
	////vizp.mergeSimilarFactors = true;
	//vizp.binaryEdges = true;

	//graph->saveGraph("/home/admitriev/Research/gtsam_test/factor_graphs/factor_graph.dot", optimizer.values(), vizp);
	//graph->print();

	//show();


	return 0;
}

int main(int argc, char* argv[]) {

	// run_euroc();
	run_cappella();

	return 0;
}
