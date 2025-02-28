#include "gtsam_test.h"
#include "data_tools.h"
#include "utils.h"

#include "cmath"
#include <regex>

using namespace gtsam;
using namespace std;

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





int run_cappella() {

	//string filename = "bigtest-1floor";
	string filename = "los-1floor";
	string directory = "/home/admitriev/Datasets/cappella_data/set_1/";

	ifstream raw_fs(directory + filename + "_universal_frame.json");
	ifstream gt_fs(directory + filename + "_gt_reconstructed_sorted.json");
	ifstream beacon_fs(directory + filename + "_beacons.json");


	json sensor_stream = json::parse(raw_fs); 
	map<string, tracking> info;


	get_gt_info(info, json::parse(gt_fs)); // fill user_info with gt_pose trajectory
	get_beacon_info(info, json::parse(beacon_fs));


	// --- Noise Models ---

	// VIO noise model

	double vio_ori_stdev = 0.175; // rad->~10degrees
	double vio_pos_stdev = 0.2;
	//double vio_ori_stdev = 0.05; // rad->~10degrees
	//double vio_pos_stdev = 0.05;
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
			// Could it be that my static can't handle single digit numbers, i.e. static9
			// Nope it can handle 9, 10 fine.
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

	int pose_num = 0;
	Values vals; // a, e, f (jeff), j, n, s (static)
	for (auto& [u, track] : info) {

		track.I = 0;

		if (track.is_beacon) {

			track.pose_key = MK(u, track.I);
			Pose3 prior_beacon_pose(track.gt_poses[0]); // Position of beacon in U frame extracted from GT
			vals.insert(track.pose_key, prior_beacon_pose);
			graph->add(NonlinearEquality<Pose3>(track.pose_key, prior_beacon_pose)); // Pose or point?
		
		}

	}

	//ISAM2Params isam_params;
	//isam_params.factorization = ISAM2Params::CHOLESKY;
	//isam_params.relinearizeSkip = 10;
	//ISAM2DoglegParams dogleg;
	//isam_params.optimizationParams = dogleg;
	//ISAM2* isam = new ISAM2(isam_params);

	int max_VIO_measurements = 60;
	int VIO_measurements = 0;

	double uwb_error = 0;
	double n_uwb_mes = 0;

	int VIO_show = 1000;
	for (json mes : sensor_stream) {

		chrono::system_clock::time_point tp = iso_string_to_time(mes["timestamp"]);
		unsigned long timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();

		if (mes["type"] == "vio") {
			Matrix44 M_L_U;
			string user;
			get_pose_matrix(mes, user, M_L_U);

			tracking& track = info.at(user);

			if (track.I == 0) {
				// If this is the first VIO pose, prior that thang

				Pose3 prior_VIO_pose(M_L_U); // Assume VIO starts at same place as GT
				track.vio_poses.push_back(prior_VIO_pose);


				track.pose_key = MK(user, track.I);
				vals.insert(track.pose_key, prior_VIO_pose);
				graph->addPrior(track.pose_key, prior_VIO_pose, VIO_pose_noise_model);

			}
			
			if (track.I >= 0) {

				track.I++;

				Pose3 pose(M_L_U);
				Pose3 last_pose = track.vio_poses.back();
				track.vio_poses.push_back(pose);

				vals.insert(MK(user, track.I), pose); // vio pose gets bound as the initial estimate to this key.
				cout << " Added key to values " << user << track.I << endl;

				Pose3 odometry = last_pose.between(pose);
				graph->add(BetweenFactor<Pose3>(MK(user, track.I - 1), MK(user, track.I), odometry, VIO_pose_noise_model));
				cout << " Added factor with keys " << user << track.I-1 << " -> " << user << track.I << endl;
				
			}

			//cout << " Added key " << user << " #" << track.I << endl;

			VIO_measurements++;

		}
		else if (mes["type"] == "uwb") {
			double range;
			string src_user, dst_user;
			get_UWB(mes, src_user, dst_user, range);

			Pose3 src_pose = info[src_user].gt_poses[info[src_user].I];
			Pose3 dst_pose = info[dst_user].gt_poses[info[dst_user].I];
			double true_range = distance3(src_pose.translation(), dst_pose.translation());
			graph->add(RangeFactor<Pose3, Pose3, double>(MK(src_user, info[src_user].I), MK(dst_user, info[dst_user].I), true_range, UWB_noise_model));

			//graph->add(RangeFactor<Pose3, Pose3, double>(MK(src_user, info[src_user].I), MK(dst_user, info[dst_user].I), range, UWB_noise_model));

			uwb_error += abs(true_range - range);
			n_uwb_mes++;

		}
		else if (mes["type"] == "gt") {

		}

		if (VIO_measurements > max_VIO_measurements) break; // TO keep the graph small and visualizable

		//isam->update(*graph, vals);
		//graph->resize(0); // According to example
		//vals.clear(); // Still don't quite get why we need this vals.clear();

	}

	// We increment track.I one last time before leaving the loop? But GTSAM doesn't use that variable
	// If you have a range at exactly the cutoff, you set a factor between a13 and a14, and then you increment a to be at 15
	// then e range to a, and e15 tries to connect to a15 (because thats what info listed) but the actual last value added is a14

	double avg_uwb_error = uwb_error / n_uwb_mes;
	cout << " Average dataset UWB error (m) " << avg_uwb_error << endl;

	// To not get key out of bounds lol
	// This garbage may be the culprit
	// 

	//for (auto& [u, track] : info) {
	//	if (!track.is_beacon) vals.insert(MK(u, track.I), track.vio_poses.back());
	//}
	// Users have uneven number of VIO poses

	//LM_lambda_search(graph, vals, info);

	vector<string> show_plots_for = { "nuno", "elahe"};

	//unpack_results(isam->calculateBestEstimate(), MK, info);
	//cout << info["elahe"].gt_poses.size() << " " << info["elahe"].vio_poses.size() << " " << info["elahe"].est_poses.size() << endl;
	//PLOT_FOR_USERS(info, show_plots_for);
	//show();

	//cout << vals.at<Pose3>(MK("jeff", 0)) << endl; // Ok so Jeff is NOT in values...

	//vals.print();


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

		unpack_results(optimizer.values(), MK, info);
		PLOT_FOR_USERS(info, show_plots_for);
		clear_results(info); // clear Est_poses trajectory

	} while (!checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol, last_error, optimizer.error()));

	show();

	cout << " Converged in " << optimizer.iterations() << " iterations, with " << optimizer.error() << " final error." << endl; // Currently doing 4 iterations


	//graph->print();

	//show();


	return 0;
}

int main(int argc, char* argv[]) {

	// run_euroc();
	run_cappella();

	return 0;
}
