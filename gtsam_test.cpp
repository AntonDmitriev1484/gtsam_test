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
                draw_points(user_info.gt_poses, "g");                      \
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
	string filename = "/home/admitriev/Datasets/cappella_data/set_1/bigtest-1floor_sorted.json";
	string output_filename = "/home/admitriev/Datasets/cappella_data/set_1/bigtest-1floor_gt_reconstructed.json";
	ifstream fs(filename);

	json sensor_stream = json::parse(fs);
	map<string, tracking_info> info;
	get_info(sensor_stream, info);

	// --- Noise Models ---

	// VIO Prior Noise Model

	//double os = 0.275;
	//double ps = 0.5; // Am I sure its orientation first, and not position? IT IS POSITION FIRST: SOURCE: https://github.com/haidai/gtsam/blob/master/examples/VisualISAMExample.cpp
	//noiseModel::Diagonal::shared_ptr a = noiseModel::Diagonal::Sigmas(Vector6(ps, ps, ps, os, os, os));

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

			Matrix44 prior_pose_matrix(userinfo.last_HTM_G_U * userinfo.first_HTM_L_G);

			Pose3 prior_VIO_pose(prior_pose_matrix);
			userinfo.vio_poses.push_back(prior_VIO_pose);
			vals.insert(userinfo.pose_key, prior_VIO_pose);
			graph->addPrior(userinfo.pose_key, prior_VIO_pose, VIO_pose_noise_model);
		}

	}


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
			Matrix44 pose_matrix_U =  u.last_HTM_G_U * HTM_L_G;

			Pose3 pose(pose_matrix_U);
			Pose3 last_pose = u.vio_poses.back(); // segfault
			u.vio_poses.push_back(pose);

			Pose3 odometry = last_pose.between(pose);
			graph->add(BetweenFactor<Pose3>(MK(user, u.I - 1), MK(user, u.I), odometry, VIO_pose_noise_model));

			vals.insert(MK(user, u.I), pose); // vio pose gets bound as the initial estimate to this key.

			string assoc_timestamp = mes["timestamp"];
			u.est_poses_iso_timestamp.push_back(assoc_timestamp); // Each key in vals, is associated with a timestamp, which will be associated with a refined pose estimate.

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
			vector<Matrix44> HTM_L_U_per_user;
			vector<string> users;
			get_GT(mes, users, HTM_L_U_per_user);

			for (int i = 0; i < users.size(); i++) {
				tracking_info& u = info.at(users[i]);

				Matrix44 pose_matrix_U = HTM_L_U_per_user[i];

				Pose3 GT_pose(pose_matrix_U);
				u.gt_poses.push_back(GT_pose);

				graph->add(PriorFactor<Pose3>(MK(users[i], u.I), GT_pose, GT_noise_model));
			}

		}


	}



	vector<string> show_plots_for = { "elahe", "nuno" };

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

	// NOTE: Comment this out if you're going to write the data. Keep it in if you want to visualize each optimization step.
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


	// NOTE: Comment this in when you're ready to write the data
	optimizer.optimize();
	
	//GraphvizFormatting vizp;
	//vizp.plotFactorPoints = true;
	////vizp.mergeSimilarFactors = true;
	//vizp.binaryEdges = true;

	//graph->saveGraph("/home/admitriev/Research/gtsam_test/factor_graphs/factor_graph.dot", optimizer.values(), vizp);
	//graph->print();

	//show();


	//unpack_results(optimizer.values(), MK, info);
	//dump_reconstructed_trajectories(info, output_filename);


	return 0;
}

int main(int argc, char* argv[]) {

	// run_euroc();
	run_cappella();

	return 0;
}
