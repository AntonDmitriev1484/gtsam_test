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
void LM_lambda_search(NonlinearFactorGraph* graph, Values vals, map<string, tracking_info> info) {
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
	//vector<double> attempt_lambdaInitial = { 20, 15, 10 };
	vector<double> attempt_lambdaInitial = { 10, 7, 5, 3, 2 };
	//vector<double> attempt_lambdaInitial = { 10, 1, 0.1, 0.001, 0.0001, 0.00001 };
	vector<double> attempt_lambdaFactor = { 5, 4, 3, 2, 1.5 }; // Won't run with 1

	vector<string> show_list = { "elahe" };

	for (double lambdaInitial : attempt_lambdaInitial) {

		for (double lambdaFactor : attempt_lambdaFactor) {

			LevenbergMarquardtParams lm_params;
			lm_params.diagonalDamping = false;
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
	for (auto& [username, userinfo] : info) {
		if (userinfo.is_beacon) {
			userinfo.pose_key = MK(username, userinfo.I);

			Pose3 prior_beacon_pose(userinfo.last_HTM_L_U); // Position of beacon in U frame extracted from GT

			vals.insert(userinfo.pose_key, prior_beacon_pose);
			//graph->addPrior(userinfo.pose_key, prior_beacon_pose, GT_noise_model);
			graph->add(NonlinearEquality<Pose3>(userinfo.pose_key, prior_beacon_pose)); // Pose or point?
		
			userinfo.gt_poses.push_back(Pose3(userinfo.last_HTM_L_U)); // Because apparently my get GT doesn't get any pose for beacons
			// This should suffice for giving them a pose becuase they are immobile
		}
		else {
			userinfo.pose_key = MK(username, userinfo.I);

			Pose3 gt_pose = userinfo.gt_poses[pose_num];


			//Matrix44 HTM_G_U = HTM_L_U * HTM_L_G.inverse();
			userinfo.M_G_U = userinfo.gt_poses[0].matrix() * userinfo.first_HTM_L_G.inverse();

			// Maybe need to apply the full original formula here as youhad it. i.e. first GT pose is NOT M_G_U for whatever reason
			// i.e. the one to calculate HTM_G_U from an inverse
			Pose3 prior_VIO_pose(userinfo.gt_poses[pose_num]);

			//userinfo.M_G_U = gt_pose.matrix().inverse();

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
			Matrix44 pose_matrix_U = u.M_G_U * HTM_L_G;
			//Matrix44 pose_matrix_U = u.last_HTM_G_U * HTM_L_G;

			Pose3 pose(pose_matrix_U);
			Pose3 last_pose = u.vio_poses.back();
			u.vio_poses.push_back(pose);


			vals.insert(MK(user, u.I), pose); // vio pose gets bound as the initial estimate to this key.

			Pose3 odometry = last_pose.between(pose);
			graph->add(BetweenFactor<Pose3>(MK(user, u.I - 1), MK(user, u.I), odometry, VIO_pose_noise_model));


			// NOTE: IT SEEMS TO BE READING ALL OF THE RANGES IN FIRST, BEFORE ANY ODOMETRY? WHY?
			//cout << "Added Key " << user << " " << u.I << endl;

			VIO_measurements++;

		}
		else if (mes["type"] == "uwb") {
			double range;
			string src_user, dst_user;
			get_UWB(mes, src_user, dst_user, range);

			//graph->add(RangeFactor<Pose3, Pose3, double>(MK(src_user, info[src_user].I), MK(dst_user, info[dst_user].I), range, UWB_noise_model));

			Pose3 src_pose = info[src_user].gt_poses[info[src_user].I];
			Pose3 dst_pose = info[dst_user].gt_poses[info[dst_user].I];
			//if (info[src_user].I % 1000 == 0) draw_vector(src_pose.translation(), dst_pose.translation(), "purple");

			double true_range = distance3(src_pose.translation(), dst_pose.translation());

			//cout << " True range " << true_range << " vs. data range " << range << endl;
			graph->add(RangeFactor<Pose3, Pose3, double>(MK(src_user, info[src_user].I), MK(dst_user, info[dst_user].I), true_range, UWB_noise_model));



		}
		else if (mes["type"] == "gt") {

		}

		//isam->update(*graph, vals);
		//graph->resize(0); // According to example
		//vals.clear(); // Still don't quite get why we need this vals.clear();

	}

	//LM_lambda_search(graph, vals, info);




	//vector<string> show_plots_for = { "nuno" };

	//unpack_results(isam->calculateBestEstimate(), MK, info);
	//cout << info["elahe"].gt_poses.size() << " " << info["elahe"].vio_poses.size() << " " << info["elahe"].est_poses.size() << endl;
	//PLOT_FOR_USERS(info, show_plots_for);
	//show();

	auto fig = figure();
	fig->name(" trajectory");

	for (const auto& [user_name, user_info] : info) {
		if (!user_info.is_beacon) {
			if (user_name == "nuno" || user_name == "elahe") {
	

				hold(on);
				draw_trajectory(user_info.vio_poses, "red");
				hold(on);
				draw_trajectory(user_info.gt_poses, "green");
				hold(on);
				draw_trajectory(user_info.est_poses, "blue");

				xlabel("X (m)");
				ylabel("Z (m)");
				zlabel("Y (m)");
			}
		}
	}


	// Once graph is complete, optimize it offline


	LevenbergMarquardtParams params;
	LevenbergMarquardtOptimizer optimizer(*graph, vals, params);

	////GaussNewtonParams params;
	////GaussNewtonOptimizer optimizer(*graph, vals, params);

	double last_error;
	do {
		last_error = optimizer.error();
		optimizer.iterate();     

		unpack_results(optimizer.values(), MK, info);
		//PLOT_FOR_USERS(info, show_plots_for);
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

	show();


	return 0;
}

int main(int argc, char* argv[]) {

	// run_euroc();
	run_cappella();

	return 0;
}
