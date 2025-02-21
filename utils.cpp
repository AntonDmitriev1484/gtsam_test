#include "utils.h"


void gen_single_vio_trajectory_scenario(int N_poses, vector<Pose3>& true_trajectory, vector<Pose3>& gt_points, vector<Pose3>& vio_trajectory) {
	// Generate GT trajectory
	Point3 initial_pos(0, 0, 2);
	Rot3 initial_rot = Rot3::Identity(); // along the +x axis
	Pose3 initial_pose(initial_rot, initial_pos);
	true_trajectory.push_back(initial_pose);

	double theta = 0.125; // rad
	// Rotation about the z-axis by some theta
	Matrix3 d_rot_mat;
	d_rot_mat << cos(theta), -sin(theta), 0,
		sin(theta), cos(theta), 0,
		0, 0, 1;

	Pose3 d_pose(Rot3(d_rot_mat), Point3(1, 0, 0)); // change in pose is a rotation about z, and movement one unit along the local x-axis

	for (int i = 0; i < N_poses; i++) {
		true_trajectory.push_back(d_pose * true_trajectory.back());
	}

	// Discrete GT points
	for (int i = 0; i < N_poses; i += 3) {
		gt_points.push_back(true_trajectory[i]);
	}

	// Generate drifted VIO poses
	Pose3 offset_vio(Rot3::Identity(), Point3(0, 0, 0)); // Assume no offset from initial GT pose
	vio_trajectory.push_back(offset_vio * initial_pose);

	double theta_drift = -0.04;
	Matrix3 drift_rot_mat;
	drift_rot_mat << cos(theta_drift), -sin(theta_drift), 0,
		sin(theta_drift), cos(theta_drift), 0,
		0, 0, 1;
	Rot3 drift_rot(drift_rot_mat);
	Pose3 drift_vio(drift_rot, Point3(+0.0, +0.02, 0));

	for (int i = 1; i < N_poses; i++) {
		Pose3 vio_pose = (d_pose * drift_vio) * vio_trajectory.back();
		vio_trajectory.push_back(vio_pose);
	}
}

void gen_multi_user_vio_trajectory_scenario(int N_poses, vector<vector<Pose3>>& all_true_trajectory, vector<vector<Pose3>>& all_gt_points, vector<vector<Pose3>>& all_vio_trajectory) {

	vector<Pose3> true1, gtp1, vio1;
	gen_single_vio_trajectory_scenario(N_poses, true1, gtp1, vio1);

	gtsam::Matrix3 R;
	R << -1, 0, 0,
		0, 1, 0,
		0, 0, 1;

	Pose3 mirror_transform(Rot3(R), Point3(-10, 5, 0));

	all_true_trajectory.push_back(true1);
	all_gt_points.push_back(gtp1);
	all_vio_trajectory.push_back(vio1);

	vector<Pose3> true2, gtp2, vio2;
	for (int i = 0; i < N_poses; i++) {
		true2.push_back(mirror_transform * true1[i]);
		vio2.push_back(mirror_transform * vio1[i]);
	}

	for (int i = 0; i < N_poses; i++) {
		if (i % 3 == 0) {
			gtp2.push_back(true2[i]);
		}
	}

	all_true_trajectory = { true1, true2 };
	all_gt_points = { gtp1, gtp2 };
	all_vio_trajectory = { vio1, vio2 };
}


void LM_lambda_search(NonlinearFactorGraph* graph, Values vals, vector<Pose3> vio_trajectory, vector<Pose3> gt_points, vector<Pose3> true_trajectory) {
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
	//vector<double> attempt_lambdaInitial = { 10, 1, 0.1, 0.001, 0.0001, 0.00001 };
	//vector<double> attempt_lambdaFactor = { 100000, 10000, 1000, 100, 10, 7, 5, 3 }; // Won't run with 1

	// It seems for this problem, lower lambdafactors give very incorrect estimates.


	// Run 2
	//vector<double> attempt_lambdaInitial = { 20, 17, 14, 10, 1, 0.1};
	//vector<double> attempt_lambdaFactor = {100000, 50000 , 12500, 6000, 4000, 2000, 1000};
	// Going up into lambdaInitial > 10 doesn't seem to help
	// Going from lambdaInitial 1 to 0.1 adds an intersting new curve to the estimated trajectory

	// What happens when we limit lambda lower and upper bound?

	// Run 3
	vector<double> attempt_lambdaInitial = { 10, 5, 1 };
	vector<double> attempt_lambdaFactor = { 100000, 50000 , 1000 };

	// Maybe because it does some writing to the graph per solve?
	// Do i need to make a deep copy of the graph per iteration?

	// Also, seems like l=10 lf=1 make it runs super slow

	for (double lambdaInitial : attempt_lambdaInitial) {

		for (double lambdaFactor : attempt_lambdaFactor) {

			LevenbergMarquardtParams lm_params;
			lm_params.diagonalDamping = true;
			lm_params.setlambdaInitial(lambdaInitial);
			lm_params.lambdaFactor = lambdaFactor;
			lm_params.linearSolverType = NonlinearOptimizerParams::LinearSolverType::MULTIFRONTAL_QR;
			LevenbergMarquardtOptimizer lm_optimizer(*graph, vals, lm_params);

			Values final_vals = lm_optimizer.optimize();
			vector<Pose3> est_trajectory;
			for (int i = 0; i < true_trajectory.size() - 1; i++) est_trajectory.push_back(final_vals.at<Pose3>(MK("x", i)));

			PLOT_W_LM_PARAMS(true_trajectory, gt_points, est_trajectory, vio_trajectory, lambdaInitial, lambdaFactor);
		}
	}

	show();
}

void LM_lambda_search_multiuser_graph(NonlinearFactorGraph* graph, Values vals, vector<vector<Pose3>> vio_trajectory,
	vector<vector<Pose3>> gt_points, vector<vector<Pose3>> true_trajectory) {
	// Make Key
	const function<Key(int, int)> MK = [](int userid, int I) {
		Key k;
		char username;
		if (userid == -1) username = 'a';
		if (userid == 0)  username = 'x';
		if (userid == 1) username = 'y';
		k = symbol(username, I); // e.x. s11 if 'static11'

		return k;
	};

	// Run 1
	//vector<double> attempt_lambdaInitial = { 10, 1, 0.1, 0.001, 0.0001, 0.00001 };
	//vector<double> attempt_lambdaFactor = { 100000, 10000, 1000, 100, 10, 7, 5, 3 };

	vector<double> attempt_lambdaInitial = { 10, 1, 0.1, 0.001, 0.0001, 0.00001 };
	vector<double> attempt_lambdaFactor = { 7, 5, 3.5, 3 , 2.5, 2, 1.5};

	for (double lambdaInitial : attempt_lambdaInitial) {

		for (double lambdaFactor : attempt_lambdaFactor) {

			LevenbergMarquardtParams lm_params;
			lm_params.diagonalDamping = true;
			lm_params.setlambdaInitial(lambdaInitial);
			lm_params.lambdaFactor = lambdaFactor;
			lm_params.linearSolverType = NonlinearOptimizerParams::LinearSolverType::MULTIFRONTAL_QR;
			LevenbergMarquardtOptimizer lm_optimizer(*graph, vals, lm_params);

			Values final_vals = lm_optimizer.optimize();

			int N_users = true_trajectory.size();
			vector<vector<Pose3>> est_trajectory;
			for (int usr = 0; usr < N_users; usr++) {

				vector<Pose3> usr_est_trajectory;
				for (int i = 0; i < true_trajectory[usr].size() - 1; i++) usr_est_trajectory.push_back(final_vals.at<Pose3>(MK(usr, i)));
				est_trajectory.push_back(usr_est_trajectory);
				
			}
			PLOT_MULTI_W_LM_PARAMS(N_users, true_trajectory, gt_points, est_trajectory, vio_trajectory, lambdaInitial, lambdaFactor);
		}
	}
	show();
}







void draw_vector(Vector3 start, Vector3 end, string color) {

	hold(on);

	vector<double> dx = { start.x() , end.x() };
	vector<double> dy = { start.y(), end.y() };
	vector<double> dz = { start.z(), end.z() };

	plot3(dx, dy, dz)->color(color);
}

void draw_coordinate_frame_axes(Rot3 rot_S_to_R, Vector3 loc_R) {

	double length = 0.5;

	hold(on);

	Rot3 T = rot_S_to_R;
	Matrix33 M = rot_S_to_R.matrix();
	// I think the rotator is somehow re-scaling the vectors in transform?

	Rot3 x_axis();

	Vector3 x_S(1, 0, 0);
	Vector3 y_S(0, 1, 0);
	Vector3 z_S(0, 0, 1);

	// Draw reference coordinate frame
	//draw_vector(loc_R, loc_R + x_S, "black");
	//draw_vector(loc_R, loc_R + y_S, "black");
	//draw_vector(loc_R, loc_R + z_S, "black");

	Vector3 x_R = T * x_S;
	Vector3 y_R = T * y_S;
	Vector3 z_R = T * z_S;

	// Draw rotated coordinate frame
	draw_vector(loc_R, loc_R + x_R, "red");
	draw_vector(loc_R, loc_R + y_R, "blue");
	draw_vector(loc_R, loc_R + z_R, "green");

}

void draw_basis(Matrix33 basis, Vector3 loc, bool as_reference_frame) {
	//hold(on);
	double length = 0.25;
	if (as_reference_frame) {
		string color = "black";
		draw_vector(loc, loc + length * basis.col(0), color);
		draw_vector(loc, loc + length * basis.col(1), color);
		draw_vector(loc, loc + length * basis.col(2), color);

	}
	else {
		draw_vector(loc, loc + length * basis.col(0), "red");
		draw_vector(loc, loc + length * basis.col(1), "blue");
		draw_vector(loc, loc + length * basis.col(2), "green");
	}

}

void draw_trajectory(vector<Pose3> trajectory, string color) {
	//hold(on);

	vector<float> xs;
	vector<float> ys;
	vector<float> zs;
	for (Pose3 pose : trajectory) {
		xs.push_back(pose.x());
		ys.push_back(pose.y());
		zs.push_back(pose.z());
	}
	plot3(xs, ys, zs)->color(color);
}

void draw_trajectory_with_orientation(vector<Pose3> trajectory, string color) {
	//hold(on);

	vector<float> xs;
	vector<float> ys;
	vector<float> zs;
	for (Pose3 pose : trajectory) {
		xs.push_back(pose.x());
		ys.push_back(pose.y());
		zs.push_back(pose.z());

		draw_vector(pose.translation(), pose.translation() + pose.rotation() * Vector3(1, 0, 0), "black");
	}
	plot3(xs, ys, zs)->color(color);
}

void draw_points(vector<Pose3> points, string color) {
	//hold(on);

	vector<double> xs;
	vector<double> ys;
	vector<double> zs;
	for (Pose3 pose : points) {
		xs.push_back(pose.x());
		ys.push_back(pose.y());
		zs.push_back(pose.z());
	}
	scatter3(xs, ys, zs)->marker_face_color({0.0,1.0,0.0}); // Hard coded to green
}

void unpack_results(Values results, const function<Key(string, int)>& MK, map<string, tracking_info>& info) {
	for (auto& [user, user_info] : info) {
		for (int i = 0; i < user_info.I; i++) {
			if (!user_info.is_beacon) {
				Key k = MK(user, i);
				Pose3 estimated_pose = results.at<Pose3>(k);
				user_info.est_poses.push_back(estimated_pose);
			}
		}
	}
}

void unpack_results_and_plot(Values results, const function<Key(string, int)>& MK, map<string, tracking_info> info, vector<string> show_list){

	// Plotting code

	for (auto& [user, user_info] : info) {
		for (int i = 0; i < user_info.I; i++) {
			if (!user_info.is_beacon) {
				Key k = MK(user, i);
				Pose3 estimated_pose = results.at<Pose3>(k);
				user_info.est_poses.push_back(estimated_pose);
			}
		}
	}

	for (const auto& [user_name, user_info] : info) {
		if (!user_info.is_beacon) {
			if (find(show_list.begin(), show_list.end(), user_name) != show_list.end()) {
				auto fig = figure();
				fig->name(user_name + " trajectory");
				title(user_name);

				hold(on);
				draw_trajectory(user_info.vio_poses, "red");
				hold(on);
				draw_points(user_info.gt_poses, "green");
				hold(on);
				draw_trajectory(user_info.est_poses, "blue");

				cout << "vio size " << user_info.vio_poses.size() << " estimated size " << user_info.est_poses.size() << endl;

				xlabel("X (m)");
				ylabel("Z (m)");  // Switch the label to match the upward axis
				zlabel("Y (m)");

				show();
			}
		}
	}
}

void clear_results(map<string, tracking_info>& info) {
	for (auto& [user, user_info] : info) {
		for (int i = 0; i < user_info.I; i++) {
			if (!user_info.is_beacon) {
				user_info.est_poses.clear();
			}
		}
	}
}