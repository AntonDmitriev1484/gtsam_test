#include "utils.h"

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
	scatter3(xs, ys, zs)->color(color);
}

void unpack_results_and_plot(Values results, const function<Key(string, int)>& MK, map<string, user_info> info, vector<string> show_list){

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