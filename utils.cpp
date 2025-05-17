#include "utils.h"

Matrix44 get_vis_rotation() {
	Matrix44 vis_rotation = Matrix::Zero(4, 4);
	vis_rotation(0, 0) = 1;
	vis_rotation(2, 1) = 1;
	vis_rotation(1, 2) = 1;
	vis_rotation(3, 3) = 1;
	return vis_rotation;
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

	Matrix44 vis_rotation(get_vis_rotation());

	basis = Pose3(vis_rotation).rotation().matrix() * basis;
	loc = Pose3(vis_rotation).rotation().matrix() * loc;

	double length = 1;
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

void draw_forward(Pose3 pose, double scale, string color) {
	draw_vector(pose.translation(), pose.translation() + (pose.rotation() * Vector3(1, 0, 0) * (scale)), "red");
	draw_vector(pose.translation(), pose.translation() + (pose.rotation() * Vector3(0, 1, 0) * (scale)), "blue");
	draw_vector(pose.translation(), pose.translation() + (pose.rotation() * Vector3(0, 0, 1) * (scale)), "green");
}

void draw_error_ellipsoid(Vector3 center, Vector3 position_error, string color) {
	double a = position_error(0); // X-axis
	double b = position_error(1); // Y-axis
	double c = position_error(2); // Z-axis

	// Center point
	double cx = center(0);
	double cy = center(1);
	double cz = center(2);

	auto theta = linspace(0, 2 * M_PI, 50);
	auto phi = linspace(0, M_PI, 50);

	std::vector<std::vector<double>> X(theta.size(), std::vector<double>(phi.size()));
	std::vector<std::vector<double>> Y(theta.size(), std::vector<double>(phi.size()));
	std::vector<std::vector<double>> Z(theta.size(), std::vector<double>(phi.size()));

	for (size_t i = 0; i < theta.size(); ++i) {
		for (size_t j = 0; j < phi.size(); ++j) {
			X[i][j] = cx + a * std::sin(phi[j]) * std::cos(theta[i]);
			Y[i][j] = cy + b * std::sin(phi[j]) * std::sin(theta[i]);
			Z[i][j] = cz + c * std::cos(phi[j]);
		}
	}

	mesh(X, Y, Z)->edge_color(color); // Wireframe, transparent ellipsoid

}

void draw_trajectory_with_error(vector<Pose3> trajectory, vector<Vector3> pose_errors, string color) {

	vector<float> xs;
	vector<float> ys;
	vector<float> zs;

	vector<double> xs_err, ys_err, zs_err;

	vector<double> errors;
	int count = 0;
	for (Pose3 pose : trajectory) {
		xs.push_back(pose.x());
		ys.push_back(pose.y());
		zs.push_back(pose.z());

		if (color == "blue" && count % 25 == 0) { // No idea why it needs a special invite to plot blue but oh well.
			draw_forward(pose, 0.1, color);
			Vector3 error = pose_errors[count] * 1;
			// Might just draw a point with a radius that is the maximum error along any axis?
			// This can appear as transparent with a border, rendering these meshes takes too long.
			
			double max_err = 0;
			for (int i = 0; i < 3; i++) {
				if (abs(error(i)) > max_err) max_err = abs(error(i));
			}

			xs_err.push_back(pose.x());
			ys_err.push_back(pose.y());
			zs_err.push_back(pose.z());
			errors.push_back(max_err);

			// Variances are tiny! How else can I visualize error?
			cout << "ErrX: " << error(0) <<  " ErrY: " << error(1) << " ErrZ: " << error(2) << endl;
			//draw_error_ellipsoid(pose.translation(),error, "black");
		}

		count++;
	}
	plot3(xs, ys, zs)->color(color);

	//for (size_t i = 0; i < xs_err.size(); ++i) {
	//	auto s = scatter3(
	//		std::vector<double>{xs_err[i]},
	//		std::vector<double>{ys_err[i]},
	//		std::vector<double>{zs_err[i]}
	//	);
	//	s->marker_face_color("none"); // transparent fill
	//	s->marker_size(errors[i]);
	//}



}

void draw_trajectory(vector<Pose3> trajectory, string color) {

	vector<float> xs;
	vector<float> ys;
	vector<float> zs;
	int count = 0;
	for (Pose3 pose : trajectory) {
		xs.push_back(pose.x());
		ys.push_back(pose.y());
		zs.push_back(pose.z());
		//if ( color == "blue") {
		//	draw_forward(pose, 0.1, color);
		//}

		if (color == "green" && count % 50 ==0) { // No idea why it needs a special invite to plot blue but oh well.
			draw_forward(pose, 0.1, color);
		}
		//if (count % 50 == 0) {
		//	draw_forward(pose, 0.1, color);
		//}
		count++;
	}
	plot3(xs, ys, zs)->color(color);
}

void draw_points(vector<Pose3> points, string color) {
	//hold(on);

	//	// For cappella (AND ONLY CAPPELLA) we rotate every trajectory s.t. y is facing up
	//Matrix44 vis_rotation = get_vis_rotation();
	//vector<Pose3> trajectory_cp(points); // Deep copy and rotate each element
	//for (int i = 0; i < trajectory_cp.size(); i++) trajectory_cp[i] = Pose3(vis_rotation) * trajectory_cp[i];

	vector<double> xs;
	vector<double> ys;
	vector<double> zs;
	for (Pose3 pose : points) {
		xs.push_back(pose.x());
		ys.push_back(pose.y());
		zs.push_back(pose.z());
	}
	scatter3(xs, ys, zs)->marker_face_color({1.0,0.0,0.0}); // Hard coded to green
}

void unpack_results(Values results, const function<Key(string, int)>& MK, map<string, tracking>& info) {
	for (auto& [user, user_info] : info) {
		for (int i = 0; i < user_info.Ix; i++) {
			if (!user_info.is_beacon) {
				Key k = MK(user, i);
				Pose3 estimated_pose = results.at<Pose3>(k);
				user_info.est_poses.push_back(estimated_pose);
			}
		}
	}
}

void unpack_results_and_plot(Values results, const function<Key(string, int)>& MK, map<string, tracking> info, vector<string> show_list){

	// Plotting code

	for (auto& [user, user_info] : info) {
		for (int i = 0; i < user_info.Ix; i++) {
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

void clear_results(map<string, tracking>& info) {
	for (auto& [user, user_info] : info) {
		for (int i = 0; i < user_info.Ix; i++) {
			if (!user_info.is_beacon) {
				user_info.est_poses.clear();
			}
		}
	}
}