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
	hold(on);
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
	hold(on);

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
	hold(on);

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