#include "data_loader.h"

// This parses a single line of EuRoC GT, all parameters by reference
bool parse_EuRoC_gt_line(ifstream& gt_file, Point3& position, Rot3& rotation,
	Vector3& velocity, Vector3& gyro_bias, Vector3& accel_bias) {

	string value;
	getline(gt_file, value, ','); // Skip over timestamp
	if (gt_file.eof()) return false;

	Vector3 start_pos;
	for (int i = 0; i < 3; i++) {
		getline(gt_file, value, ',');
		start_pos(i) = stof(value.c_str());
	}
	Vector4 start_rot;
	for (int i = 0; i < 4; i++) {
		getline(gt_file, value, ',');
		start_rot(i) = stof(value.c_str());
	}
	for (int i = 0; i < 3; i++) {
		getline(gt_file, value, ',');
		velocity(i) = stof(value.c_str());
	}
	// EuRoC GT also gives you the bias in the IMU sensor
	for (int i = 0; i < 3; i++) {
		getline(gt_file, value, ',');
		gyro_bias(i) = stof(value.c_str());
	}
	for (int i = 0; i < 2; i++) {
		getline(gt_file, value, ',');
		accel_bias(i) = stof(value.c_str());
	}
	getline(gt_file, value, '\n');
	accel_bias(2) = stof(value.c_str());

	// TODO: GT sensor is rotated out of body frame according to its sensor.yaml
	// Does this mean I need to rotate all of these?

	Point3 p(start_pos(0), start_pos(1), start_pos(2));
	position = p;
	Rot3 r(start_rot(0), start_rot(1), start_rot(2), start_rot(3)); // Rotation as quaternion
	rotation = r;

	bool havemore = !gt_file.eof();
	return havemore;
}

// This parses a single line of EuRoC IMU, all parameters by reference
bool parse_EuRoC_imu_line(ifstream& imu_file, Vector3& V_angular, Vector3& A_axial) {
	string s;
	getline(imu_file, s, ','); // Skip over timestamp
	if (imu_file.eof()) return false;

	for (int j = 0; j < 3; j++) {
		getline(imu_file, s, ',');
		V_angular(j) = stof(s.c_str());
	}
	for (int j = 0; j < 2; j++) {
		getline(imu_file, s, ',');
		A_axial(j) = stof(s.c_str());
	}
	// Last value in EuRoC row is terminated with \r\n instead of a comma
	getline(imu_file, s, '\r');
	A_axial(2) = stof(s.c_str());
	return !imu_file.eof(); // Return true so long as we arent at the end of the file
}