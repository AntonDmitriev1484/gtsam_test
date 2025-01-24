#include "data_tools.h"

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

// If something's going funky, check here to make sure I'm not reading in the matrix as it's transpose.
void get_pose_matrix(json d, string& user, Matrix44& pose_matrix) {

	user = d["user"];
	auto m_pose_matrix = d["pose"];
	int i = 0;
	int j = 0;
	for (const auto& row : m_pose_matrix) {
		if (row.is_array()) {
			for (const double& element : row) {
				//HTM_L_G(i, j) = static_cast<double>(element.get<float>());
				pose_matrix(i, j) = element;
				j++;
			}
		}
		j = 0;
		i++;
	}
}

void get_GT(json d, vector<string>& users, vector<Matrix44>& pose_matrices) {

	auto gt_collected_poses = d["poses"];

	for (auto gt_collect : gt_collected_poses) {
		string user;
		Matrix44 HTM_L_U;
		get_pose_matrix(gt_collect, user, HTM_L_U);

		// Filter out static beacons for now
		if (user.find("static") == std::string::npos ) {
			users.push_back(user);
			pose_matrices.push_back(HTM_L_U);
		}
	}

}

void get_UWB(json d, string& src_user, string& dst_user, double& range) {
	src_user = d["src"];
	dst_user = d["dst"];
	range = d["range"];
}

void update_info_with_VIO(json d, map<string, user_info>& info) {
	Matrix44 HTM_L_G;
	string user;
	get_pose_matrix(d, user, HTM_L_G);
	bool is_beacon = user.find("static") != std::string::npos;

	if (info.find(user) == info.end()) {
		// No data registered under VIO should be a beacon
		user_info u;
		if (!is_beacon) {
			u = { is_beacon, HTM_L_G, HTM_L_G, I_4x4, I_4x4, vector<Pose3>(), vector<Pose3>(), vector<Pose3>(), Key() };
		}
		else {
			// This case should never run
			cout << "what" << endl;
			u = { is_beacon, I_4x4, I_4x4, HTM_L_G, I_4x4, vector<Pose3>(), vector<Pose3>(), vector<Pose3>(), Key() };
		}
		info.insert(make_pair(user, u));
	}
	else {
		info.at(user).last_HTM_L_G = HTM_L_G;
	}
}

void update_info_with_GT(json d, map<string, user_info>& info) {

	auto gt_collected_poses = d["poses"];
	for (auto gt_collect : gt_collected_poses) {
		string user;
		Matrix44 HTM_L_U;
		get_pose_matrix(gt_collect, user, HTM_L_U);
		bool is_beacon = user.find("static") != std::string::npos; // If this assumption is wrong, you can also use this

		if (info.find(user) == info.end()) {
			// Users that don't have VO collected for them are always static beacons
			// All beacons appear in the GT measurement
			// So if they are not yet in the map, then they are beacons
			user_info u;
			if (!is_beacon) {
				// This case should never run
				cout << "what2" << endl;
				u = { is_beacon, I_4x4, HTM_L_U, I_4x4, I_4x4, vector<Pose3>(), vector<Pose3>(), vector<Pose3>(), Key() };
			}
			else {
				u = { is_beacon, I_4x4, I_4x4, HTM_L_U, I_4x4, vector<Pose3>(), vector<Pose3>(), vector<Pose3>(), Key()};
			}
			info.insert(make_pair(user, u));
		}

		user_info& u = info.at(user);
		//u.is_beacon = is_beacon; // just to be safe

		u.last_HTM_L_U = HTM_L_U;

		Matrix44 HTM_L_G = u.last_HTM_L_G;

		Matrix44 HTM_G_U = HTM_L_U * HTM_L_G.inverse();
		u.last_HTM_G_U = HTM_G_U;
	}

}

void get_info(json data, map<string, user_info>& info) {


	for (json mes : data) {

		string measurement_type = mes["type"];

		if (measurement_type == "vio") {
			update_info_with_VIO(mes, info);
		}
		else if (measurement_type == "gt") {
			// All static beacons should be given with the GT
			update_info_with_GT(mes, info);
			break;
		}

	}
}

chrono::system_clock::time_point iso_string_to_time(string timeString) {

	// Separate the fractional part (microseconds) from the rest of the string
	size_t dotPos = timeString.find('.');
	std::string timeWithoutMicros = timeString.substr(0, dotPos);
	std::string microsecondsStr = timeString.substr(dotPos + 1);

	// Parse the time without microseconds
	std::istringstream ss(timeWithoutMicros);
	std::tm timeStruct = {};
	ss >> std::get_time(&timeStruct, "%Y-%m-%dT%H:%M:%S");

	// Convert tm to time_point
	std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(std::mktime(&timeStruct));

	// Convert the microseconds part (up to 6 digits)
	int microseconds = std::stoi(microsecondsStr.substr(0, 6)); // Get first 6 digits as microseconds
	tp += std::chrono::microseconds(microseconds);
}