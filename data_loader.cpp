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

void get_info(json data, map<string, user_info>& info) {

	// It will add static beacons as users.
	// Just remove them from the map before doing any processing
	// Or I'm guessing we'll need to treat them as users for the ranges.
	// Also my code doesn't necessarily get all static beacons

	// Ok I think the poses of all of the static beacons are included in every groundtruth

	for (json mes : data) {

		string measurement_type = mes["type"];

		if (measurement_type == "vio") {

			Matrix44 HTM_L_G;
			string user;
			get_pose_matrix(mes, user, HTM_L_G);

			// If this user hasn't been added to our map yet
				if (info.find(user) == info.end()) {
					// Initialize the struct
					user_info u = { HTM_L_G, I_4x4, I_4x4, vector<Pose3>(), vector<Pose3>() };
					info.insert(make_pair(user, u));
				}
				else {
					info.at(user).last_HTM_L_G = HTM_L_G;
				}

		}
		else if (measurement_type == "gt") {

			vector<Matrix44> HTM_L_U_per_user;
			vector<string> users;
			get_GT(mes, users, HTM_L_U_per_user);
			// The matrix corresponding to the user will be at the same index

			for (int i = 0; i < users.size(); i++) {

				string username = users[i];

					Matrix44 HTM_L_U = HTM_L_U_per_user[i];
					Matrix44 HTM_L_G = info[username].last_HTM_L_G;

					Matrix44 HTM_G_U = HTM_L_U * HTM_L_G.inverse();

					info.at(username).last_HTM_L_U = HTM_L_U;
					info.at(username).last_HTM_G_U = HTM_G_U;

				//info.insert(make_pair(username, u));
				// Insert only adds in new pairs, it doesn't automatically add new ones
			}

			// Break, once we have computed an initial HTM_G_U for all users.
			// This will let us translate everything into the universal frame.
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