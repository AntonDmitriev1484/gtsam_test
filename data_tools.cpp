#include "data_tools.h"

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


void get_gt_info(map<string, tracking>& info, json gt_data) {
	for (json mes : gt_data) {
		if (mes["type"] == "gt_reconstruct") {
			Matrix44 M_L_U;
			string user;
			get_pose_matrix(mes, user, M_L_U);

			//If user hasn't been added yet
			if (info.find(user) == info.end()) {
				tracking t;
				t.is_beacon = false;
				info.insert(make_pair(user, t));
			}
			info.at(user).gt_poses.push_back(Pose3(M_L_U)); //Load all GT poses into tracking.
		}
	}
}

void get_beacon_info(map<string, tracking>& info, json beacon_data) {
	for (json mes : beacon_data) {
			Matrix44 M_L_U;
			string user;
			get_pose_matrix(mes, user, M_L_U);

			//If beacon hasn't been added yet
			if (info.find(user) == info.end()) {
				tracking t;
				t.gt_poses.push_back(Pose3(M_L_U)); // Only need to push back once
				t.is_beacon = true;
				info.insert(make_pair(user, t));
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

void write_trajectory_KITTI_format(vector<Pose3> trajectory, ofstream& fs) {
	// Take the HMT and squish into a row
	if (!fs.is_open()) {  // Check if the file is open
		std::cerr << "Error: File stream is not open!" << std::endl;
		return;
	}

	for (Pose3 pose : trajectory) {
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 4; c++) {
				Matrix44 m = pose.matrix();
				if (r != 3 and c != 3) {
					fs << m(r,c) << " ";
				}
				else {
					fs << m(r,c);
				}
			}
		}
		fs << endl;
	}


}