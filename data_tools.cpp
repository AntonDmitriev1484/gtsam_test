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

void update_info_with_VIO(json d, map<string, tracking_info>& info) {
	Matrix44 HTM_L_G;
	string user;
	get_pose_matrix(d, user, HTM_L_G);
	bool is_beacon = user.find("static") != std::string::npos;

	if (info.find(user) == info.end()) {
		// No data registered under VIO should be a beacon
		tracking_info u;
		if (!is_beacon) {
			u = { is_beacon, HTM_L_G, HTM_L_G, I_4x4, I_4x4, vector<Pose3>(), vector<Pose3>(), vector<Pose3>(), vector<string>(), Key() ,0};
		}
		else {
			// This case should never run
			cout << "what" << endl;
			u = { is_beacon, I_4x4, I_4x4, HTM_L_G, I_4x4, vector<Pose3>(), vector<Pose3>(), vector<Pose3>(), vector<string>(), Key() ,0 };
		}
		info.insert(make_pair(user, u));
	}
	else {
		info.at(user).last_HTM_L_G = HTM_L_G;
	}
}

void update_info_with_GT(json d, map<string, tracking_info>& info) {

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
			tracking_info u;
			if (!is_beacon) {
				// This case should never run
				cout << "what2" << endl;
				u = { is_beacon, I_4x4, HTM_L_U, I_4x4, I_4x4, vector<Pose3>(), vector<Pose3>(), vector<Pose3>(), vector<string>(), Key() ,0};
			}
			else {
				u = { is_beacon, I_4x4, I_4x4, HTM_L_U, I_4x4, vector<Pose3>(), vector<Pose3>(), vector<Pose3>(), vector<string>(), Key() ,0};
			}
			info.insert(make_pair(user, u));
		}

		tracking_info& u = info.at(user);
		//u.is_beacon = is_beacon; // just to be safe

		u.last_HTM_L_U = HTM_L_U;

		Matrix44 HTM_L_G = u.last_HTM_L_G;

		Matrix44 HTM_G_U = HTM_L_U * HTM_L_G.inverse();
		u.last_HTM_G_U = HTM_G_U;
	}

}

void get_info(json data, map<string, tracking_info>& info) {


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

void dump_reconstructed_trajectories(map<string, tracking_info> info, string filename) {
	// Write the reconstructed GT trajectories for each user to a json file
	// Run a python script to sort entries into chronological order

	// TODO: Might need to add a starter curly brace and ending curly brace that wrap around all items?
	std::ofstream fs(filename);
	json jarray = json::array();
	for (auto& [user, user_info] : info) {
		vector<string> timestamps = user_info.est_poses_iso_timestamp;
		vector<Pose3> trajectory = user_info.est_poses;

		for (int i = 0; i < trajectory.size(); i++) {
			Pose3 pose_U = trajectory[i];
			Matrix4 m = pose_U.matrix();
			json jdata = {
					{"timestamp", timestamps[i]},
					{"type", "gt_reconstruct"},
					{"user", user},
					{"pose", {
						{ m(0,0),m(0,1), m(0,2), m(0,3)},
						{ m(1,0),m(1,1), m(1,2), m(1,3)},
						{ m(2,0),m(2,1), m(2,2), m(2,3)},
						{ m(3,0),m(3,1), m(3,2), m(3,3)},
					}
			}
			};
			jarray.push_back(jdata);
		}
	}

	fs << jarray.dump(1);

	fs.close();


}
