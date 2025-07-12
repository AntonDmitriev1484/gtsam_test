#pragma once

#include "tracker.h"
#include "data_tools.h"


void get_pose_matrix(json d, string& user, Matrix44& pose_matrix) {

	//user = d["user"];
	auto m_pose_matrix = d["T_body_world"];
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

void get_GT_HTM(json d, Pose3& gt_pose) {
	Matrix44 pose_mat;
	string usr;
	get_pose_matrix(d, usr, pose_mat);
	gt_pose = Pose3(pose_mat);
}

void get_GT(json d, Pose3& gt_pose) {
	// Create rotation from quaternion format
	gt_pose = Pose3(Rot3(d["qx"], d["qy"], d["qz"], d["qw"]), Point3(d["x"], d["y"], d["z"]));
}

void get_V(json d, Vector3& velocity) {
	velocity = Vector3(
		d["v_world"]["vx"], 
		d["v_world"]["vy"], 
		d["v_world"]["vz"]);
}

void get_UWB(json d, string& src_user, string& dst_user, double& range) {
	//src_user = "2";
	dst_user = to_string(d["id"]); // Was ID
	range = d["range"]; // Was RANGE
}

void get_IMU(json d, Vector3& accel, Vector3& gyro) {
	accel = Vector3(d["ax"], d["ay"], d["az"]);
	gyro = Vector3(d["gx"], d["gy"], d["gz"]);
}

// void get_gt_info(map<string, tracking>& info, json gt_data) {
// 	for (json mes : gt_data) {
// 		if (mes["type"] == "gt_reconstruct") {
// 			Matrix44 M_L_U;
// 			string user;
// 			get_pose_matrix(mes, user, M_L_U);

// 			//If user hasn't been added yet
// 			if (info.find(user) == info.end()) {
// 				tracking t;
// 				t.is_beacon = false;
// 				info.insert(make_pair(user, t));
// 			}
// 			info.at(user).gt_poses.push_back(Pose3(M_L_U)); //Load all GT poses into tracking.
// 		}
// 	}
// }

// void get_beacon_info(map<string, tracking>& info, json beacon_data) {
// 	// Beacon position will 
// 	for (json beacon : beacon_data) {
// 		Rot3 rot();
// 		Vector3 v;
// 		auto raw_position = beacon["position"];
// 		int i = 0;
// 		for (const auto& row : raw_position) {
// 			v(i) = row;
// 			i++;
// 		}

// 		string user = to_string(beacon["ID"]);
// 		Pose3 beacon_pos(Rot3::Identity(), v);

// 			//If beacon hasn't been added yet
// 			if (info.find(user) == info.end()) {
// 				tracking t;
// 				t.gt_poses.push_back(beacon_pos); // Only need to push back once
// 				t.is_beacon = true;
// 				info.insert(make_pair(user, t));
// 			}
// 	}
// }

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

	// Interesting, we get a non - 0 0 0 1 last row
	// They tell you to cut off the last row anyways...

	for (Pose3 pose : trajectory) {
		Matrix44 m = pose.matrix();
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 4; c++) {

				if (r == 2 && c == 3) {
					fs << m(r, c) << endl;
				}
				else {
					fs << m(r, c) << " ";
				}


			}
		}
	}
}

// void write_vector_HTM_format(vector<Vector3> trajectory, ofstream& fs) {
// 	// Take the HMT and squish into a row
// 	if (!fs.is_open()) {  // Check if the file is open
// 		std::cerr << "Error: File stream is not open!" << std::endl;
// 		return;
// 	}

// 	// Interesting, we get a non - 0 0 0 1 last row
// 	// They tell you to cut off the last row anyways...

// 	for (Vector3 pose : trajectory) {
// 		Matrix44 m = pose.matrix();
// 		for (int r = 0; r < 3; r++) {
// 			for (int c = 0; c < 4; c++) {

// 				if (r == 2 && c == 3) {
// 					fs << m(r, c) << endl;
// 				}
// 				else {
// 					fs << m(r, c) << " ";
// 				}


// 			}
// 		}
// 	}
// }

void write_trajectory_TUM_format(vector<Pose3> trajectory, vector<double> timestamps, ofstream& fs, Pose3 transform) {

	fs << std::fixed << std::setprecision(8);  // Set once before the loop

    for (size_t i = 0; i < trajectory.size(); i++) {
        const Pose3& pose = trajectory[i];
		Pose3 out_pose = pose * transform; // Apply some transform to each pose before dumping

        Point3 t = out_pose.translation();
        Rot3 R = out_pose.rotation();
        Quaternion q = out_pose.rotation().toQuaternion();

        fs << timestamps[i] << " ";
        fs << t.x() << " " << t.y() << " " << t.z() << " ";
        fs << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }

}

void write_timestamps(vector<Pose3> trajectory, vector<double> timestamps, ofstream& fs) {
	fs << std::fixed << std::setprecision(6);  // For 6 digits after the decimal

	for (size_t i = 0; i < timestamps.size(); ++i) {
		fs << std::setw(6) << std::setfill('0') << i << " "  // zero-padded index
		<< timestamps[i] << "\n";                         // full timestamp
	}

}