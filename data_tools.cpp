#pragma once

#include "tracker.h"
#include "data_tools.h"


void get_pose_matrix(json d, string& user, Matrix44& pose_matrix) {

	auto m_pose_matrix = d;
	int i = 0;
	int j = 0;
	for (const auto& row : m_pose_matrix) {
		if (row.is_array()) {
			for (const double& element : row) {
				pose_matrix(i, j) = element;
				j++;
			}
		}
		j = 0;
		i++;
	}

}

void get_pose_from_HTM(json d, Pose3& pose) {
	Matrix44 pose_mat;
	string usr;
	get_pose_matrix(d, usr, pose_mat);
	pose = Pose3(pose_mat);
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


void write_trajectory_TUM_format(vector<Pose3> trajectory, vector<double> timestamps, ofstream& fs, const Pose3& transform) {

	fs << std::fixed << std::setprecision(8);  // Set once before the loop

    for (size_t i = 0; i < trajectory.size(); i++) {
        const Pose3& T_body_to_world = trajectory[i];

		Pose3 out_pose = T_body_to_world.inverse();

        Point3 t = out_pose.translation();
        Rot3 R = out_pose.rotation();
        Quaternion q = out_pose.rotation().toQuaternion();

        fs << timestamps[i] << " ";
        fs << t.x() << " " << t.y() << " " << t.z() << " ";
        fs << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
}

// Specifically written to make this graph output compatible with plot_all
void write_trajectory_HTM_JSON_format(
    const vector<Pose3>& trajectory,
    const vector<double>& timestamps,
    ofstream& fs,
    const std::string& pose_type,
    double start,
    double init_newmap,
    double end
){
    json traj_json = json::array();

    for (size_t i = 0; i < trajectory.size(); i++) {
        const Pose3& T_body_to_world = trajectory[i];

        // Convert Pose3 -> 4x4 homogeneous transform matrix
        Pose3 out_pose = T_body_to_world.inverse();

        Matrix4 H = out_pose.matrix();

        json pose_json;
        pose_json["t"] = timestamps[i];
        pose_json["type"] = pose_type;

        pose_json["T_body_world"] = {
            {H(0,0), H(0,1), H(0,2), H(0,3)},
            {H(1,0), H(1,1), H(1,2), H(1,3)},
            {H(2,0), H(2,1), H(2,2), H(2,3)},
            {H(3,0), H(3,1), H(3,2), H(3,3)}
        };

		// Mark down failure intervals
		string status = "tracking";
		pose_json["status"] = status; // Re apply status label to newly generated SLAM poses

        traj_json.push_back(pose_json);
    }

    fs << traj_json.dump(2);  // pretty print with indent=2
}

void write_anchor_positions(
    std::map<string, tracking>& anchors,
    std::ofstream& fs)
{
    json output = json::array();

    for (const auto& [id, tracking] : anchors) {

        if (tracking.est_poses.empty()) {
            continue;
        }

        const auto p = tracking.est_poses.back().translation();

        output.push_back({
            {"ID", id},
            {"position", {p.x(), p.y(), p.z()}}
        });
    }

    fs << output.dump(4);
}

void write_timestamps(vector<Pose3> trajectory, vector<double> timestamps, ofstream& fs) {
	fs << std::fixed << std::setprecision(6);  // For 6 digits after the decimal

	for (size_t i = 0; i < timestamps.size(); ++i) {
		fs << std::setw(6) << std::setfill('0') << i << " "  // zero-padded index
		<< timestamps[i] << "\n";                         // full timestamp
	}

}