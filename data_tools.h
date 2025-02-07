#pragma once

// Probably would be better to rename this dataset_utils or something

#include "gtsam_test.h"
//#include "<nlohmann/json.hpp>"
#include "nlohmann/json.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <chrono>

using namespace std;
using namespace gtsam;

using json = nlohmann::json;

// Cappella parsing code
struct tracking_info {

	// Computed in get_info
	bool is_beacon;
	Matrix44 first_HTM_L_G; // lets us compute first VIO pose
	Matrix44 last_HTM_L_G; // most recent VIO pose
	Matrix44 last_HTM_L_U; // most recent GT pose
	Matrix44 last_HTM_G_U; // most recent estimate of what Universal -> Global transform is

	// Used to store trajectory at runtime
	vector<Pose3> vio_poses;
	vector<Pose3> gt_poses;
	vector<Pose3> est_poses; // Estimated poses

	Key pose_key;
	int I;
};

void get_pose_matrix(json d, string& user, Matrix44& pose_matrix);
void get_GT(json d, vector<string>& users, vector<Matrix44>& pose_matrices);
void get_UWB(json d, string& src_user, string& dst_user, double& range);
void get_info(json data, map<string, tracking_info>& info);

chrono::system_clock::time_point iso_string_to_time(string timeString);

void get_info2(json raw_data, json gt_data, map<string, tracking_info>& info);

void get_gt_info(map<string, tracking_info>& info, json gt_data);


