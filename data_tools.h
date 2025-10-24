#pragma once
#include "gtsam_test.h"
#include "json.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <chrono>

using namespace std;
using namespace gtsam;

using json = nlohmann::json;

#include "tracker.h"
// struct tracking;



void get_pose_matrix(json d, string& user, Matrix44& pose_matrix);
//void get_GT(json d, vector<string>& users, vector<Matrix44>& pose_matrices);
void get_GT(json d, Pose3& gt_pose);
void get_pose_from_HTM(json d, Pose3& gt_pose);
void get_V(json d, Vector3& velocity);
void get_UWB(json d, string& src_user, string& dst_user, double& range);
void get_IMU(json d, Vector3& accel, Vector3& gyro);

chrono::system_clock::time_point iso_string_to_time(string timeString);

// void get_gt_info(map<string, tracking>& info, json gt_data);
// void get_beacon_info(map<string, tracking>& info, json beacon_data);

void write_trajectory_KITTI_format(vector<Pose3> trajectory, ofstream& fs);
void write_trajectory_TUM_format(vector<Pose3> trajectory, vector<double> timestamps, ofstream& fs, Pose3 transform = Pose3::Identity());
void write_timestamps(vector<Pose3> trajectory, vector<double> timestamps, ofstream& fs);
