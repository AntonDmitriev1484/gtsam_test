#pragma once

#include "gtsam_test.h"

#include <cstring>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

bool parse_EuRoC_gt_line(ifstream& gt_file, Point3& position, Rot3& rotation,
	Vector3& velocity, Vector3& gyro_bias, Vector3& accel_bias);

bool parse_EuRoC_imu_line(ifstream& imu_file, Vector3& V_angular, Vector3& A_axial);