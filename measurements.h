#pragma once
#include "gtsam_test.h"
#include "nlohmann/json.hpp"
#include "data_tools.h"
#include "utils.h"
#include "cmath"
#include <regex>

using PreintegrationType = gtsam::PreintegrationBase;
using namespace gtsam;
using namespace std;
using json = nlohmann::json;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)


void processGT( json mes,
	tracking& user,
	NonlinearFactorGraph* graph,
	Values& vals,
	ISAM2* isam,
	PreintegrationType* imu_preintegrated,
	NavState& prev_state,
	const SharedNoiseModel& GT_noise_model,
	Pose3 T_imu_body,
	string debug_dump_directory);

void processUWB(
	json mes,
	string src_user,
	map<string, tracking > & info,
	NonlinearFactorGraph* graph,
	Values& vals,
	ISAM2* isam,
	PreintegrationType* imu_preintegrated,
	NavState& prev_state,
	const SharedNoiseModel& UWB_noise_model,
	const SharedNoiseModel& GT_noise_model,
	int& imu_counter,
	int& imu_count_at_last_correction,
	double dt,
	int& uwb_counter,
	string debug_dump_directory);

void processSyntheticUWB(
	json mes,
	string src_user,
	map<string, tracking > & info,
	NonlinearFactorGraph* graph,
	Values& vals,
	ISAM2* isam,
	PreintegrationType* imu_preintegrated,
	NavState& prev_state,
	const SharedNoiseModel& UWB_noise_model,
	const SharedNoiseModel& GT_noise_model,
	const SharedNoiseModel& FakePrior_noise_model,
	int& imu_counter,
	int& imu_count_at_last_correction,
	double dt,
	int& uwb_counter,
	Pose3& T_imu_body,
	string debug_dump_directory,
	double uwb_stdev);