#pragma once
#include "gtsam_test.h"
#include "nlohmann/json.hpp"
#include "data_tools.h"
#include "utils.h"
#include "cmath"
#include <regex>

using PreintegrationType = gtsam::PreintegrationBase;
using PreintegrationParams = gtsam::PreintegratedCombinedMeasurements::Params;
// using namespace gtsam;
// using namespace std;
using json = nlohmann::json;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

struct tracking {

	// Computed in get_info
	bool is_beacon;

	vector<Pose3> gt_poses;
	vector<double> gt_timestamps; // Parallel array to GT poses.

	vector<Pose3> est_poses; // Estimated pose
	vector<double> est_timestamps; // Parallel array to poses.
	vector<Vector3> est_velocities; // Estimated velocity from IMU factor
	vector<Vector3> est_poses_error; // Estimated poses error

	imuBias::ConstantBias constant_bias; // Constant bias

	Key pose_key;
	int Ix;
	int Iv;
	int Ib;
};

class Tracker {
public:

    ISAM2* isam;
    NonlinearFactorGraph* graph;
    Values vals;

    NavState prev_state;
    
    PreintegrationType* imu_preintegrated;
    imuBias::ConstantBias prior_imu_bias;

    Pose3 T_imu_body;
    SharedNoiseModel GT_noise_model;
    SharedNoiseModel UWB_noise_model;
    SharedNoiseModel FakePrior_noise_model;
    SharedNoiseModel Velocity_noise_model;
    SharedNoiseModel Bias_noise_model;

    string debug_dir;

    double delta_t;
    string id;
    tracking track;


Tracker(const string& id,
            const Pose3& T_imu_body,
            const SharedNoiseModel& GT_noise_model,
            const SharedNoiseModel& UWB_noise_model,
            const SharedNoiseModel& FakePrior_noise_model,
            const SharedNoiseModel& Velocity_noise_model,
            const SharedNoiseModel& Bias_noise_model,
            const imuBias::ConstantBias prior_imu_bias,
            const double delta_t,
            const string& debug_dir);

    void init(json sensor_stream);
    void init_anchor();

    void processGT(const json& mes, tracking& user, NavState& prev_state);
    void processSyntheticUWB(const json& mes,
                             const string& src_user,
                             std::map<string, tracking>& info,
                             NavState& prev_state,
                             int& imu_counter,
                             int& imu_count_at_last_correction,
                             double dt,
                             int& uwb_counter,
                             double uwb_stdev);
};


