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
    vector<Pose3> correction_poses; // Just the poses resulting from a correction
	vector<double> est_timestamps; // Parallel array to poses.

	vector<Vector3> est_velocities; // Estimated velocity from IMU factor
	vector<Vector3> est_poses_error; // Estimated poses error

	imuBias::ConstantBias constant_bias; // Constant bias
    PreintegrationBase::Bias changing_bias; // dynamic bias

	Key pose_key;
	int Ix;
	int Iv;
	int Ib;
};

class Tracker {
public:

    bool use_smoother;
    ISAM2* isam;
    IncrementalFixedLagSmoother* smoother; // I didn't even know 'er
    FixedLagSmoother::KeyTimestampMap key_timestamps;

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
    double elapsed_t; // Time elapsed since last correction

    string id;
    tracking track;

    std::map<string, tracking> anchors;

	std::mt19937 uwb_rng;   // Random Number Generator for synthetic UWB measurements
    double uwb_stdev;
    
    //Debug
    vector<Pose3> suwb_base_poses;
    vector<Pose3> mag_vectors; // identity rotation, just translation
    vector<Pose3> postproc_velocity_vectors;
    vector<Pose3> est_velocity_vectors;

    double mes_start;

    Tracker(const string& id,
            const Pose3& T_imu_body,
            const double delta_t,
            const double smoother_lag,
            const bool use_smoother,
            const double uwb_stdev,
            const SharedNoiseModel& GT_noise_model,
            const SharedNoiseModel& UWB_noise_model,
            const SharedNoiseModel& FakePrior_noise_model,
            const SharedNoiseModel& Velocity_noise_model,
            const SharedNoiseModel& Bias_noise_model,
			std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params,
            const imuBias::ConstantBias prior_imu_bias,
            const string& debug_dir);

    void init(json sensor_stream);
    void init_anchors(json anchor_json);
    void init_anchor(string id);
    void init_state(json mes);

    void exec_iSAM(NavState& proposed, double mes_timestamp, 
        string msg="", bool print=false);
void exec_iSAM_GT(NavState& proposed, double mes_timestamp, 
    string msg, bool print);
    void exec_smoother(NavState& proposed, double mes_timestamp, 
        string msg="", bool print=false);

    void processSLAM(const json& mes);
    
    void processSUWB(const json& mes, int& uwb_counter, double uwb_stdev);

    Pose3 filteredPose(Rot3 preintegration_rot);

};

// Moved in here because circular includes confuse me
void get_gt_info(map<string, tracking>& info, json gt_data);
void get_beacon_info(map<string, tracking>& info, json beacon_data);
