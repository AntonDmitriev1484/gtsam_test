#pragma once
#include "gtsam_test.h"
#include "json.hpp"
#include "data_tools.h"
#include "utils.h"
#include "cmath"
#include <regex>

using PreintegrationType = gtsam::PreintegrationBase;
using PreintegrationParams = gtsam::PreintegratedCombinedMeasurements::Params;
using namespace gtsam;
using json = nlohmann::json;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

struct tracking {

	// Computed in get_info
	bool is_beacon;

    // Note: Unsued, just comparing to Optitrack post output
	vector<Pose3> slam_poses;
	vector<double> slam_timestamps; // Parallel array to GT poses.

	vector<Pose3> est_poses; // Estimated pose
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
    Vector3 prior_velocity;

    Pose3 T_body_to_imu;
    Pose3 T_body_to_decawave;
    SharedNoiseModel SLAM_noise_model;
    SharedNoiseModel UWB_noise_model;
    SharedNoiseModel Velocity_noise_model;
    SharedNoiseModel Bias_noise_model;

    string debug_dir, out_dir;

    double delta_t;
    string id;
    tracking track;

    map<string, tracking> anchors;
    map<int, Tracker>& other_trackers;

    double start;
    double init_newmap;
    double end;

    ofstream estimated_trajectory_fs;
    ofstream estimated_trajectory_htm_json_fs;
    ofstream slam_trajectory_fs;
    ofstream log_fs;

    double start_timestamp;
    bool start_graph = false;

    bool use_filter;
    one_euro_filter<Eigen::Array<double, 3, 1>, double> translation_filt;

    map<int, one_euro_filter<Eigen::Array<double, 1, 1>, double>> range_filt;
    map<int, deque<double>> prev_ranges;
    
    Tracker(const string id,
            map<int, Tracker>& others,
            const Pose3 T_body_to_imu,
            const Pose3 T_body_to_decawave,
            const double smoother_lag,
            const bool use_smoother,
            const bool use_filter,
            const bool use_uwb,
            const bool synth_live_slam_mode, 
            const SharedNoiseModel& SLAM_noise_model,
            const SharedNoiseModel& UWB_noise_model,
            const SharedNoiseModel& Velocity_noise_model,
            const SharedNoiseModel& Bias_noise_model,
			std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params,
            const Vector6 prior_imu_bias,
            const Vector3 prior_velocity,
            const string out_dir,
            const string debug_dir);

    void init(json sensor_stream);
    void init_anchors(json anchor_json);
    void init_anchor(string id);
    void init_state(json sensor_stream);
    
    Pose3 report_estimate(Pose3 initial, double timestamp); // take in a GTSAM pose, apply 1-euro filter, and append output to est

    void exec_iSAM(NavState& proposed, double mes_timestamp, 
        string msg="", bool print=false);
    void exec_smoother(NavState& proposed, double mes_timestamp, 
        string msg="", bool print=false);

    
    //emulator parameter, toggles between Flock and IMU only
    bool use_uwb;
    bool synth_live_slam_mode; // run an imu integration for synthetic live SLAM
    int imu_counter = 0;

	deque<json> gt_pose_buffer;
	deque<json> range_buffer;
    int imu_available;
    string slam_status;

    void processSensor(const json& mes);
    void processSLAM(const json& mes);
    void processUWB(const json& mes);
};

// Moved in here because circular includes confuse me
void get_gt_info(map<string, tracking>& info, json gt_data);
void get_beacon_info(map<string, tracking>& info, json beacon_data);
std::shared_ptr<PreintegratedCombinedMeasurements::Params> get_imu_preintegration_params(int ASCALE, int GSCALE, Pose3 T_inertial_to_world);