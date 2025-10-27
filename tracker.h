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

    Pose3 T_body_to_imu;
    Pose3 T_body_to_decawave;
    SharedNoiseModel GT_noise_model;
    SharedNoiseModel UWB_noise_model;
    SharedNoiseModel FakePrior_noise_model;
    SharedNoiseModel Velocity_noise_model;
    SharedNoiseModel Bias_noise_model;

    string debug_dir;

    double delta_t;
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
    ofstream* estimated_trajectory_fs;
    ofstream* slam_trajectory_fs;

    double mes_start;

    vector<double> nlos_score_window;

    bool use_filter;
    one_euro_filter<Eigen::Array<double, 3, 1>, double> translation_filt;
    // OneEuroFilter<3> translation_filt;

    Tracker(const string& id,
            const Pose3 T_body_to_imu,
            const Pose3 T_body_to_decawave,
            const double delta_t,
            const double smoother_lag,
            const bool use_smoother,
            const bool use_filter,
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
    void init_state(json sensor_stream, json priors);

    Pose3 report_estimate(Pose3 initial, double timestamp); // take in a GTSAM pose, apply 1-euro filter, and append output to est

    void exec_iSAM(NavState& proposed, double mes_timestamp, 
        string msg="", bool print=false);

    void exec_smoother(NavState& proposed, double mes_timestamp, 
        string msg="", bool print=false);

    void processSLAM(const json& mes);
    void processSyntheticUWB(const json& mes, int& uwb_counter, double uwb_stdev);
    void processAssistedUWB(const json& mes, int& uwb_counter);
};

// Moved in here because circular includes confuse me
void get_gt_info(map<string, tracking>& info, json gt_data);
void get_beacon_info(map<string, tracking>& info, json beacon_data);
std::shared_ptr<PreintegratedCombinedMeasurements::Params> get_imu_preintegration_params(int ASCALE, int GSCALE);