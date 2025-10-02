// #pragma once
// #include "gtsam_test.h"
// #include "nlohmann/json.hpp"
// #include "data_tools.h"
// #include "utils.h"
// #include "cmath"
// #include <regex>
#include "tracker.h"

using PreintegrationType = gtsam::PreintegrationBase;
using PreintegrationParams = gtsam::PreintegratedCombinedMeasurements::Params;
// using namespace gtsam;
// using namespace std;
using json = nlohmann::json;


class CentralTracker {
public:

    bool use_smoother;
    ISAM2* isam;
    IncrementalFixedLagSmoother* smoother; // I didn't even know 'er
    FixedLagSmoother::KeyTimestampMap key_timestamps;

    NonlinearFactorGraph* graph;
    Values vals;

    string debug_dir;

    std::map<string, Tracker> users;

    std::map<string, tracking> anchors;
    void init_anchors(json anchor_json);
    void init_anchor(string id);

    
    //Debug

    CentralTracker(
        const bool use_smoother,
        const double smoother_lag,
        const string data_dir,
        const string out_dir,
        double uwb_synth_stdev,
        json transform_json,
        json anchor_json
        );

    void exec_iSAM(NavState& proposed, double mes_timestamp, 
        string msg="", bool print=false);

    void exec_smoother(NavState& proposed, double mes_timestamp, 
        string msg="", bool print=false);

};

// Moved in here because circular includes confuse me
void get_beacon_info(map<string, tracking>& info, json beacon_data);
std::shared_ptr<PreintegratedCombinedMeasurements::Params> get_imu_preintegration_params(int ASCALE, int GSCALE);