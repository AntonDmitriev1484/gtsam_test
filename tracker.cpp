#include "tracker.h"

Tracker::Tracker(const string& id,
            const Pose3& T_imu_body,
            const SharedNoiseModel& GT_noise_model,
            const SharedNoiseModel& UWB_noise_model,
            const SharedNoiseModel& FakePrior_noise_model,
            const SharedNoiseModel& Velocity_noise_model,
            const SharedNoiseModel& Bias_noise_model,
            const imuBias::ConstantBias prior_imu_bias,
            const double delta_t,
            const string& debug_dir) {

    // TODO: this->prior_velocity <= make this initializable to non-zero after post_process
    this->prior_imu_bias = prior_imu_bias;

    // Initialize noise models
    this->GT_noise_model = GT_noise_model;
    this->UWB_noise_model = UWB_noise_model;
    this->FakePrior_noise_model = FakePrior_noise_model;
    this->Velocity_noise_model = Velocity_noise_model;
    this->Bias_noise_model = Bias_noise_model;

    // Instantiate graph
    this->graph = new NonlinearFactorGraph();

    // Values initialized in class
    // Tracking initialized in class
    
    // Instantiate iSAM
	ISAM2Params isam_params;
	// Must be some way to set up a verbose option.
	isam_params.factorization = ISAM2Params::QR;
	isam_params.relinearizeThreshold = 0.01;
	isam_params.relinearizeSkip = 1; // More informed optimization at the cost of more computing.
	isam_params.enableDetailedResults = true;
	ISAM2DoglegParams dogleg;
	isam_params.optimizationParams = dogleg;
	this->isam = new ISAM2(isam_params);

}

void Tracker::init_anchor(){

}

void Tracker::init(json sensor_stream) {

    // Find first SLAM pose, and use it to set GT prior.

	int pose_num = 0;
	int mes_start = 0;
    Pose3 find_first_gt_pose;
    int mes_idx = 0;
    double start_timestamp = 0;

    for (json mes : sensor_stream) {
        if (mes["type"] == "slam_pose") {
            start_timestamp = (double) mes["t"];
            mes_start = mes_idx;
            Matrix44 gt_pose_world;
            string usrname;
            get_pose_matrix(mes, usrname, gt_pose_world);

            //Pose3 gt_pose = slam_to_world * gt_pose_slam;
            Pose3 asdf(gt_pose_world);
            Pose3 gt_pose = asdf * T_imu_body.inverse();

            find_first_gt_pose = gt_pose;
            break;
        }
        mes_idx ++;
    }

    // <<< Problem behind start spaghetti is here >>>
    Pose3 start_pose = find_first_gt_pose;
    Vector3 prior_velocity(-0.3, -0.8, 0); // I'm assuming this should be in the world frame?

    vals.insert(X(track.Ix), start_pose);
    vals.insert(V(track.Iv), prior_velocity);
    vals.insert(B(track.Ib), prior_imu_bias);

    graph->addPrior(X(track.Ix), start_pose, GT_noise_model);
    graph->addPrior(V(track.Iv), prior_velocity, Velocity_noise_model);
    graph->addPrior(B(track.Ib), prior_imu_bias, Bias_noise_model);

    track.est_poses.push_back(start_pose); // We'll take the estimate out of values and put it here.
    track.est_timestamps.push_back(start_timestamp);
    track.est_velocities.push_back(prior_velocity);
    track.constant_bias = prior_imu_bias;

    // Once priors have been inserted into graph and vals,
    // initialize isam with these estimates.
    isam->update(*graph, vals);
	graph->resize(0);
	vals.clear();
}