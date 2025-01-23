#include "gtsam_test.h"
#include "data_loader.h"
#include "utils.h"
#include "cmath"

using namespace gtsam;
using namespace std;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)


void define_prior_noise_model() {

}
void define_IMU_factor_noise_model(boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params) {

	// TODO: Start by looking here if something is wrong
	double GYRO_BIAS = 1.9393e-05;
	double GYRO_NOISE = 1.6968e-04; // NIs this the same as noise sigma? // Shouldn't I multiply this by sqrt(sample rate) then
	double ACCEL_BIAS = 3.0000e-3;
	double ACCEL_NOISE = 2.0000e-3;

	Matrix33 accel_noise_cov = I_3x3 * pow(ACCEL_NOISE, 2);
	Matrix33 gyro_noise_cov = I_3x3 * pow(GYRO_NOISE, 2);
	Matrix33 noise_integration_cov = I_3x3 * 1e-8;  // error committed in integrating position from velocities

	Matrix33 accel_bias_cov = I_3x3 * pow(ACCEL_BIAS, 2);
	Matrix33 gyro_bias_cov = I_3x3 * pow(GYRO_BIAS, 2);
	Matrix66 initial_bias_cov = I_6x6 * 1e-5; // Why 6x6???

	// Initialize our first and only IMU factor
	// by adding our prior data points to be preintegrated

	// Need to create a parameters object to define an IMU factor that preintegrates
	// These parameters define the uncertainty model for this factor

	imu_preintegration_params->accelerometerCovariance = accel_noise_cov;
	imu_preintegration_params->integrationCovariance = noise_integration_cov;
	imu_preintegration_params->gyroscopeCovariance = gyro_noise_cov;
	imu_preintegration_params->biasAccCovariance = accel_bias_cov;
	imu_preintegration_params->biasOmegaCovariance = gyro_bias_cov;
	imu_preintegration_params->biasAccOmegaInt = initial_bias_cov;

	Vector3 imu_pose_in_body_frame = Vector3(0, 0, 0);
	Rot3 imu_rot_in_body_frame = Rot3(0, 0, 0, 0);

	imu_preintegration_params->body_P_sensor = Pose3(imu_rot_in_body_frame, imu_pose_in_body_frame);
}
void run_euroc() {
	string imu_filename = "/home/admitriev/Datasets/EuRoC_orbslam3_data/drone_imu/V101_imu0/data_gt_tstp_aligned.csv";
	string gt_filename = "/home/admitriev/Datasets/EuRoC_orbslam3_data/ground_truth/V101_state_groundtruth_estimate0/data.csv";


	ifstream imu_file(imu_filename.c_str());
	ifstream gt_file(gt_filename.c_str());

	if (!imu_file.is_open()) {
		std::cerr << "Error: Could not open IMU file: " << imu_filename << std::endl;
	}

	// Skip over the first line of both files
	string value;
	getline(gt_file, value);
	string s;
	getline(imu_file, s);

	int seconds = 30;
	double dt = 0.005;

	int max_plot_datapoints = int(double(seconds)/dt);

	// Find initial position from GroundTruth

	Vector3 prior_vel, start_gyro_bias, start_accel_bias;
	Point3 prior_pos;
	Rot3 prior_rot;

	vector<double> gt_xs, gt_ys, gt_zs;
	vector<Pose3> gps_poses;
	Point3 position;
	Rot3 r;
	Vector3 vel, gb, ab;
	int i = 0;
	while (parse_EuRoC_gt_line(gt_file, position, r, vel, gb, ab)) {
		if (i == 0) {
			prior_pos = position;
			prior_rot = r;
			prior_vel = vel;
			start_gyro_bias = gb;
			start_accel_bias = ab;
		}
		if (i < max_plot_datapoints) {
			gt_xs.push_back(position.x());
			gt_ys.push_back(position.y());
			gt_zs.push_back(position.z());
			Pose3 p(r, position);
			gps_poses.push_back(p);
			i++;
		}
	}
	cout << i << " GT points " << endl;

	//parse_EuRoC_gt_line(gt_file, prior_pos, prior_rot, prior_vel, start_gyro_bias, start_accel_bias);
	Pose3 prior_pose(prior_rot, prior_pos);

	vector<double> xs, ys, zs;
	xs.push_back(prior_pos.x());
	ys.push_back(prior_pos.y());
	zs.push_back(prior_pos.z());

	imuBias::ConstantBias prior_imu_bias;

	Values values;
	int c = 0; // Correction step index
	values.insert(X(c), prior_pose);
	values.insert(V(c), prior_vel);
	values.insert(B(c), prior_imu_bias);

	// Define Prior Noise Model:
	noiseModel::Diagonal::shared_ptr prior_pose_noise_model = noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1); // m/s
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);


	// Define IMU Preintegration Noise Model:
	// Set up a preintegration where gravity points at -Z in Nav frame
	auto imu_preintegration_params = PreintegratedImuMeasurements::Params::MakeSharedU();

	double GYRO_BIAS = 1.9393e-05;
	double GYRO_NOISE = 1.6968e-04; // NIs this the same as noise sigma? // Shouldn't I multiply this by sqrt(sample rate) then
	double ACCEL_BIAS = 3.0000e-3;
	double ACCEL_NOISE = 2.0000e-3;

	Matrix33 accel_noise_cov = I_3x3 * pow(ACCEL_NOISE, 2);
	Matrix33 gyro_noise_cov = I_3x3 * pow(GYRO_NOISE, 2);
	Matrix33 noise_integration_cov = I_3x3 * 1e-8;  // error committed in integrating position from velocities

	Matrix33 accel_bias_cov = I_3x3 * pow(ACCEL_BIAS, 2);
	Matrix33 gyro_bias_cov = I_3x3 * pow(GYRO_BIAS, 2);
	Matrix66 initial_bias_cov = I_6x6 * 1e-5; // Why 6x6???

	imu_preintegration_params->accelerometerCovariance = accel_noise_cov;
	imu_preintegration_params->integrationCovariance = noise_integration_cov;
	imu_preintegration_params->gyroscopeCovariance = gyro_noise_cov;

	// Specify the pose of the sensor in the body frame
	imu_preintegration_params->body_P_sensor = Pose3(I_4x4);


	// Now we can finally start building our factor graph
	NonlinearFactorGraph* graph = new NonlinearFactorGraph();
	// Initialize our (initial) prior factors, and also set up our coordinate frame
	graph->addPrior(X(c), prior_pose, prior_pose_noise_model);
	//graph->emplace_shared<PriorFactor<Pose3>>(X(c), prior_pose, prior_pose_noise_model)
	// as far as I understand these are the same thing
	graph->addPrior(V(c), prior_vel, prior_velocity_noise_model);
	graph->addPrior(B(c), prior_imu_bias, prior_bias_noise_model);


	ISAM2Params isam_params;
	isam_params.factorization = ISAM2Params::CHOLESKY;
	isam_params.relinearizeSkip = 10;
	ISAM2* isam = new ISAM2(isam_params);


	shared_ptr<PreintegratedImuMeasurements> imu_preintegrated = std::make_shared<PreintegratedImuMeasurements>(imu_preintegration_params, prior_imu_bias);


	NavState previous_state(prior_pose, prior_vel);
	imuBias::ConstantBias previous_bias = prior_imu_bias;

	int preintegration_window = 100; // Since I don't have a measurement yet, setting this manual window to define an imu factor
	unsigned long long imu_integrations = 0;

	Rot3 T_R_to_S = prior_rot;
	Rot3 T_S_to_R = T_R_to_S.inverse(); // This is a reasonable strategy

	double length = 0.5;
	using namespace matplot;
	hold(on);
	Vector3 loc_R = prior_pos;

	//draw_coordinate_frame_axes(Rot3::Identity(), prior_pos); // Draw Reference frame

	Matrix33 basis_R = I_3x3;
	Matrix33 basis_S = T_R_to_S.matrix() * basis_R;

	draw_basis(basis_R, loc_R, true);
	draw_basis(T_S_to_R.matrix() * basis_S, loc_R, false);

	Rot3 delta_T_S_to_R;

	Vector3 vel_angular_S, accel_linear_S, vel_angular_R, accel_linear_R, adjusted_accel_linear;

	Vector3 vel_linear_R = prior_vel;
	Vector3 pos_linear_R = prior_pos;
	Vector3 vel_linear_R_next, pos_linear_R_next;

	Pose3 current_pose;
	Vector3 current_vel;
	auto current_bias = imuBias::ConstantBias();

	Rot3 T_S_to_R_next;

	// Nonlinearity comes from measurement function linear or nonlinear -> noise in measurement, ex. RSSI is nonlinear
	// Ex. TOF on UWB leads to a linear relationship
	// rather than motion

	// Currently getting my result because the GPS measurements are weighted so highly i.e. very low noise
	// Factor graph might be overkill -> only necessary for loop closure.
	// Loop closure done based on visual features only.

	// Enguang offered to collaborate but I think this project is cooked.
		// Algorithms for IoT instructor may be able to help.

	// Enguang HAS had to manually adjust for acceleration before.

	// So KITTI dataset example he gave uses an HTM as T_w_imu
	NavState prev_imu_state(prior_rot, prior_pos, prior_vel);
	NavState imu_state = prev_imu_state;

	while (parse_EuRoC_imu_line(imu_file, vel_angular_S, accel_linear_S)) {
		//accel_linear_S = accel_linear_S - Vector3(0, 0, 9.81);
		// Have had to compensate for EuRoC
		// Coordinate frame transform is necessary before integration

		//// This, looks correct
		//delta_T_S_to_R = Rot3::Expmap(vel_angular_S * dt);
		//T_S_to_R_next = T_S_to_R * delta_T_S_to_R;
		//
		//accel_linear_R = T_S_to_R * accel_linear_S;
		//accel_linear_R = accel_linear_R + Vector3(0, 0, -9.81);

		//vel_linear_R_next = vel_linear_R + accel_linear_R * dt;
		//pos_linear_R_next = pos_linear_R + vel_linear_R * dt + (0.5) * accel_linear_R * pow(dt, 2);

		//vel_linear_R = vel_linear_R_next;
		//pos_linear_R = pos_linear_R_next;
		//T_S_to_R = T_S_to_R_next;

		////vel_angular_R = delta_T_S_to_R * vel_angular_S;
		////vel_angular_R = delta_T_S_to_R.rpy();


		imu_preintegrated->integrateMeasurement(accel_linear_S, vel_angular_S, dt);
		imu_integrations++;

		if (imu_integrations < max_plot_datapoints) {

			if (imu_integrations % preintegration_window == 0) {
				c++;

				auto cBiasKey = B(c);
				auto cPoseKey = X(c);
				auto cVelKey = V(c);

				graph->emplace_shared<ImuFactor>(X(c - 1), V(c - 1), cPoseKey, cVelKey, B(c-1), *imu_preintegrated);

				graph->emplace_shared<BetweenFactor<imuBias::ConstantBias>>(B(c-1), cBiasKey,
					imuBias::ConstantBias(),
					prior_bias_noise_model); // No idea if this method is correct

				// IMU prediction as graph node
				imu_state = imu_preintegrated->predict(prev_imu_state, imuBias::ConstantBias());
				values.insert(X(c), imu_state.pose());
				values.insert(V(c), imu_state.velocity());
				values.insert(B(c), current_bias);

				// Or create GPS factor as graph node
				//Pose3 gps_pose = gps_poses[imu_integrations];
				//auto isotropic_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
				//auto noise_model_gps = isotropic_noise;
				//graph->emplace_shared<PriorFactor<Pose3>>(cPoseKey, gps_pose, noise_model_gps);


				////gtsam::NavState p()
				////imu_preintegrated->predict(gps_poses[imu_integrations-preintegration_window]);

				//values.insert(X(c), gps_pose);
				//values.insert(V(c), current_vel);
				//values.insert(B(c), current_bias);


				// Update solver:
				isam->update(*graph, values);
				Values result = isam->calculateEstimate();

				values.clear(); // Don't understand why you need to clear values? I thought thats what the numbered keys are for....
				imu_preintegrated->resetIntegration();



				current_pose = result.at<Pose3>(cPoseKey);
				current_vel = result.at<Vector3>(cVelKey);
				current_bias = result.at<imuBias::ConstantBias>(cBiasKey);

				prev_imu_state = imu_state;
				imu_state = NavState(current_pose, current_vel);



				//Matrix33 basis_Rt = T_R_to_S.matrix() * T_S_to_R.matrix(); // As we progress, basis_Rt will get further from I_3x3. This will show us how much T_S_to_R has drifted
				//loc_R = Vector3(gt_xs[imu_integrations], gt_ys[imu_integrations], gt_zs[imu_integrations]);
				//draw_basis(basis_R, loc_R, true);
				//draw_basis(basis_Rt, loc_R, false);

				xs.push_back(current_pose.x());
				ys.push_back(current_pose.y());
				zs.push_back(current_pose.z());
			}
		}

		//previous_state = proposed_state;


	}


	using namespace matplot;
	hold(on);
	scatter3(gt_xs, gt_ys, gt_zs)->color("g");
	hold(on);
	scatter3(xs, ys, zs)->color("r");

	xlabel("X (m)");
	ylabel("Y (m)");
	zlabel("Z (m)");
	xlim({ -2.5,2.5 });
	ylim({ -2.5,2.5 });
	zlim({ -2.5,2.5 });

	show();
}


int run_cappella() {
	string filename = "/home/admitriev/Datasets/cappella_data/set_1/bigtest-1floor.json";


	ifstream fs(filename);
	if (!fs.is_open()) {
		std::cerr << "Failed to open the file." << std::endl;
	}

	json sensor_stream = json::parse(fs);

	map<string, user_info> info;
	get_info(sensor_stream, info);

	// Data is collected with Z as the up-axis, adjust data for Y to be on the up-axis
	Matrix44 vis_rotation = Matrix::Zero(4, 4);
	vis_rotation(0, 0) = 1;
	vis_rotation(2, 1) = 1;
	vis_rotation(1, 2) = 1;
	vis_rotation(3, 3) = 1;

	for (json mes : sensor_stream) {

		string measurement_type = mes["type"];
		chrono::system_clock::time_point tp = iso_string_to_time(mes["timestamp"]);
		unsigned long timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();

		if (measurement_type == "vio") {

			Matrix44 HTM_L_G;
			string user;
			get_pose_matrix(mes, user, HTM_L_G);

			user_info& u = info.at(user);

			u.last_HTM_L_G = HTM_L_G;

			Matrix44 pose_matrix_U = vis_rotation * u.last_HTM_G_U * HTM_L_G;
			Pose3 pose_U(pose_matrix_U);
			u.vio_poses.push_back(pose_U);

		}
		else if (measurement_type == "uwb") {

			double range;
			string src_user, dst_user;
			get_UWB(mes, src_user, dst_user, range);

		}
		else if (measurement_type == "gt") {

			vector<Matrix44> HTM_L_U_per_user;
			vector<string> users;
			get_GT(mes, users, HTM_L_U_per_user);

			for (int i = 0; i < users.size(); i++) {
				user_info& u = info.at(users[i]);
				Matrix44 pose_matrix_U = vis_rotation * HTM_L_U_per_user[i];
				Pose3 Pose_U(pose_matrix_U);
				u.gt_poses.push_back(Pose_U);
			}
		}
	}


	using namespace matplot;
	for (const auto& [user_name, user_info] : info) {

		auto fig = figure();
		fig->name(user_name + " trajectory");
		title(user_name);

		hold(on);

		vector<float> xs;
		vector<float> ys;
		vector<float> zs;
		for (Pose3 pose : user_info.vio_poses) {
			xs.push_back(pose.x());
			ys.push_back(pose.y());
			zs.push_back(pose.z());
		}
		plot3(xs, ys, zs)->color("r");

		//scatter3(xs, ys, zs)->color("r");
		// TODO figure out how to plot as a continuous line instead of scatterplot

		hold(on);

		vector<double> gt_xs;
		vector<double> gt_ys;
		vector<double> gt_zs;
		for (Pose3 pose : user_info.gt_poses) {
			gt_xs.push_back(pose.x());
			gt_ys.push_back(pose.y());
			gt_zs.push_back(pose.z());
		}
		scatter3(gt_xs, gt_ys, gt_zs)->color("g");

		xlabel("X (m)");
		ylabel("Z (m)");  // Switch the label to match the upward axis
		zlabel("Y (m)");

	}

	show();


	return 0;
}

int main(int argc, char* argv[]) {

	// run_euroc();
	run_cappella();

	return 0;
}