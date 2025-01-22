// gtsam_test.cpp : Defines the entry point for the application.
//

#include "gtsam_test.h"
#include "cmath"

using namespace gtsam;
using namespace std;
// Note: Don't use matplot namespace here, as it creates ambiguity with gtsam::Vector3

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)


// This parses a single line of EuRoC GT, all parameters by reference
bool parse_EuRoC_gt_line(ifstream& gt_file, Point3& position, Rot3& rotation,
	Vector3& velocity, Vector3& gyro_bias, Vector3& accel_bias) {

	string value;
	getline(gt_file, value, ','); // Skip over timestamp
	if (gt_file.eof()) return false;

	Vector3 start_pos;
	for (int i = 0; i < 3; i++) {
		getline(gt_file, value, ',');
		start_pos(i) = stof(value.c_str());
	}
	Vector4 start_rot;
	for (int i = 0; i < 4; i++) {
		getline(gt_file, value, ',');
		start_rot(i) = stof(value.c_str());
	}
	for (int i = 0; i < 3; i++) {
		getline(gt_file, value, ',');
		velocity(i) = stof(value.c_str());
	}
	// EuRoC GT also gives you the bias in the IMU sensor
	for (int i = 0; i < 3; i++) {
		getline(gt_file, value, ',');
		gyro_bias(i) = stof(value.c_str());
	}
	for (int i = 0; i < 2; i++) {
		getline(gt_file, value, ',');
		accel_bias(i) = stof(value.c_str());
	}
	getline(gt_file, value, '\n');
	accel_bias(2) = stof(value.c_str());

	// TODO: GT sensor is rotated out of body frame according to its sensor.yaml
	// Does this mean I need to rotate all of these?

	Point3 p(start_pos(0), start_pos(1), start_pos(2));
	position = p;
	Rot3 r(start_rot(0), start_rot(1), start_rot(2), start_rot(3)); // Rotation as quaternion
	rotation = r;

	bool havemore = !gt_file.eof();
	return havemore;
}

// This parses a single line of EuRoC IMU, all parameters by reference
bool parse_EuRoC_imu_line(ifstream& imu_file, Vector3& V_angular, Vector3& A_axial) {
	string s;
	getline(imu_file, s, ','); // Skip over timestamp
	if (imu_file.eof()) return false;

	for (int j = 0; j < 3; j++) {
		getline(imu_file, s, ',');
		V_angular(j) = stof(s.c_str());
	}
	for (int j = 0; j < 2; j++) {
		getline(imu_file, s, ',');
		A_axial(j) = stof(s.c_str());
	}
	// Last value in EuRoC row is terminated with \r\n instead of a comma
	getline(imu_file, s, '\r');
	A_axial(2) = stof(s.c_str());
	return !imu_file.eof(); // Return true so long as we arent at the end of the file
}

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

void draw_vector(Vector3 start, Vector3 end, string color) {

	// Why dont they give you an option to plot a SINGLE FUCKING VECTOR

	using namespace matplot;

	hold(on);

	//Vector3 delta = end - start;
	//double length = (end - start).norm();
	//auto t = iota(0, length);
	//double start_x = start.x();
	// I guess its implied this vector is going in the x direction only, so we don't need Vector3
	//auto x_parametric = transform(iota(0, delta.x()), [start_x](auto x) {return start_x + x; });

	vector<double> dx = { start.x() , end.x() };
	vector<double> dy = { start.y(), end.y() };
	vector<double> dz = { start.z(), end.z() };

	plot3(dx, dy, dz)->color(color);

	// can also set line_width property

}

void draw_coordinate_frame_axes(Rot3 rot_S_to_R, Vector3 loc_R) {

	double length = 0.5;

	using namespace matplot;

	hold(on);

	Rot3 T = rot_S_to_R;
	Matrix33 M = rot_S_to_R.matrix();
	// I think the rotator is somehow re-scaling the vectors in transform?

	Rot3 x_axis();

	Vector3 x_S(1, 0, 0);
	Vector3 y_S(0, 1, 0);
	Vector3 z_S(0, 0, 1);

	// Draw reference coordinate frame
	//draw_vector(loc_R, loc_R + x_S, "black");
	//draw_vector(loc_R, loc_R + y_S, "black");
	//draw_vector(loc_R, loc_R + z_S, "black");

	Vector3 x_R = T * x_S;
	Vector3 y_R = T * y_S;
	Vector3 z_R = T * z_S;

	// Draw rotated coordinate frame
	draw_vector(loc_R, loc_R+x_R, "red");
	draw_vector(loc_R,  loc_R+y_R, "blue");
	draw_vector(loc_R, loc_R+z_R, "green");

}

void draw_basis(Matrix33 basis, Vector3 loc, bool as_reference_frame) {
	using namespace matplot;
	hold(on);
	double length = 0.25;
	if (as_reference_frame) {
		string color = "black";
		draw_vector(loc, loc + length*basis.col(0), color);
		draw_vector(loc, loc + length * basis.col(1), color);
		draw_vector(loc, loc + length * basis.col(2), color);

	}
	else {
		draw_vector(loc, loc + length * basis.col(0), "red");
		draw_vector(loc, loc + length * basis.col(1), "blue");
		draw_vector(loc, loc + length * basis.col(2), "green");
	}

}


int main(int argc, char* argv[]) {

	string imu_filename = "/home/admitriev/Datasets/EuRoC_orbslam3_data/drone_imu/V101_imu0/data_gt_tstp_aligned.csv";
	string gt_filename = "/home/admitriev/Datasets/EuRoC_orbslam3_data/ground_truth/V101_state_groundtruth_estimate0/data.csv";
	ifstream imu_file(imu_filename.c_str()); // Is it even opening the file
	ifstream gt_file(gt_filename.c_str());

	if (!imu_file.is_open()) {
		std::cerr << "Error: Could not open IMU file: " << imu_filename << std::endl;
		return 1; // Exit with error code
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
	noiseModel::Diagonal::shared_ptr prior_pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
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