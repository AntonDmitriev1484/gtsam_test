// gtsam_test.cpp : Defines the entry point for the application.
//

#include "gtsam_test.h"

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
}

void draw_vector(Vector3 start, Vector3 end, string color) {


	vector<double> dx = { start.x(), end.x() - start.x()};
	vector<double> dy = { start.y(), end.y() - start.y() };
	vector<double> dz = { start.z(), end.z() - start.z() };

	using namespace matplot;
	plot3(dx, dy, dz)->color(color);
}

void draw_coordinate_frame_axes(Rot3 rot_S_to_R, Vector3 loc_R) {

	double length = 0.5;

	using namespace matplot;

	hold(on);

	// Visualization bug
	
	// Almost seems like the coordinate frames aren't being translated porperly away from 0,0?

	Rot3 T = rot_S_to_R;
	Matrix33 M = rot_S_to_R.matrix();
	// I think the rotator is somehow re-scaling the vectors in transform?

	Rot3 x_axis();

	Vector3 x_S(1, 0, 0);
	Vector3 y_S(0, 1, 0);
	Vector3 z_S(0, 0, 1);

	//Vector3 x_R = T * x_S;
	//Vector3 y_R = T * y_S;
	//Vector3 z_R = T * z_S;

	Vector3 x_R = M * x_S;
	Vector3 y_R = M * y_S;
	Vector3 z_R = M * z_S;

	x_R = loc_R +x_R;
	y_R = loc_R + y_R;
	z_R = loc_R + z_R;

	draw_vector(loc_R, x_R, "red");
	draw_vector(loc_R,  y_R, "blue");
	draw_vector(loc_R, z_R, "green");

}


int main(int argc, char* argv[]) {

	string imu_filename = "/home/admitriev/Datasets/EuRoC_orbslam3_data/drone_imu/V101_imu0/data.csv";
	string gt_filename = "/home/admitriev/Datasets/EuRoC_orbslam3_data/ground_truth/V101_state_groundtruth_estimate0/data.csv";
	ifstream imu_file(imu_filename.c_str());
	ifstream gt_file(gt_filename.c_str());

	// Skip over the first line of both files
	string value;
	getline(gt_file, value);
	string s;
	getline(imu_file, s);

	int seconds = 10;
	double dt = 0.005;

	int max_plot_datapoints = int(double(seconds)/dt);

	// Find initial position from GroundTruth

	Vector3 prior_vel, start_gyro_bias, start_accel_bias;
	Point3 prior_pos;
	Rot3 prior_rot;

	vector<double> gt_xs, gt_ys, gt_zs;
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

	Values init_values; 
	int c = 0; // Correction step index
	init_values.insert(X(c), prior_pose);
	init_values.insert(V(c), prior_vel);
	init_values.insert(B(c), prior_imu_bias);

	// Define Prior Noise Model:
	noiseModel::Diagonal::shared_ptr prior_pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
	noiseModel::Diagonal::shared_ptr prior_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1); // m/s
	noiseModel::Diagonal::shared_ptr prior_bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

	// Define IMU Preintegration Noise Model:
	boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_preintegration_params = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);;
	define_IMU_factor_noise_model(imu_preintegration_params);


	// Now we can finally start building our factor graph
	NonlinearFactorGraph* graph = new NonlinearFactorGraph();
	// Initialize our (initial) prior factors, and also set up our coordinate frame
	graph->addPrior(X(c), prior_pose, prior_pose_noise_model);
	graph->addPrior(V(c), prior_vel, prior_velocity_noise_model);
	graph->addPrior(B(c), prior_imu_bias, prior_bias_noise_model);


	PreintegrationType* imu_preintegrated = new PreintegratedCombinedMeasurements(imu_preintegration_params, prior_imu_bias);

	NavState previous_state(prior_pose, prior_vel);
	imuBias::ConstantBias previous_bias = prior_imu_bias;
	NavState current_state = previous_state; // Just an object to represent our most recent state estimate
	NavState proposed_state = previous_state;
		
	int preintegration_window = 100; // Since I don't have a measurement yet, setting this manual window to define an imu factor
	unsigned long long imu_integrations = 0;


	Rot3 T_S_to_R = prior_rot.inverse(); // Start by assuming GT initial orientation is aligned with S
	// It seems this initial matrix is *very* wrong

	//draw_coordinate_frame_axes(T_S_to_R, Vector3(0,0,0));

	//draw_coordinate_frame_axes(T_S_to_R, prior_pos);

	draw_coordinate_frame_axes(T_S_to_R, Vector3(2,0,2));

	Rot3 delta_T_S_to_R;

	Vector3 vel_angular_S, accel_axial_S, vel_angular_R, accel_axial_R;
	int j = 0;
	while (parse_EuRoC_imu_line(imu_file, vel_angular_S, accel_axial_S)) {


		delta_T_S_to_R = Rot3::Rodrigues(vel_angular_S * dt);
		T_S_to_R = T_S_to_R * delta_T_S_to_R; // Now need to add this change onto the current transform

		accel_axial_R = T_S_to_R * accel_axial_S;
		accel_axial_R += Vector3(0, 0, -9.81); // R z-axis is vertically aligned, so this compensates for gravity

		vel_angular_R = T_S_to_R * vel_angular_S; // Dont think you can do this

		imu_preintegrated->integrateMeasurement(accel_axial_R, vel_angular_R, dt);
		imu_integrations++;

		// Just taking the directly predicted state of the IMU from preintegration
		// No optimization for now - just for testing data

		proposed_state = imu_preintegrated->predict(previous_state, previous_bias);

		if (imu_integrations < max_plot_datapoints) {
			Point3 p = proposed_state.position();
			xs.push_back(p.x());
			ys.push_back(p.y());
			zs.push_back(p.z());
			j++;

			//if (imu_integrations < 10) {
			//	draw_coordinate_frame_axes(T_S_to_R, proposed_state.position());
			//}
		}


	}

	cout << j << " IMU integrations plotted" << endl;

	
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