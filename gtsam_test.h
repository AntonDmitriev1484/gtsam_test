// gtsam_test.h : Include file for standard system include files,
// or project specific include files.

#pragma once

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>


// GT Sensor Information
// 
//(base)admitriev@DESKTOP - VNHNRTI:~/ Datasets / EuRoC_orbslam3_data / ground_truth / V101_state_groundtruth_estimate0$ cat sensor.yaml
//# General sensor definitions.
//sensor_type: visual - inertial
//comment : The nonlinear least - squares batch solution over the Vicon pose and IMU measurements including time offset estimation.
//
//# Sensor extrinsics wrt.the body - frame.This is the transformation of the
//# tracking prima to the body frame.
//T_BS:
//cols: 4
//rows : 4
//data : [1.0, 0.0, 0.0, 0.0,
//0.0, 1.0, 0.0, 0.0,
//0.0, 0.0, 1.0, 0.0,
//0.0, 0.0, 0.0, 1.0]


// IMU Sensor Information
//(base)admitriev@DESKTOP - VNHNRTI:~/ Datasets / EuRoC_orbslam3_data / drone_imu / V101_imu0$ cat sensor.yaml
//#Default imu sensor yaml file
//sensor_type : imu
//comment : VI - Sensor IMU(ADIS16448)
//
//# Sensor extrinsics wrt.the body - frame.
//T_BS:
//cols: 4
//rows : 4
//data : [1.0, 0.0, 0.0, 0.0,
//0.0, 1.0, 0.0, 0.0,
//0.0, 0.0, 1.0, 0.0,
//0.0, 0.0, 0.0, 1.0]
//rate_hz : 200
//
//# inertial sensor noise model parameters(static)
//gyroscope_noise_density: 1.6968e-04     # [rad / s / sqrt(Hz)](gyro "white noise")
//gyroscope_random_walk : 1.9393e-05       # [rad / s ^ 2 / sqrt(Hz)](gyro bias diffusion)
//accelerometer_noise_density : 2.0000e-3  # [m / s ^ 2 / sqrt(Hz)](accel "white noise")
//accelerometer_random_walk : 3.0000e-3    # [m / s ^ 3 / sqrt(Hz)].  (accel bias diffusion)


//(base)admitriev@DESKTOP - VNHNRTI:~/ Datasets / EuRoC_orbslam3_data / drone_imu / V101_imu0$ head - 10 data.csv
//#timestamp[ns], w_RS_S_x[rad s ^ -1], w_RS_S_y[rad s ^ -1], w_RS_S_z[rad s ^ -1], a_RS_S_x[m s ^ -2], a_RS_S_y[m s ^ -2], a_RS_S_z[m s ^ -2]
//1403715273262142976, -0.0020943951023931952, 0.017453292519943295, 0.07749261878854824, 9.0874956666666655, 0.13075533333333333, -3.6938381666666662
//1403715273267142912, -0.0013962634015954637, 0.019547687622336492, 0.07819075048934597, 9.0793234583333327, 0.122583125, -3.6938381666666662
//1403715273272143104, -0.0020943951023931952, 0.016755160819145562, 0.074700091985357306, 9.0384624166666665, 0.14709974999999997, -3.6693215416666662
//1403715273277143040, -0.0027925268031909274, 0.020943951023931952, 0.07819075048934597, 9.0711512499999998, 0.122583125, -3.67749375
//1403715273282142976, -0.0020943951023931952, 0.020943951023931952, 0.078888882190143686, 9.0793234583333327, 0.13075533333333333, -3.702010375
//1403715273287142912, 0.0, 0.021642082724729686, 0.07958701389094143, 9.0548068333333322, 0.073549874999999987, -3.6938381666666662
//1403715273292143104, -0.0013962634015954637, 0.023038346126325153, 0.074001960284559576, 9.0956678750000002, 0.13892754166666665, -3.6611493333333334
//1403715273297143040, -0.0034906585039886592, 0.022340214425527419, 0.080285145591739146, 9.0629790416666669, 0.073549874999999987, -3.6693215416666662
//1403715273302142976, -0.0041887902047863905, 0.017453292519943295, 0.07609635538695278, 9.1365289166666663, 0.14709974999999997, -3.702010375

// Also? It looks like these are starting at different timestamps???
// WHat is RS_R vs RS_S subscript mean? - I wish they EXPLAINED THEIR SUBSCRIPTS

//(base)admitriev@DESKTOP - VNHNRTI:~/ Datasets / EuRoC_orbslam3_data / ground_truth / V101_state_groundtruth_estimate0$ head - 10 data.csv
//#timestamp, p_RS_R_x[m], p_RS_R_y[m], p_RS_R_z[m], q_RS_w[], q_RS_x[], q_RS_y[], q_RS_z[], v_RS_R_x[m s ^ -1], v_RS_R_y[m s ^ -1], v_RS_R_z[m s ^ -1], b_w_RS_S_x[rad s ^ -1], b_w_RS_S_y[rad s ^ -1], b_w_RS_S_z[rad s ^ -1], b_a_RS_S_x[m s ^ -2], b_a_RS_S_y[m s ^ -2], b_a_RS_S_z[m s ^ -2]
//1403715274302142976, 0.878612, 2.142470, 0.947262, 0.060514, -0.828459, -0.058956, -0.553641, 0.009474, -0.014009, -0.002145, -0.002229, 0.020700, 0.076350, -0.012492, 0.547666, 0.069073
//1403715274307142912, 0.878658, 2.142398, 0.947252, 0.060571, -0.828433, -0.059051, -0.553664, 0.009056, -0.015066, -0.001741, -0.002229, 0.020700, 0.076350, -0.012492, 0.547666, 0.069073
//1403715274312143104, 0.878703, 2.142317, 0.947242, 0.060600, -0.828405, -0.059100, -0.553697, 0.008820, -0.017332, -0.002013, -0.002229, 0.020700, 0.076350, -0.012492, 0.547666, 0.069073
//1403715274317143040, 0.878746, 2.142225, 0.947230, 0.060577, -0.828395, -0.059095, -0.553715, 0.008513, -0.019354, -0.003039, -0.002229, 0.020700, 0.076350, -0.012492, 0.547666, 0.069073
//1403715274322142976, 0.878787, 2.142125, 0.947213, 0.060511, -0.828410, -0.059057, -0.553704, 0.008006, -0.020572, -0.003657, -0.002229, 0.020700, 0.076350, -0.012492, 0.547666, 0.069073
//1403715274327142912, 0.878826, 2.142021, 0.947195, 0.060430, -0.828430, -0.058994, -0.553690, 0.007499, -0.020926, -0.003418, -0.002229, 0.020700, 0.076350, -0.012492, 0.547666, 0.069073
//1403715274332143104, 0.878862, 2.141919, 0.947181, 0.060351, -0.828434, -0.058938, -0.553697, 0.006937, -0.019963, -0.002471, -0.002229, 0.020700, 0.076350, -0.012492, 0.547666, 0.069073
//1403715274337143040, 0.878896, 2.141825, 0.947171, 0.060292, -0.828421, -0.058910, -0.553727, 0.006449, -0.017784, -0.001462, -0.002229, 0.020700, 0.076350, -0.012492, 0.547666, 0.069073
//1403715274342142976, 0.878927, 2.141743, 0.947165, 0.060273, -0.828396, -0.058910, -0.553766, 0.006253, -0.015039, -0.000948, -0.002229, 0.020700, 0.076350, -0.012492, 0.547666, 0.069073


//(base)admitriev@DESKTOP - VNHNRTI:~/ Datasets / EuRoC / V101 / mav0 / vicon0$ cat sensor.yaml
//# General sensor definitions.
//sensor_type: pose
//comment : Pose measurement from a Vicon system.
//
//# Sensor extrinsics wrt.the body - frame.This is the transformation of the
//# vicon body origin to the body frame.
//T_BS:
//cols: 4
//rows : 4
//data : [0.33638, -0.01749, 0.94156, 0.06901,
//-0.02078, -0.99972, -0.01114, -0.02781,
//0.94150, -0.01582, -0.33665, -0.12395,
//0.0, 0.0, 0.0, 1.0]



	//Rot3 T_PoseSensor_to_Body(0.33638, -0.01749, 0.94156, 
	//	-0.02078, -0.99972, -0.01114, 
	//	0.94150, -0.01582, -0.33665);
