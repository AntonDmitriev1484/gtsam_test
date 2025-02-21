#pragma once

#include "matplot.h"
#include "gtsam_test.h"
#include "data_tools.h"

#include "cmath"
#include <regex>
#include <functional>

using namespace gtsam;
using namespace matplot;
using namespace std;


void draw_vector(Vector3 start, Vector3 end, string color);
void draw_coordinate_frame_axes(Rot3 rot_S_to_R, Vector3 loc_R);
void draw_basis(Matrix33 basis, Vector3 loc, bool as_reference_frame);
void draw_trajectory(vector<Pose3> trajectory, string color);
void draw_trajectory_with_orientation(vector<Pose3> trajectory, string color);
void draw_points(vector<Pose3> points, string color);

void unpack_results_and_plot(Values results, const function<Key(string, int)>& MK, map<string, tracking_info> info, vector<string> show_list);
void unpack_results(Values results, const function<Key(string, int)>& MK, map<string, tracking_info>& info);
void clear_results(map<string, tracking_info>& info);

void gen_single_vio_trajectory_scenario(int N_poses, vector<Pose3>& true_trajectory, vector<Pose3>& gt_points, vector<Pose3>& vio_trajectory);
void gen_multi_user_vio_trajectory_scenario(int N_poses, vector<vector<Pose3>>& all_true_trajectory, vector<vector<Pose3>>& all_gt_points, vector<vector<Pose3>>& all_vio_trajectory);

void LM_lambda_search(NonlinearFactorGraph* graph, Values vals, vector<Pose3> vio_trajectory, vector<Pose3> gt_points, vector<Pose3> true_trajectory);
void LM_lambda_search_multiuser_graph(NonlinearFactorGraph* graph, Values vals, vector<vector<Pose3>> vio_trajectory, vector<vector<Pose3>> gt_points, vector<vector<Pose3>> true_trajectory);


#define PLOT_FOR_USERS(INFO, SHOW_LIST) {						           \
    for (const auto& [user_name, user_info] : INFO) {                      \
        if (!user_info.is_beacon) {                                        \
            if (find(SHOW_LIST.begin(), SHOW_LIST.end(), user_name) != SHOW_LIST.end()) { \
                auto fig = figure();                                       \
                fig->name(user_name + " trajectory");                      \
                title(user_name);                                          \
                                                                           \
                hold(on);                                                  \
                draw_trajectory(user_info.vio_poses, "red");               \
                hold(on);                                                  \
                draw_points(user_info.gt_poses, "g");                      \
                hold(on);                                                  \
                draw_trajectory(user_info.est_poses, "blue");              \
                                                                           \
                xlabel("X (m)");                                           \
                ylabel("Z (m)");                                           \
                zlabel("Y (m)");                                           \
                                                                           \
            }                                                              \
        }                                                                  \
    }                                                                      \
}

#define PLOT(GT_TRAJECTORY, GT_POINTS, EST_TRAJECTORY, VIO_TRAJECTORY) {			   \
                auto fig = figure();                                       \
                fig->name("Trajectory");                      \
				title("Trajectories");                                          \
                                                                           \
                hold(on);                                                  \
                draw_trajectory(VIO_TRAJECTORY, "red");               \
                hold(on);                                                  \
				draw_points(GT_POINTS, "green");							\
				hold(on);													\
                draw_trajectory(GT_TRAJECTORY, "green");                      \
                hold(on);                                                  \
                draw_trajectory(EST_TRAJECTORY, "blue");              \
                                                                           \
                xlabel("X (m)");                                           \
                ylabel("Y (m)");                                           \
                zlabel("Z (m)");                                           \
                                                                           \
				zlim({-30,30});												\
}

#define PLOT_W_LM_PARAMS(GT_TRAJECTORY, GT_POINTS, EST_TRAJECTORY, VIO_TRAJECTORY, LAMBDA, LAMBDA_FACTOR) {			   \
                auto fig = figure();                                       \
                fig->name("Trajectory");                      \
				title("Trajectories L="+to_string(LAMBDA)+" LF="+to_string(LAMBDA_FACTOR));                                          \
                                                                           \
                hold(on);                                                  \
                draw_trajectory(VIO_TRAJECTORY, "red");               \
                hold(on);                                                  \
				draw_points(GT_POINTS, "green");							\
				hold(on);													\
                draw_trajectory(GT_TRAJECTORY, "green");                      \
                hold(on);                                                  \
                draw_trajectory(EST_TRAJECTORY, "blue");              \
                                                                           \
                xlabel("X (m)");                                           \
                ylabel("Y (m)");                                           \
                zlabel("Z (m)");                                           \
                                                                           \
				zlim({-30,30});												\
}

#define PLOT_MULTI(N_USERS, GT_TRAJECTORY, GT_POINTS, EST_TRAJECTORY, VIO_TRAJECTORY) {			   \
                auto fig = figure();                                       \
                fig->name("Trajectory");                      \
				title("Trajectory");                                     \
                for (int usr = 0; usr < N_USERS; usr++) { \
                        hold(on);                                                  \
                        draw_trajectory(VIO_TRAJECTORY[usr], "red");               \
                        hold(on);                                                  \
                        draw_points(GT_POINTS[usr], "green");							\
                        hold(on);													\
                        draw_trajectory(GT_TRAJECTORY[usr], "green");                      \
                        hold(on);                                                  \
                        draw_trajectory(EST_TRAJECTORY[usr], "blue");              \
                }                                                                 \
                xlabel("X (m)");                                           \
                ylabel("Y (m)");                                           \
                zlabel("Z (m)");                                           \
                                                                           \
				zlim({-30,30});												\
}

#define PLOT_MULTI_W_LM_PARAMS(N_USERS, GT_TRAJECTORY, GT_POINTS, EST_TRAJECTORY, VIO_TRAJECTORY, LAMBDA, LAMBDA_FACTOR) {			   \
                auto fig = figure();                                       \
                fig->name("Trajectory");                      \
				title("Trajectories L="+to_string(LAMBDA)+" LF="+to_string(LAMBDA_FACTOR));                                          \
                for (int usr = 0; usr < N_USERS; usr++) { \
                        hold(on);                                                  \
                        draw_trajectory(VIO_TRAJECTORY[usr], "red");               \
                        hold(on);                                                  \
                        draw_points(GT_POINTS[usr], "green");							\
                        hold(on);													\
                        draw_trajectory(GT_TRAJECTORY[usr], "green");                      \
                        hold(on);                                                  \
                        draw_trajectory(EST_TRAJECTORY[usr], "blue");              \
                }                                                                 \
                xlabel("X (m)");                                           \
                ylabel("Y (m)");                                           \
                zlabel("Z (m)");                                           \
                                                                           \
				zlim({-30,30});												\
}

#define PLOT_MULTI_W_DISAM_PARAMS(N_USERS, GT_TRAJECTORY, GT_POINTS, EST_TRAJECTORY, VIO_TRAJECTORY, RLT, RLS, D, NU) {			   \
                auto fig = figure();                                       \
                fig->name("Trajectory");                      \
				title("Trajectories RLT="+to_string(RLT)+" RLS="+to_string(RLS)+" Delta="+to_string(D)+ " Updates="+to_string(NU));                                          \
                for (int usr = 0; usr < N_USERS; usr++) { \
                        hold(on);                                                  \
                        draw_trajectory(VIO_TRAJECTORY[usr], "red");               \
                        hold(on);                                                  \
                        draw_points(GT_POINTS[usr], "green");							\
                        hold(on);													\
                        draw_trajectory(GT_TRAJECTORY[usr], "green");                      \
                        hold(on);                                                  \
                        draw_trajectory(EST_TRAJECTORY[usr], "blue");              \
                }                                                                 \
                xlabel("X (m)");                                           \
                ylabel("Y (m)");                                           \
                zlabel("Z (m)");                                           \
                                                                           \
				zlim({-30,30});												\
}