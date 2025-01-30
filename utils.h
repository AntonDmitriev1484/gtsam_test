#pragma once

#include "matplot.h"
#include "gtsam_test.h"
#include "data_tools.h"

#include <functional>

using namespace gtsam;
using namespace matplot;
using namespace std;


void draw_vector(Vector3 start, Vector3 end, string color);
void draw_coordinate_frame_axes(Rot3 rot_S_to_R, Vector3 loc_R);
void draw_basis(Matrix33 basis, Vector3 loc, bool as_reference_frame);
void draw_trajectory(vector<Pose3> trajectory, string color);
void draw_points(vector<Pose3> points, string color);

void unpack_results_and_plot(Values results, const function<Key(string, int)>& MK, map<string, tracking_info> info, vector<string> show_list);
void unpack_results(Values results, const function<Key(string, int)>& MK, map<string, tracking_info>& info);
void clear_results(map<string, tracking_info>& info);