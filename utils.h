#pragma once

#include "matplot.h"
#include "gtsam_test.h"

using namespace gtsam;
using namespace std;
using namespace matplot;


void draw_vector(Vector3 start, Vector3 end, string color);
void draw_coordinate_frame_axes(Rot3 rot_S_to_R, Vector3 loc_R);
void draw_basis(Matrix33 basis, Vector3 loc, bool as_reference_frame);