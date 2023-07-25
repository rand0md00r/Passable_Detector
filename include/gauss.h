#pragma once
#include <vector>
#include <array>
#include "passable_detector_node.h"

std::vector<double> gaussKernel(int samples, double sigma);
void gaussSmoothen(std::array<Cell,num_bin>& values, double sigma, int samples);
