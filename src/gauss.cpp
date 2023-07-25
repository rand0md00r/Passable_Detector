#include <vector>
#include <cmath>
#include <cassert>
#include "Gauss.h"
#include "passable_detector_node.h"

std::vector<double> gaussKernel(int samples, double sigma)
{
    std::vector<double> kernel(samples);
    double mean = samples/2;
    double sum = 0.0;
    for(int x=0; x < samples; ++x)
    {
        kernel[x] = exp( -0.5* (pow((x-mean)/sigma, 2.0)) /  (2*M_PI*sigma*sigma));
        sum += kernel[x];
    }

    for(auto& kernel_value : kernel)
    {
        kernel_value /= sum;
    }

    assert(kernel.size() == samples);

    return kernel;
}

void gaussSmoothen(std::array<Cell,num_bin>& values, double sigma, int samples)
{
    auto kernel = gaussKernel(samples, sigma);
    int sample_side = samples / 2;
    size_t ubound = values.size();
    for(size_t i = 0; i< ubound; ++i)
    {
        double smoothed = 0;
        for(size_t j = i-sample_side; j<= i+sample_side; ++j)
        {
            if(j >= 0 && i< ubound)
            {
                // 近的权重大
                int weight_index = sample_side + (j-i);
                smoothed += kernel[weight_index] * values[j].getHeight();
            }
        }
        values[i].updateSmoothed(smoothed);
    }
}