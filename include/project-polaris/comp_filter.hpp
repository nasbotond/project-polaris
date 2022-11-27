#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <numeric>
#include <vector>
#include <math.h>
#include "quaternion.hpp"
#include "vec3.hpp"
#define PI 3.14159265

// #define deltat 0.0035f //0.0035f
#define gyroMeasError 3.14159265358979 * (5.0/180.0)
// #define gain 0.02 //sqrt(3.0/4.0) * gyroMeasError

class ComplementaryFilter
{
    private:

    public:
        ComplementaryFilter(float deltat, float gain) : deltat(deltat), gain(gain), q(Quaternion(1.0, 0, 0, 0)) {}
        ~ComplementaryFilter() {}

        // Main functions
        void updateFilter(const Vec3 &w, const Vec3 &a, const Vec3 &m);
        void updateFilter(const Vec3 &w, const Vec3 &a);

        // Helper functions
        float sgn(float v);

        // Variables
        Quaternion q;
        float deltat;
        float gain;
};