#pragma once

#include <iostream>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <numeric>
#include <vector>
#include "quaternion.hpp"
#include "vec3.hpp"

#define PI 3.14159265

// #define deltat 0.0035f //0.0035f
// #define gyroMeasError 3.14159265358979 * (5.0/180.0)
// #define gyroMeasDrift 3.14159265358979 * (0.2/180.0)
// #define beta sqrt(3.0/4.0) * gyroMeasError
// #define zeta sqrt(3.0/4.0) * gyroMeasDrift
// #define beta 0.28
#define zeta 0.0

class MadgwickFilter
{
    private:

    public:

        MadgwickFilter(float deltat, float beta) : deltat(deltat), beta(beta), q(Quaternion(1.0, 0.0, 0.0, 0.0)) {}
        ~MadgwickFilter() {}

        // Main functions
        void updateMARGFilter(Vec3 &w, Vec3 &a, Vec3 &m);
        void updateIMUFilter(Vec3 &w, Vec3 &a);

        // Helper functions
        void setInitialState(Quaternion &initial);

        // Variables
        Quaternion q;
        float b_x = 1.0, b_z = 0;
        float w_bx = 0, w_by = 0, w_bz = 0;
        float deltat;
        float beta;
};