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

class MadgwickFilter
{
    private:

    public:

        MadgwickFilter(float deltat, float beta, float zeta) : deltat(deltat), beta(beta), zeta(zeta), q(Quaternion(1.0, 0.0, 0.0, 0.0)) {}
        ~MadgwickFilter() {}

        // Main functions
        void updateMARGFilter(Vec3 w, Vec3 a, Vec3 m);
        void updateIMUFilter(Vec3 w, Vec3 a);

        // Helper functions
        void setInitialState(Quaternion &initial);

        // Variables
        Quaternion q;
        float b_x = 1.0, b_z = 0;
        float w_bx = 0, w_by = 0, w_bz = 0;
        float deltat;
        float beta;
        float zeta;
};