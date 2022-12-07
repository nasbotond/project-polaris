#pragma once

#include <iostream>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <numeric>
#include <vector>
#include <math.h>
#include <algorithm>
#include "quaternion.hpp"
#include "vec3.hpp"

class NaiveFilter
{
    private:

    public:
        NaiveFilter(float deltat, float gain) : deltat(deltat), gain(gain), q(Quaternion(1.0, 0, 0, 0)) {}
        ~NaiveFilter() {}

        // Main functions
        void updateFilter(const Vec3 w, const Vec3 a, const Vec3 m);
        void updateFilter(const Vec3 w, const Vec3 a);

        // Helper functions
        float sgn(float v);
        void setInitialState(Quaternion &initial);

        // Variables
        Quaternion q;
        float deltat;
        float gain;
};