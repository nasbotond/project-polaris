#pragma once
#include <cstdlib>
#include <string>
#include <sstream>
#include <numeric>
#include <math.h>

class Vec3
{
    public:
        float x, y, z;

        Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
        Vec3() {}
        ~Vec3() {}

        void norm()
        {
            // return sqrt(x * x + y * y + z * z);
            float norm = sqrt(x * x + y * y + z * z);
            x /= norm;
            y /= norm;
            z /= norm;
        }
};