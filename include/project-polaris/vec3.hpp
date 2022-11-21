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
            // return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
            float norm = sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
            this->x /= norm;
            this->y /= norm;
            this->z /= norm;
        }
};