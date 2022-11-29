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
            float norm = sqrt(x * x + y * y + z * z);
            x /= norm;
            y /= norm;
            z /= norm;
        }

        static Vec3 cross(Vec3 a, Vec3 b)
        {
            return Vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
        }

        std::string to_string()
        {
            return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z);
        }
};