#pragma once
#include <cstdlib>
#include <string>
#include <sstream>
#include <numeric>
#include <math.h>
#include "vec3.hpp"

class Quaternion
{
    public:
        float q_1, q_2, q_3, q_4;

        Quaternion(float q_1, float q_2, float q_3, float q_4) : q_1(q_1), q_2(q_2), q_3(q_3), q_4(q_4) {}
        ~Quaternion() {}

        float roll()
        {
            return atan2(2*q_3*q_4 - 2*q_1*q_2, 2*q_1*q_1 + 2*q_4*q_4 - 1);
        }

        float pitch()
        {
            return -asin(2*q_2*q_4 + 2*q_1*q_3);
        }

        float yaw()
        {
            return atan2(2*q_2*q_3 - 2*q_1*q_4, 2*q_1*q_1 + 2*q_2*q_2 - 1);
        }

        void norm()
        {
            // return sqrt(q_1*q_1 + q_2*q_2 + q_3*q_3 + q_4*q_4);
            float norm = sqrt(q_1*q_1 + q_2*q_2 + q_3*q_3 + q_4*q_4);
            q_1 /= norm;
            q_2 /= norm;
            q_3 /= norm;
            q_4 /= norm;
        }

        static Quaternion getOrientationFromAccMag(Vec3 &a, Vec3 &m)
        {
            Vec3 z = a;
            Vec3 m_inv = Vec3(-m.x, -m.y, -m.z);
            Vec3 tmp = Vec3::cross(z, m_inv);
            Vec3 x = Vec3::cross(tmp, z);
            Vec3 y = Vec3::cross(z, x);
            x.norm();
            y.norm();
            z.norm();

            float w_sq = (1.0 + x.x + y.y + z.z) / 4.0;
            float x_sq = (1.0 + x.x - y.y - z.z) / 4.0;
            float y_sq = (1.0 - x.x + y.y - z.z) / 4.0;
            float z_sq = (1.0 - x.x - y.y + z.z) / 4.0;

            Quaternion result = Quaternion(0, 0, 0, 0);
            result.q_1 = sqrt(w_sq);
            result.q_2 = (y.z - z.y)/abs(y.z - z.y) * sqrt(x_sq);
            result.q_3 = (z.x - x.z)/abs(z.x - x.z) * sqrt(y_sq);
            result.q_4 = (x.y - y.x)/abs(x.y - y.x) * sqrt(z_sq);

            return result;
        }
};