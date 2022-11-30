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
        Quaternion() {}
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
            float norm = sqrt(q_1*q_1 + q_2*q_2 + q_3*q_3 + q_4*q_4);
            q_1 /= norm;
            q_2 /= norm;
            q_3 /= norm;
            q_4 /= norm;
        }

        static Quaternion getOrientationFromAccMag(Vec3 a, Vec3 m)
        {
            Vec3 z = a;
            Vec3 m_inv = Vec3(-m.x, -m.y, -m.z);
            Vec3 tmp = Vec3::cross(z, m);
            Vec3 x = Vec3::cross(tmp, z);
            Vec3 y = Vec3::cross(z, x);
            x.norm();
            y.norm();
            z.norm();

            // Sarabandi Thomas Method

            float threshold = 0.0;

            float w_sq;
            float x_sq;
            float y_sq;
            float z_sq;

            if((x.x + y.y + z.z) > threshold)
            {
                w_sq = (1.0 + x.x + y.y + z.z) / 4.0;
            }
            else
            {
                w_sq = ((z.y - y.z)*(z.y - y.z) + (x.z - z.x)*(x.z - z.x) + (y.x - x.y)*(y.x - x.y)) / (4.0 * (3.0 - x.x - y.y - z.z));
            }

            if((x.x - y.y - z.z) > threshold)
            {
                x_sq = (1.0 + x.x - y.y - z.z) / 4.0;
            }
            else
            {
                x_sq = ((z.y - y.z)*(z.y - y.z) + (x.y + y.x)*(x.y + y.x) + (x.z + z.x)*(x.z + z.x)) / (4.0 * (3.0 - x.x + y.y + z.z));
            }

            if((-x.x + y.y - z.z) > threshold)
            {
                y_sq = (1.0 - x.x + y.y - z.z) / 4.0;
            }
            else
            {
                y_sq = ((x.z - z.x)*(x.z - z.x) + (x.y + y.x)*(x.y + y.x) + (y.z + z.y)*(y.z + z.y)) / (4.0 * (3.0 + x.x - y.y + z.z));
            }

            if((-x.x - y.y + z.z) > threshold)
            {
                z_sq = (1.0 - x.x - y.y + z.z) / 4.0;
            }
            else
            {
                z_sq = ((y.x - x.y)*(y.x - x.y) + (z.x + x.z)*(z.x + x.z) + (z.y + y.z)*(z.y + y.z)) / (4.0 * (3.0 + x.x + y.y - z.z));
            }


            Quaternion result = Quaternion(0, 0, 0, 0);

            result.q_1 = sqrt(w_sq);
            result.q_2 = (z.y - y.z)/abs(z.y - y.z) * sqrt(x_sq);
            result.q_3 = (x.z - z.x)/abs(x.z - z.x) * sqrt(y_sq);
            result.q_4 = (y.x - x.y)/abs(y.x - x.y) * sqrt(z_sq);

            return result;
        }

        bool isNaN()
        {
            return (isnan(q_1) || isnan(q_2) || isnan(q_3) || isnan(q_4));
        }
};