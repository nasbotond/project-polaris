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
            float sin_p = 2*q_2*q_4 + 2*q_1*q_3;
            if(abs(sin_p) >= 0.999)
            {
                return 0;
            }
            return atan2(2*q_3*q_4 - 2*q_1*q_2, 2*q_1*q_1 + 2*q_4*q_4 - 1); // MADG NED
        }

        float pitch()
        {
            float sin_p = 2*q_2*q_4 + 2*q_1*q_3;
            if(std::abs(sin_p) >= 0.999)
            {
                return std::copysign(M_PI/2, sin_p);
            }
            return -asin(sin_p);
        }

        float yaw()
        {
            float sin_p = 2*q_2*q_4 + 2*q_1*q_3;
            if(abs(sin_p) >= 0.999)
            {
                return std::copysign(2.0*atan2(q_2, q_1), sin_p);
            }
            return atan2(2*q_2*q_3 - 2*q_1*q_4, 2*q_1*q_1 + 2*q_2*q_2 - 1); // MADG NED
        }

        void norm()
        {
            float norm = sqrt(q_1*q_1 + q_2*q_2 + q_3*q_3 + q_4*q_4);
            q_1 /= norm;
            q_2 /= norm;
            q_3 /= norm;
            q_4 /= norm;
        }

        bool isNaN()
        {
            return (isnan(q_1) || isnan(q_2) || isnan(q_3) || isnan(q_4));
        }

        static float sgn(float v)
        {
        return (v < 0) ? -1.0 : ((v > 0) ? 1.0 : 0.0);
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
            result.q_2 = sgn(z.y - y.z) * sqrt(x_sq);
            result.q_3 = sgn(x.z - z.x) * sqrt(y_sq);
            result.q_4 = sgn(y.x - x.y) * sqrt(z_sq);

            return result;
        }
        
        static Quaternion getOrientationFromAcc(Vec3 a)
        {
            Quaternion q_a = Quaternion(1.0, 0, 0, 0);

            Vec3 acc = a;
            if((acc.x*acc.x + acc.y*acc.y + acc.z*acc.z) > 0)
            {
                acc.norm();

                float theta = atan2(acc.y, acc.z); // roll
                float phi = atan2(-acc.x, sqrt((acc.y*acc.y + acc.z*acc.z))); // pitch
                
                float psi = 0; // yaw

                q_a.q_1 = cos(0.5*theta)*cos(0.5*phi);
                q_a.q_2 = sin(0.5*theta)*cos(0.5*phi);
                q_a.q_3 = cos(0.5*theta)*sin(0.5*phi);
                q_a.q_4 = -sin(0.5*theta)*sin(0.5*phi);

                q_a.norm();
            }

            return q_a;
        }
};