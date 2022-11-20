#pragma once
#include <cstdlib>
#include <string>
#include <sstream>
#include <numeric>
#include <math.h>

class Quaternion
{
    public:
        float q_1, q_2, q_3, q_4;

        Quaternion(float q_1, float q_2, float q_3, float q_4) : q_1(q_1), q_2(q_2), q_3(q_3), q_4(q_4) {}
        ~Quaternion() {}

        float roll()
        {
            return atan2(2*this->q_3*this->q_4-2*this->q_1*this->q_2, 2*this->q_1*this->q_1 + 2*this->q_4*this->q_4-1);
        }

        float pitch()
        {
            return -asin(2*this->q_2*this->q_4+2*this->q_1*this->q_3);
        }

        float yaw()
        {
            return atan2(2*this->q_3*this->q_4-2*this->q_1*this->q_2, 2*this->q_1*this->q_1 + 2*this->q_4*this->q_4-1);
        }

        void norm()
        {
            float norm = sqrt(this->q_1*this->q_1 + this->q_2*this->q_2 + this->q_3*this->q_3 + this->q_4*this->q_4);
            this->q_1 /= norm;
            this->q_2 /= norm;
            this->q_3 /= norm;
            this->q_4 /= norm;
        }
};