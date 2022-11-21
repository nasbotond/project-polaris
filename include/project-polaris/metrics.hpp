#include "quaternion.hpp"

namespace Metrics
{
    Quaternion inv(const Quaternion &q)
    {
        Quaternion inv = q;
        inv.q_2 *= -1.0f;
        inv.q_3 *= -1.0f;
        inv.q_4 *= -1.0f;
        return inv;
    }

    Quaternion hamiltonProduct(const Quaternion &q1, const Quaternion &q2)
    {
        Quaternion prod = Quaternion(0, 0, 0, 0);
        prod.q_1 = q1.q_1*q2.q_1 - q1.q_2*q2.q_2 - q1.q_3*q2.q_3 - q1.q_4*q2.q_4;
        prod.q_2 = q1.q_1*q2.q_2 + q1.q_2*q2.q_1 + q1.q_3*q2.q_4 - q1.q_4*q2.q_3;
        prod.q_3 = q1.q_1*q2.q_3 - q1.q_2*q2.q_4 + q1.q_3*q2.q_1 + q1.q_4*q2.q_2;
        prod.q_4 = q1.q_1*q2.q_4 + q1.q_2*q2.q_3 - q1.q_3*q2.q_2 + q1.q_4*q2.q_1;

        return prod;
    }

    Quaternion error_quaternion(Quaternion &gt, Quaternion &est)
    {
        Quaternion gt_inv = inv(gt);
        Quaternion error_q = hamiltonProduct(gt_inv, est);
        error_q.norm();
        return error_q;
    }

    float angular_dist(const Quaternion &error_q)
    {
        return 2.0f*acos(abs(error_q.q_1));
    }

    void RMSE()
    {

    }
}