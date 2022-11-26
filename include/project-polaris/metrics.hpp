#include "quaternion.hpp"

namespace Metrics
{
    // Inverse same as conjugate for unit quaternions
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

    Quaternion error_quaternion_earth(Quaternion &gt, Quaternion &est)
    {
        Quaternion gt_inv = inv(gt);
        Quaternion error_q = hamiltonProduct(est, gt_inv);
        error_q.norm();
        return error_q;
    }

    float total_error(const Quaternion &error_q)
    {
        float x = abs(error_q.q_1);
        if (x<=-1.0) 
        {
            return 2.0 * M_PI;
        } 
        else if (x>=1.0)
        {
            return 0;
        } 
        else 
        {
            return 2.0 * acos(x);
        }
    }

    float heading_error(const Quaternion &error_q)
    {
        return 2.0f * atan2(abs(error_q.q_4), abs(error_q.q_1));
    }

    float inclination_error(const Quaternion &error_q)
    {
        float x = sqrt((error_q.q_1*error_q.q_1) + (error_q.q_4*error_q.q_4));
        if (x<=-1.0) 
        {
            return 2 * M_PI;
        } 
        else if (x>=1.0)
        {
            return 0;
        } 
        else 
        {
            return 2.0 * acos(x);
        }
    }

    float euler_roll_diff(Quaternion &gt, Quaternion &est)
    {
        float roll_gt = gt.roll();
        float roll_est = est.roll();
        return (roll_gt - roll_est)*180/M_PI;
    }

    float euler_pitch_diff(Quaternion &gt, Quaternion &est)
    {
        float pitch_gt = gt.pitch();
        float pitch_est = est.pitch();
        return (pitch_gt - pitch_est)*180/M_PI;
    }

    float euler_yaw_diff(Quaternion &gt, Quaternion &est)
    {
        float yaw_gt = gt.yaw();
        float yaw_est = est.yaw();
        return (yaw_gt - yaw_est)*180/M_PI;
    }
    
    void RMSE()
    {

    }
}