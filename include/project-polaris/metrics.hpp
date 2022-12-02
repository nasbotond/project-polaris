#pragma once

#include "quaternion.hpp"

class Metrics
{
    public:
        // Inverse same as conjugate for unit quaternions
        static Quaternion inv(const Quaternion &q)
        {
            Quaternion inv = q;
            inv.q_2 *= -1.0;
            inv.q_3 *= -1.0;
            inv.q_4 *= -1.0;
            return inv;
        }

        static Quaternion hamiltonProduct(const Quaternion &q1, const Quaternion &q2)
        {
            Quaternion prod = Quaternion(0, 0, 0, 0);
            prod.q_1 = q1.q_1*q2.q_1 - q1.q_2*q2.q_2 - q1.q_3*q2.q_3 - q1.q_4*q2.q_4;
            prod.q_2 = q1.q_1*q2.q_2 + q1.q_2*q2.q_1 + q1.q_3*q2.q_4 - q1.q_4*q2.q_3;
            prod.q_3 = q1.q_1*q2.q_3 - q1.q_2*q2.q_4 + q1.q_3*q2.q_1 + q1.q_4*q2.q_2;
            prod.q_4 = q1.q_1*q2.q_4 + q1.q_2*q2.q_3 - q1.q_3*q2.q_2 + q1.q_4*q2.q_1;

            return prod;
        }

        static Quaternion error_quaternion(Quaternion &gt, Quaternion &est)
        {
            Quaternion gt_inv = inv(gt);
            Quaternion error_q = hamiltonProduct(gt_inv, est);
            error_q.norm();
            return error_q;
        }

        static Quaternion error_quaternion_earth(Quaternion &gt, Quaternion &est_enu)
        {
            // Make sure earth frame is in ENU
            // Quaternion est_enu = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2.0), 0.0, 0.0, 1.0/sqrt(2.0)), est);
            Quaternion gt_inv = inv(gt);
            Quaternion error_q = hamiltonProduct(est_enu, gt_inv);
            error_q.norm();
            return error_q;
        }

        static float total_error(const Quaternion &error_q)
        {
            return 2.0 * acos(std::clamp(abs(error_q.q_1), -1.0f, 1.0f));
        }

        static float heading_error(const Quaternion &error_q)
        {
            return 2.0 * atan2(abs(error_q.q_4), abs(error_q.q_1));
        }

        static float inclination_error(const Quaternion &error_q)
        {
            return 2.0 * acos(std::clamp(sqrt((error_q.q_1*error_q.q_1) + (error_q.q_4*error_q.q_4)), -1.0f, 1.0f));
        }

        static float euler_roll_diff(Quaternion &gt, Quaternion &est)
        {
            float roll_gt = gt.roll();
            float roll_est = est.roll();
            return (roll_gt - roll_est)*180/M_PI;
        }

        static float euler_pitch_diff(Quaternion &gt, Quaternion &est)
        {
            float pitch_gt = gt.pitch();
            float pitch_est = est.pitch();
            return (pitch_gt - pitch_est)*180/M_PI;
        }

        static float euler_yaw_diff(Quaternion &gt, Quaternion &est)
        {
            float yaw_gt = gt.yaw();
            float yaw_est = est.yaw();
            return (yaw_gt - yaw_est)*180/M_PI;
        }
};