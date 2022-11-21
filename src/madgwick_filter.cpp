#include "madgwick_filter.hpp"

void MadgwickFilter::updateMARGFilter(Vec3 &w, Vec3 &a, Vec3 &m)
{
    // local system variables
    float f_1, f_2, f_3, f_4, f_5, f_6;
    float J_11or24, J_12or23, J_13or22, J_41, J_42, J_43, J_44, J_51, J_52, J_14or21, J_32, J_33, J_53, J_54, J_61, J_62, J_63, J_64;
    float w_err_x, w_err_y, w_err_z;
    float h_x, h_y, h_z;

    Quaternion qDot_omega = Quaternion(0, 0, 0, 0);
    Quaternion qHatDot = Quaternion(0, 0, 0, 0);

    if((m.x == 0.0f) && (m.y == 0.0f) && (m.z == 0.0f))
    {
        updateIMUFilter(w, a);
        return;
    }

    float twom_x = 2.0f * m.x;
    float twom_y = 2.0f * m.y;
    float twom_z = 2.0f * m.z;

    // compute flux in the earth frame
    float q_1q_2 = this->q.q_1 * this->q.q_2;
    float q_1q_3 = this->q.q_1 * this->q.q_3;
    float q_1q_4 = this->q.q_1 * this->q.q_4;
    float q_3q_4 = this->q.q_3 * this->q.q_4;
    float q_2q_3 = this->q.q_2 * this->q.q_3;
    float q_2q_4 = this->q.q_2 * this->q.q_4;

    h_x = twom_x * (0.5f - this->q.q_3 * this->q.q_3 - this->q.q_4 * this->q.q_4) + twom_y * (q_2q_3 - q_1q_4) + twom_z * (q_2q_4 + q_1q_3);
    h_y = twom_x * (q_2q_3 + q_1q_4) + twom_y * (0.5f - this->q.q_2 * this->q.q_2 - this->q.q_4 * this->q.q_4) + twom_z * (q_3q_4 - q_1q_2);
    h_z = twom_x * (q_2q_4 - q_1q_3) + twom_y * (q_3q_4 + q_1q_2) + twom_z * (0.5f - this->q.q_2 * this->q.q_2 - this->q.q_3 * this->q.q_3);

    // normalise the flux vector to have only components in the x and z
    float b_x = sqrt((h_x * h_x) + (h_y * h_y));
    float b_z = h_z;

    float halfq_1 = 0.5f * this->q.q_1;
    float halfq_2 = 0.5f * this->q.q_2;
    float halfq_3 = 0.5f * this->q.q_3;
    float halfq_4 = 0.5f * this->q.q_4;
    float twoq_1 = 2.0f * this->q.q_1;
    float twoq_2 = 2.0f * this->q.q_2;
    float twoq_3 = 2.0f * this->q.q_3;
    float twoq_4 = 2.0f * this->q.q_4;
    float twob_x = 2.0f * b_x;
    float twob_z = 2.0f * b_z;
    float twob_xq_1 = 2.0f * b_x * this->q.q_1;
    float twob_xq_2 = 2.0f * b_x * this->q.q_2;
    float twob_xq_3 = 2.0f * b_x * this->q.q_3;
    float twob_xq_4 = 2.0f * b_x * this->q.q_4;
    float twob_zq_1 = 2.0f * b_z * this->q.q_1;
    float twob_zq_2 = 2.0f * b_z * this->q.q_2;
    float twob_zq_3 = 2.0f * b_z * this->q.q_3;
    float twob_zq_4 = 2.0f * b_z * this->q.q_4;

    if(!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) 
    {
        a.norm();
        m.norm();

        // compute the objective function and Jacobian
        f_1 = twoq_2 * this->q.q_4 - twoq_1 * this->q.q_3 - a.x;
        f_2 = twoq_1 * this->q.q_2 + twoq_3 * this->q.q_4 - a.y;
        f_3 = 1.0f - twoq_2 * this->q.q_2 - twoq_3 * this->q.q_3 - a.z;
        f_4 = twob_x * (0.5f - this->q.q_3 * this->q.q_3 - this->q.q_4 * this->q.q_4) + twob_z * (q_2q_4 - q_1q_3) - m.x; 
        f_5 = twob_x * (this->q.q_2 * this->q.q_3 - this->q.q_1 * this->q.q_4) + twob_z * (this->q.q_1 * this->q.q_2 + this->q.q_3 * this->q.q_4) - m.y;
        f_6 = twob_x * (q_1q_3 + q_2q_4) + twob_z * (0.5f - this->q.q_2 * this->q.q_2 - this->q.q_3 * this->q.q_3) - m.z;
        J_11or24 = twoq_3;
        J_12or23 = 2.0f * this->q.q_4;
        J_13or22 = twoq_1;
        J_14or21 = twoq_2;
        J_32 = 2.0f * J_14or21;
        J_33 = 2.0f * J_11or24;
        J_41 = twob_zq_3;
        J_42 = twob_zq_4;
        J_43 = 2.0f * twob_xq_3 + twob_zq_1;
        J_44 = 2.0f * twob_xq_4 - twob_zq_2;
        J_51 = twob_xq_4 - twob_zq_2;
        J_52 = twob_xq_3 + twob_zq_1;
        J_53 = twob_xq_2 + twob_zq_4;
        J_54 = twob_xq_1 - twob_zq_3;
        J_61 = twob_xq_3;
        J_62 = twob_xq_4 - 2.0f * twob_zq_2;
        J_63 = twob_xq_1 - 2.0f * twob_zq_3;
        J_64 = twob_xq_2;

        // compute the gradient (matrix multiplication)
        qHatDot.q_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
        qHatDot.q_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
        qHatDot.q_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
        qHatDot.q_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

        // normalise the gradient to estimate direction of the gyroscope error
        qHatDot.norm();

        // compute angular estimated direction of the gyroscope error
        w_err_x = twoq_1 * qHatDot.q_2 - twoq_2 * qHatDot.q_1 - twoq_3 * qHatDot.q_4 + twoq_4 * qHatDot.q_3;
        w_err_y = twoq_1 * qHatDot.q_3 + twoq_2 * qHatDot.q_4 - twoq_3 * qHatDot.q_1 - twoq_4 * qHatDot.q_2;
        w_err_z = twoq_1 * qHatDot.q_4 - twoq_2 * qHatDot.q_3 + twoq_3 * qHatDot.q_2 - twoq_4 * qHatDot.q_1;

        // compute and remove the gyroscope biases
        w_bx += w_err_x * deltat * zeta;
        w_by += w_err_y * deltat * zeta;
        w_bz += w_err_z * deltat * zeta;
        w.x -= w_bx;
        w.y -= w_by;
        w.z -= w_bz;
    }

    // compute the quaternion rate measured by gyroscopes
    qDot_omega.q_1 = -halfq_2 * w.x - halfq_3 * w.y - halfq_4 * w.z;
    qDot_omega.q_2 = halfq_1 * w.x + halfq_3 * w.z - halfq_4 * w.y;
    qDot_omega.q_3 = halfq_1 * w.y - halfq_2 * w.z + halfq_4 * w.x;
    qDot_omega.q_4 = halfq_1 * w.z + halfq_2 * w.y - halfq_3 * w.x;

    // compute then integrate the estimated quaternion rate
    this->q.q_1 += (qDot_omega.q_1 - (beta * qHatDot.q_1)) * deltat;
    this->q.q_2 += (qDot_omega.q_2 - (beta * qHatDot.q_2)) * deltat;
    this->q.q_3 += (qDot_omega.q_3 - (beta * qHatDot.q_3)) * deltat;
    this->q.q_4 += (qDot_omega.q_4 - (beta * qHatDot.q_4)) * deltat;

    // normalise quaternion
    this->q.norm();

    // std::cout << "q1: " << q.q_1 << std::endl;
    // std::cout << "q2: " << q.q_2 << std::endl;
    // std::cout << "q3: " << q.q_3 << std::endl;
    // std::cout << "q4: " << q.q_4 << std::endl;
}

void MadgwickFilter::updateIMUFilter(Vec3 &w, Vec3 &a)
{
    float q_1 = this->q.q_1;
    float q_2 = this->q.q_2;
    float q_3 = this->q.q_3;
    float q_4 = this->q.q_4;

    float f_1, f_2, f_3;
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;

    Quaternion qDot_omega = Quaternion(0, 0, 0, 0);
    Quaternion qHatDot = Quaternion(0, 0, 0, 0);

    float halfq_1 = 0.5f * q_1;
    float halfq_2 = 0.5f * q_2;
    float halfq_3 = 0.5f * q_3;
    float halfq_4 = 0.5f * q_4;
    float twoq_1 = 2.0f * q_1;
    float twoq_2 = 2.0f * q_2;
    float twoq_3 = 2.0f * q_3;

    if(!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) 
    {
        a.norm();

        f_1 = twoq_2 * q_4 - twoq_1 * q_3 - a.x;
        f_2 = twoq_1 * q_2 + twoq_3 * q_4 - a.y;
        f_3 = 1.0f - twoq_2 * q_2 - twoq_3 * q_3 - a.z; 
        J_11or24 = twoq_3;
        J_12or23 = 2.0f * q_4;
        J_13or22 = twoq_1;
        J_14or21 = twoq_2;
        J_32 = 2.0f * J_14or21;
        J_33 = 2.0f * J_11or24;

        qHatDot.q_1 = J_14or21 * f_2 - J_11or24 * f_1;
        qHatDot.q_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3; 
        qHatDot.q_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1; 
        qHatDot.q_4 = J_14or21 * f_1 + J_11or24 * f_2;

        qHatDot.norm();
    }

    qDot_omega.q_1 = -halfq_2 * w.x - halfq_3 * w.y - halfq_4 * w.z;
    qDot_omega.q_2 = halfq_1 * w.x + halfq_3 * w.z - halfq_4 * w.y;
    qDot_omega.q_3 = halfq_1 * w.y - halfq_2 * w.z + halfq_4 * w.x;
    qDot_omega.q_4 = halfq_1 * w.z + halfq_2 * w.y - halfq_3 * w.x;

    this->q.q_1 += (qDot_omega.q_1 - (beta * qHatDot.q_1)) * deltat;
    this->q.q_2 += (qDot_omega.q_2 - (beta * qHatDot.q_2)) * deltat;
    this->q.q_3 += (qDot_omega.q_3 - (beta * qHatDot.q_3)) * deltat;
    this->q.q_4 += (qDot_omega.q_4 - (beta * qHatDot.q_4)) * deltat;

    this->q.norm();

    // std::cout << "q1 IMU: " << q_1 << std::endl;
    // std::cout << "q2: " << q_2 << std::endl;
    // std::cout << "q3: " << q_3 << std::endl;
    // std::cout << "q4: " << q_4 << std::endl;

}