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

    if((m.x == 0.0) && (m.y == 0.0) && (m.z == 0.0))
    {
        updateIMUFilter(w, a);
        return;
    }

    float q_1q_2 = q.q_1 * q.q_2;
    float q_1q_3 = q.q_1 * q.q_3;
    float q_1q_4 = q.q_1 * q.q_4;
    float q_3q_4 = q.q_3 * q.q_4;
    float q_2q_3 = q.q_2 * q.q_3;
    float q_2q_4 = q.q_2 * q.q_4;

    float halfq_1 = 0.5 * q.q_1;
    float halfq_2 = 0.5 * q.q_2;
    float halfq_3 = 0.5 * q.q_3;
    float halfq_4 = 0.5 * q.q_4;
    float twoq_1 = 2.0 * q.q_1;
    float twoq_2 = 2.0 * q.q_2;
    float twoq_3 = 2.0 * q.q_3;
    float twoq_4 = 2.0 * q.q_4;
    float twob_x = 2.0 * b_x;
    float twob_z = 2.0 * b_z;
    float twob_xq_1 = 2.0 * b_x * q.q_1;
    float twob_xq_2 = 2.0 * b_x * q.q_2;
    float twob_xq_3 = 2.0 * b_x * q.q_3;
    float twob_xq_4 = 2.0 * b_x * q.q_4;
    float twob_zq_1 = 2.0 * b_z * q.q_1;
    float twob_zq_2 = 2.0 * b_z * q.q_2;
    float twob_zq_3 = 2.0 * b_z * q.q_3;
    float twob_zq_4 = 2.0 * b_z * q.q_4;

    if(!((a.x == 0.0) && (a.y == 0.0) && (a.z == 0.0))) 
    {
        // std::cout << "ax: " << a.x << std::endl;
        // std::cout << "ay: " << a.y << std::endl;
        // std::cout << "az: " << a.z << std::endl;

        a.norm();
        m.norm();

        // compute the objective function and Jacobian
        f_1 = twoq_2 * q.q_4 - twoq_1 * q.q_3 - a.x;
        f_2 = twoq_1 * q.q_2 + twoq_3 * q.q_4 - a.y;
        f_3 = 1.0 - twoq_2 * q.q_2 - twoq_3 * q.q_3 - a.z;
        f_4 = twob_x * (0.5 - q.q_3 * q.q_3 - q.q_4 * q.q_4) + twob_z * (q_2q_4 - q_1q_3) - m.x; 
        f_5 = twob_x * (q.q_2 * q.q_3 - q.q_1 * q.q_4) + twob_z * (q.q_1 * q.q_2 + q.q_3 * q.q_4) - m.y;
        f_6 = twob_x * (q_1q_3 + q_2q_4) + twob_z * (0.5 - q.q_2 * q.q_2 - q.q_3 * q.q_3) - m.z;
        J_11or24 = twoq_3;
        J_12or23 = 2.0 * q.q_4;
        J_13or22 = twoq_1;
        J_14or21 = twoq_2;
        J_32 = 2.0 * J_14or21;
        J_33 = 2.0 * J_11or24;
        J_41 = twob_zq_3;
        J_42 = twob_zq_4;
        J_43 = 2.0 * twob_xq_3 + twob_zq_1;
        J_44 = 2.0 * twob_xq_4 - twob_zq_2;
        J_51 = twob_xq_4 - twob_zq_2;
        J_52 = twob_xq_3 + twob_zq_1;
        J_53 = twob_xq_2 + twob_zq_4;
        J_54 = twob_xq_1 - twob_zq_3;
        J_61 = twob_xq_3;
        J_62 = twob_xq_4 - 2.0 * twob_zq_2;
        J_63 = twob_xq_1 - 2.0 * twob_zq_3;
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
    q.q_1 += (qDot_omega.q_1 - (beta * qHatDot.q_1)) * deltat;
    q.q_2 += (qDot_omega.q_2 - (beta * qHatDot.q_2)) * deltat;
    q.q_3 += (qDot_omega.q_3 - (beta * qHatDot.q_3)) * deltat;
    q.q_4 += (qDot_omega.q_4 - (beta * qHatDot.q_4)) * deltat;

    // normalise quaternion
    q.norm();

    // std::cout << "q1: " << q.q_1 << std::endl;
    // std::cout << "q2: " << q.q_2 << std::endl;
    // std::cout << "q3: " << q.q_3 << std::endl;
    // std::cout << "q4: " << q.q_4 << std::endl;

    // recompute flux in the earth frame with new q
    q_1q_2 = q.q_1 * q.q_2;
    q_1q_3 = q.q_1 * q.q_3;
    q_1q_4 = q.q_1 * q.q_4;
    q_3q_4 = q.q_3 * q.q_4;
    q_2q_3 = q.q_2 * q.q_3;
    q_2q_4 = q.q_2 * q.q_4;
    float twom_x = 2.0 * m.x;
    float twom_y = 2.0 * m.y;
    float twom_z = 2.0 * m.z;

    h_x = twom_x * (0.5 - q.q_3 * q.q_3 - q.q_4 * q.q_4) + twom_y * (q_2q_3 - q_1q_4) + twom_z * (q_2q_4 + q_1q_3);
    h_y = twom_x * (q_2q_3 + q_1q_4) + twom_y * (0.5 - q.q_2 * q.q_2 - q.q_4 * q.q_4) + twom_z * (q_3q_4 - q_1q_2);
    h_z = twom_x * (q_2q_4 - q_1q_3) + twom_y * (q_3q_4 + q_1q_2) + twom_z * (0.5 - q.q_2 * q.q_2 - q.q_3 * q.q_3);
    
    // h_x = m.x * (q.q_1*q.q_1 + q.q_2*q.q_2 -q.q_3*q.q_3 - q.q_4*q.q_4) + twom_y * (q_2q_3 - q_1q_4) + twom_z * (q_2q_4 + q_1q_3);
    // h_y = twom_x * (q_2q_3 + q_1q_4) + m.y * (q.q_1*q.q_1 - q.q_2*q.q_2 + q.q_3*q.q_3 - q.q_4*q.q_4) + twom_z * (q_3q_4 - q_1q_2);
    // h_z = twom_x * (q_2q_4 - q_1q_3) + twom_y * (q_3q_4 + q_1q_2) + m.z * (q.q_1*q.q_1 - q.q_2*q.q_2 - q.q_3*q.q_3 + q.q_4*q.q_4);

    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z;
    // std::cout << b_x << std::endl;
    // std::cout << b_z << std::endl;
}

/*From BROAD*/
// void MadgwickFilter::updateMARGFilter(Vec3 &g, Vec3 &a, Vec3 &m) {
//     float recipNorm;
//     float s0, s1, s2, s3;
//     float qDot1, qDot2, qDot3, qDot4;
//     float hx, hy;
//     float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

//     // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
//     // if((m.x == 0.0) && (m.y == 0.0) && (m.z == 0.0)) {
//     //     updateIMU(gx, gy, gz, ax, ay, az);
//     //     return;
//     // }

//     // Rate of change of quaternion from gyroscope
//     qDot1 = 0.5 * (-q.q_2 * g.x - q.q_3 * g.y - q.q_4 * g.z);
//     qDot2 = 0.5 * (q.q_1 * g.x + q.q_3* g.z - q.q_4 * g.y);
//     qDot3 = 0.5 * (q.q_1 * g.y - q.q_2 * g.z + q.q_4 * g.x);
//     qDot4 = 0.5 * (q.q_1 * g.z + q.q_2 * g.y - q.q_3 * g.x);

//     // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//     if(!((a.x == 0.0) && (a.y == 0.0) && (a.z == 0.0))) {

//         // Normalise accelerometer measurement
//         recipNorm = 1.0/sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
//         a.x *= recipNorm;
//         a.y *= recipNorm;
//         a.z *= recipNorm;

//         // Normalise magnetometer measurement
//         recipNorm = 1.0/sqrtf(m.x * m.x + m.y * m.y + m.z * m.z);
//         m.x *= recipNorm;
//         m.y *= recipNorm;
//         m.z *= recipNorm;

//         // Auxiliary variables to avoid repeated arithmetic
//         _2q0mx = 2.0 * q.q_1 * m.x;
//         _2q0my = 2.0 * q.q_1 * m.y;
//         _2q0mz = 2.0 * q.q_1 * m.z;
//         _2q1mx = 2.0 * q.q_2 * m.x;
//         _2q0 = 2.0 * q.q_1;
//         _2q1 = 2.0 * q.q_2;
//         _2q2 = 2.0 * q.q_3;
//         _2q3 = 2.0 * q.q_4;
//         _2q0q2 = 2.0 * q.q_1 * q.q_3;
//         _2q2q3 = 2.0 * q.q_3 * q.q_4;
//         q0q0 = q.q_1 * q.q_1;
//         q0q1 = q.q_1 * q.q_2;
//         q0q2 = q.q_1 * q.q_3;
//         q0q3 = q.q_1 * q.q_4;
//         q1q1 = q.q_2 * q.q_2;
//         q1q2 = q.q_2 * q.q_3;
//         q1q3 = q.q_2 * q.q_4;
//         q2q2 = q.q_3 * q.q_3;
//         q2q3 = q.q_3 * q.q_4;
//         q3q3 = q.q_4 * q.q_4;

//         // Reference direction of Earth's magnetic field
//         hx = m.x * q0q0 - _2q0my * q.q_4 + _2q0mz * q.q_3 + m.x * q1q1 + _2q1 * m.y * q.q_3 + _2q1 * m.z * q.q_4 - m.x * q2q2 - m.x * q3q3;
//         hy = _2q0mx * q.q_4 + m.y * q0q0 - _2q0mz * q.q_2 + _2q1mx * q.q_3 - m.y * q1q1 + m.y * q2q2 + _2q2 * m.x * q.q_4 - m.y * q3q3;
//         _2bx = sqrtf(hx * hx + hy * hy);
//         _2bz = -_2q0mx * q.q_3 + _2q0my * q.q_2 + m.z * q0q0 + _2q1mx * q.q_4 - m.z * q1q1 + _2q2 * m.y * q.q_4 - m.z * q2q2 + m.z * q3q3;
//         _4bx = 2.0 * _2bx;
//         _4bz = 2.0 * _2bz;

//         // Gradient decent algorithm corrective step
//         s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - a.x) + _2q1 * (2.0 * q0q1 + _2q2q3 - a.y) - _2bz * q.q_3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (-_2bx * q.q_4 + _2bz * q.q_2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + _2bx * q.q_3 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);
//         s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - a.x) + _2q0 * (2.0 * q0q1 + _2q2q3 - a.y) - 4.0 * q.q_2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - a.z) + _2bz * q.q_4 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (_2bx * q.q_3 + _2bz * q.q_1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + (_2bx * q.q_4 - _4bz * q.q_2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);
//         s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - a.x) + _2q3 * (2.0 * q0q1 + _2q2q3 - a.y) - 4.0 * q.q_3 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - a.z) + (-_4bx * q.q_3 - _2bz * q.q_1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (_2bx * q.q_2 + _2bz * q.q_4) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + (_2bx * q.q_1 - _4bz * q.q_3) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);
//         s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - a.x) + _2q2 * (2.0 * q0q1 + _2q2q3 - a.y) + (-_4bx * q.q_4 + _2bz * q.q_2) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (-_2bx * q.q_1 + _2bz * q.q_3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + _2bx * q.q_2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);
//         recipNorm = 1.0/sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//         s0 *= recipNorm;
//         s1 *= recipNorm;
//         s2 *= recipNorm;
//         s3 *= recipNorm;

//         // Apply feedback step
//         qDot1 -= beta * s0;
//         qDot2 -= beta * s1;
//         qDot3 -= beta * s2;
//         qDot4 -= beta * s3;
//     }

//     // Integrate rate of change of quaternion to yield quaternion
//     q.q_1 += qDot1 * (1.0 / (1.0/deltat));
//     q.q_2 += qDot2 * (1.0 / (1.0/deltat));
//     q.q_3 += qDot3 * (1.0 / (1.0/deltat));
//     q.q_4 += qDot4 * (1.0 / (1.0/deltat));

//     // Normalise quaternion
//     recipNorm = 1.0/sqrtf(q.q_1 * q.q_1 + q.q_2 * q.q_2 + q.q_3 * q.q_3 + q.q_4 * q.q_4);
//     q.q_1 *= recipNorm;
//     q.q_2 *= recipNorm;
//     q.q_3 *= recipNorm;
//     q.q_4 *= recipNorm;
// }

void MadgwickFilter::updateIMUFilter(Vec3 &w, Vec3 &a)
{
    float q_1 = q.q_1;
    float q_2 = q.q_2;
    float q_3 = q.q_3;
    float q_4 = q.q_4;

    float f_1, f_2, f_3;
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;

    Quaternion qDot_omega = Quaternion(0, 0, 0, 0);
    Quaternion qHatDot = Quaternion(0, 0, 0, 0);

    float halfq_1 = 0.5 * q_1;
    float halfq_2 = 0.5 * q_2;
    float halfq_3 = 0.5 * q_3;
    float halfq_4 = 0.5 * q_4;
    float twoq_1 = 2.0 * q_1;
    float twoq_2 = 2.0 * q_2;
    float twoq_3 = 2.0 * q_3;

    if(!((a.x == 0.0) && (a.y == 0.0) && (a.z == 0.0))) 
    {
        a.norm();

        f_1 = twoq_2 * q_4 - twoq_1 * q_3 - a.x;
        f_2 = twoq_1 * q_2 + twoq_3 * q_4 - a.y;
        f_3 = 1.0 - twoq_2 * q_2 - twoq_3 * q_3 - a.z;
        J_11or24 = twoq_3;
        J_12or23 = 2.0 * q_4;
        J_13or22 = twoq_1;
        J_14or21 = twoq_2;
        J_32 = 2.0 * J_14or21;
        J_33 = 2.0 * J_11or24;

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

    q.q_1 += (qDot_omega.q_1 - (beta * qHatDot.q_1)) * deltat;
    q.q_2 += (qDot_omega.q_2 - (beta * qHatDot.q_2)) * deltat;
    q.q_3 += (qDot_omega.q_3 - (beta * qHatDot.q_3)) * deltat;
    q.q_4 += (qDot_omega.q_4 - (beta * qHatDot.q_4)) * deltat;

    q.norm();

    // std::cout << "q1 IMU: " << q_1 << std::endl;
    // std::cout << "q2: " << q_2 << std::endl;
    // std::cout << "q3: " << q_3 << std::endl;
    // std::cout << "q4: " << q_4 << std::endl;

}