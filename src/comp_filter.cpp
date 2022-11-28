#include "comp_filter.hpp"

void ComplementaryFilter::setInitialState(Quaternion &initial)
{
    q.q_1 = initial.q_1;
    q.q_2 = initial.q_2;
    q.q_3 = initial.q_3;
    q.q_4 = initial.q_4;
}

void ComplementaryFilter::updateFilter(const Vec3 &w, const Vec3 &a, const Vec3 &m)
{
    float norm;
    float oneMinusGain = 1 - gain;

    Quaternion q_omega = Quaternion(0, 0, 0, 0);
    Quaternion q_am = Quaternion(0, 0, 0, 0);

    float half_q_1 = 0.5 * q.q_1;
    float half_q_2 = 0.5 * q.q_2;
    float half_q_3 = 0.5 * q.q_3;
    float half_q_4 = 0.5 * q.q_4;

    // float phi = atan2(a.y, a.z); // roll
    // float theta = atan2(-a.x, sqrt((a.y*a.y + a.z*a.z))); // pitch

    // float b_x = m.x*cos(phi) + m.y*sin(phi)*sin(theta) + m.z*sin(phi)*cos(theta);
    // float b_y = m.y*cos(theta) - m.z*sin(theta);
    // float b_z = -m.x*sin(phi) + m.y*cos(phi)*sin(theta) + m.z*cos(phi)*cos(theta);
    
    // float psi = atan2(-b_y, b_x); // yaw

    // q_am.q_1 = cos(0.5*theta)*cos(0.5*phi)*cos(0.5*psi) + sin(0.5*theta)*sin(0.5*phi)*sin(0.5*psi);
    // q_am.q_2 = sin(0.5*theta)*cos(0.5*phi)*cos(0.5*psi) - cos(0.5*theta)*sin(0.5*phi)*sin(0.5*psi);
    // q_am.q_3 = cos(0.5*theta)*sin(0.5*phi)*cos(0.5*psi) + sin(0.5*theta)*cos(0.5*phi)*sin(0.5*psi);
    // q_am.q_4 = cos(0.5*theta)*cos(0.5*phi)*sin(0.5*psi) - sin(0.5*theta)*sin(0.5*phi)*cos(0.5*psi);

    Vec3 Ry = Vec3(a.y*m.z-m.y*a.z, a.x*m.z - a.z*m.x, a.x*m.y-a.y-m.x);
    Vec3 Rx = Vec3(Ry.y*m.z - Ry.z*a.y, Ry.x*a.z - Ry.z*a.x, Ry.x*a.y - Ry.y*a.x);
    Ry.norm();
    Rx.norm();

    // Chiaverini method
    q_am.q_1 = (Rx.x + Ry.y + a.z + 1.0) > 0.0 ? 0.5 * sqrt(Rx.x + Ry.y + a.z + 1.0) : 1.0;
    q_am.q_2 = 0.5 * sgn(a.y - Ry.z) * sqrt(std::clamp(Rx.x - Ry.y - a.z, -1.0f, 1.0f) + 1.0);
    q_am.q_3 = 0.5 * sgn(Rx.z - a.x) * sqrt(std::clamp(Ry.y - a.z - Rx.x, -1.0f, 1.0f) + 1.0);
    q_am.q_4 = 0.5 * sgn(Ry.x - Rx.y) * sqrt(std::clamp(a.z - Rx.x - Ry.y, -1.0f, 1.0f) + 1.0);

    q_am.norm();

    // Rz = a/np.linalg.norm(a)
    // if frame.upper() == 'NED':
    //     Ry = np.cross(Rz, m)
    //     Rx = np.cross(Ry, Rz)
    // else:
    //     Rx = np.cross(m, Rz)
    //     Ry = np.cross(Rz, Rx)
    // Rx /= np.linalg.norm(Rx)
    // Ry /= np.linalg.norm(Ry)
    // R = np.c_[Rx, Ry, Rz].T

    // Q = np.zeros((dcm.shape[0], 4))
    // Q[:, 0] = 0.5*np.sqrt(dcm.trace(axis1=1, axis2=2) + 1.0)
    // Q[:, 1] = 0.5*np.sign(dcm[:, 2, 1] - dcm[:, 1, 2])*np.sqrt(np.clip(dcm[:, 0, 0]-dcm[:, 1, 1]-dcm[:, 2, 2], -1.0, 1.0) + 1.0)
    // Q[:, 2] = 0.5*np.sign(dcm[:, 0, 2] - dcm[:, 2, 0])*np.sqrt(np.clip(dcm[:, 1, 1]-dcm[:, 2, 2]-dcm[:, 0, 0], -1.0, 1.0) + 1.0)
    // Q[:, 3] = 0.5*np.sign(dcm[:, 1, 0] - dcm[:, 0, 1])*np.sqrt(np.clip(dcm[:, 2, 2]-dcm[:, 0, 0]-dcm[:, 1, 1], -1.0, 1.0) + 1.0)
    // Q /= np.linalg.norm(Q, axis=1)[:, None]

    // q_omega_1 = 1 - deltat*(half_q_2 * w_x - half_q_3 * w_y - half_q_4 * w_z);
    // q_omega_2 = 1 + deltat*(half_q_1 * w_x + half_q_3 * w_z - half_q_4 * w_y);
    // q_omega_3 = 1 + deltat*(half_q_1 * w_y - half_q_2 * w_z + half_q_4 * w_x);  
    // q_omega_4 = deltat*(-half_q_1 * w_z + half_q_2 * w_y - half_q_3 * w_x) + 1;
    q_omega.q_1 = -half_q_2 * w.x - half_q_3 * w.y - half_q_4 * w.z;
    q_omega.q_2 = half_q_1 * w.x + half_q_3 * w.z - half_q_4 * w.y;
    q_omega.q_3 = half_q_1 * w.y - half_q_2 * w.z + half_q_4 * w.x;
    q_omega.q_4 = -half_q_1 * w.z + half_q_2 * w.y - half_q_3 * w.x;

    q.q_1 += (q_omega.q_1 * deltat * oneMinusGain) - gain*(q.q_1 - q_am.q_1);
    q.q_2 += (q_omega.q_2 * deltat * oneMinusGain) - gain*(q.q_2 - q_am.q_2);
    q.q_3 += (q_omega.q_3 * deltat * oneMinusGain) - gain*(q.q_3 - q_am.q_3);
    q.q_4 += (q_omega.q_4 * deltat * oneMinusGain) - gain*(q.q_4 - q_am.q_4);

    q.norm();

    // std::cout << "q1: " << q_1 << std::endl;
    // std::cout << "q2: " << q_2 << std::endl;
    // std::cout << "q3: " << q_3 << std::endl;
    // std::cout << "q4: " << q_4 << std::endl;
}

void ComplementaryFilter::updateFilter(const Vec3 &w, const Vec3 &a)
{
    float norm;
    float oneMinusGain = 1 - gain;

    Quaternion q_omega = Quaternion(0, 0, 0, 0);
    Quaternion q_am = Quaternion(0, 0, 0, 0);

    float half_q_1 = 0.5 * q.q_1;
    float half_q_2 = 0.5 * q.q_2;
    float half_q_3 = 0.5 * q.q_3;
    float half_q_4 = 0.5 * q.q_4;

    float phi = atan2(a.y, a.z); // roll
    float theta = atan2(-a.x, sqrt((a.y*a.y + a.z*a.z))); // pitch
    
    float psi = 0; // yaw

    q_am.q_1 = cos(0.5*theta)*cos(0.5*phi)*cos(0.5*psi) + sin(0.5*theta)*sin(0.5*phi)*sin(0.5*psi);
    q_am.q_2 = sin(0.5*theta)*cos(0.5*phi)*cos(0.5*psi) - cos(0.5*theta)*sin(0.5*phi)*sin(0.5*psi);
    q_am.q_3 = cos(0.5*theta)*sin(0.5*phi)*cos(0.5*psi) + sin(0.5*theta)*cos(0.5*phi)*sin(0.5*psi);
    q_am.q_4 = cos(0.5*theta)*cos(0.5*phi)*sin(0.5*psi) - sin(0.5*theta)*sin(0.5*phi)*cos(0.5*psi);

    // q_omega_1 = 1 - deltat*(half_q_2 * w_x - half_q_3 * w_y - half_q_4 * w_z);
    // q_omega_2 = 1 + deltat*(half_q_1 * w_x + half_q_3 * w_z - half_q_4 * w_y);
    // q_omega_3 = 1 + deltat*(half_q_1 * w_y - half_q_2 * w_z + half_q_4 * w_x);  
    // q_omega_4 = deltat*(-half_q_1 * w_z + half_q_2 * w_y - half_q_3 * w_x) + 1;
    q_omega.q_1 = -half_q_2 * w.x - half_q_3 * w.y - half_q_4 * w.z;
    q_omega.q_2 = half_q_1 * w.x + half_q_3 * w.z - half_q_4 * w.y;
    q_omega.q_3 = half_q_1 * w.y - half_q_2 * w.z + half_q_4 * w.x;
    q_omega.q_4 = -half_q_1 * w.z + half_q_2 * w.y - half_q_3 * w.x;

    q.q_1 += (q_omega.q_1 * deltat * oneMinusGain) - gain*(q.q_1 - q_am.q_1);
    q.q_2 += (q_omega.q_2 * deltat * oneMinusGain) - gain*(q.q_2 - q_am.q_2);
    q.q_3 += (q_omega.q_3 * deltat * oneMinusGain) - gain*(q.q_3 - q_am.q_3);
    q.q_4 += (q_omega.q_4 * deltat * oneMinusGain) - gain*(q.q_4 - q_am.q_4);

    q.norm();

    // std::cout << "q1: " << q_1 << std::endl;
    // std::cout << "q2: " << q_2 << std::endl;
    // std::cout << "q3: " << q_3 << std::endl;
    // std::cout << "q4: " << q_4 << std::endl;
}

float ComplementaryFilter::sgn(float v)
{
  return (v < 0) ? -1.0 : ((v > 0) ? 1.0 : 0.0);
}