#include "comp_filter.hpp"

void ComplementaryFilter::updateFilter(Vec3 w, Vec3 a, Vec3 m)
{
    float norm;
    float oneMinusGain = 1 - this->gain;

    Quaternion q_omega = Quaternion(0, 0, 0, 0);
    Quaternion q_am = Quaternion(0, 0, 0, 0);

    float half_q_1 = 0.5f * this->q.q_1;
    float half_q_2 = 0.5f * this->q.q_2;
    float half_q_3 = 0.5f * this->q.q_3;
    float half_q_4 = 0.5f * this->q.q_4;

    float phi = atan2(a.y, a.z); // roll
    float theta = atan2(-a.x, sqrt((a.y*a.y + a.z*a.z))); // pitch

    float b_x = m.x*cos(phi) + m.y*sin(phi)*sin(theta) + m.z*sin(phi)*cos(theta);
    float b_y = m.y*cos(theta) - m.z*sin(theta);
    float b_z = -m.x*sin(phi) + m.y*cos(phi)*sin(theta) + m.z*cos(phi)*cos(theta);
    
    float psi = atan2(-b_y, b_x); // yaw

    q_am.q_1 = cos(0.5f*theta)*cos(0.5f*phi)*cos(0.5f*psi) + sin(0.5f*theta)*sin(0.5f*phi)*sin(0.5f*psi);
    q_am.q_2 = sin(0.5f*theta)*cos(0.5f*phi)*cos(0.5f*psi) - cos(0.5f*theta)*sin(0.5f*phi)*sin(0.5f*psi);
    q_am.q_3 = cos(0.5f*theta)*sin(0.5f*phi)*cos(0.5f*psi) + sin(0.5f*theta)*cos(0.5f*phi)*sin(0.5f*psi);
    q_am.q_4 = cos(0.5f*theta)*cos(0.5f*phi)*sin(0.5f*psi) - sin(0.5f*theta)*sin(0.5f*phi)*cos(0.5f*psi);

    // q_omega_1 = 1 - deltat*(half_q_2 * w_x - half_q_3 * w_y - half_q_4 * w_z);
    // q_omega_2 = 1 + deltat*(half_q_1 * w_x + half_q_3 * w_z - half_q_4 * w_y);
    // q_omega_3 = 1 + deltat*(half_q_1 * w_y - half_q_2 * w_z + half_q_4 * w_x);  
    // q_omega_4 = deltat*(-half_q_1 * w_z + half_q_2 * w_y - half_q_3 * w_x) + 1;
    q_omega.q_1 = -half_q_2 * w.x - half_q_3 * w.y - half_q_4 * w.z;
    q_omega.q_2 = half_q_1 * w.x + half_q_3 * w.z - half_q_4 * w.y;
    q_omega.q_3 = half_q_1 * w.y - half_q_2 * w.z + half_q_4 * w.x;
    q_omega.q_4 = -half_q_1 * w.z + half_q_2 * w.y - half_q_3 * w.x;

    this->q.q_1 += (q_omega.q_1 * this->deltat * oneMinusGain) - this->gain*(this->q.q_1 - q_am.q_1);
    this->q.q_2 += (q_omega.q_2 * this->deltat * oneMinusGain) - this->gain*(this->q.q_2 - q_am.q_2);
    this->q.q_3 += (q_omega.q_3 * this->deltat * oneMinusGain) - this->gain*(this->q.q_3 - q_am.q_3);
    this->q.q_4 += (q_omega.q_4 * this->deltat * oneMinusGain) - this->gain*(this->q.q_4 - q_am.q_4);

    this->q.norm();

    // std::cout << "q1: " << q_1 << std::endl;
    // std::cout << "q2: " << q_2 << std::endl;
    // std::cout << "q3: " << q_3 << std::endl;
    // std::cout << "q4: " << q_4 << std::endl;
}

void ComplementaryFilter::updateFilter(Vec3 w, Vec3 a)
{
    float norm;
    float oneMinusGain = 1 - this->gain;

    Quaternion q_omega = Quaternion(0, 0, 0, 0);
    Quaternion q_am = Quaternion(0, 0, 0, 0);

    float half_q_1 = 0.5f * this->q.q_1;
    float half_q_2 = 0.5f * this->q.q_2;
    float half_q_3 = 0.5f * this->q.q_3;
    float half_q_4 = 0.5f * this->q.q_4;

    float phi = atan2(a.y, a.z); // roll
    float theta = atan2(-a.x, sqrt((a.y*a.y + a.z*a.z))); // pitch
    
    float psi = 0; // yaw

    q_am.q_1 = cos(0.5f*theta)*cos(0.5f*phi)*cos(0.5f*psi) + sin(0.5f*theta)*sin(0.5f*phi)*sin(0.5f*psi);
    q_am.q_2 = sin(0.5f*theta)*cos(0.5f*phi)*cos(0.5f*psi) - cos(0.5f*theta)*sin(0.5f*phi)*sin(0.5f*psi);
    q_am.q_3 = cos(0.5f*theta)*sin(0.5f*phi)*cos(0.5f*psi) + sin(0.5f*theta)*cos(0.5f*phi)*sin(0.5f*psi);
    q_am.q_4 = cos(0.5f*theta)*cos(0.5f*phi)*sin(0.5f*psi) - sin(0.5f*theta)*sin(0.5f*phi)*cos(0.5f*psi);

    // q_omega_1 = 1 - deltat*(half_q_2 * w_x - half_q_3 * w_y - half_q_4 * w_z);
    // q_omega_2 = 1 + deltat*(half_q_1 * w_x + half_q_3 * w_z - half_q_4 * w_y);
    // q_omega_3 = 1 + deltat*(half_q_1 * w_y - half_q_2 * w_z + half_q_4 * w_x);  
    // q_omega_4 = deltat*(-half_q_1 * w_z + half_q_2 * w_y - half_q_3 * w_x) + 1;
    q_omega.q_1 = -half_q_2 * w.x - half_q_3 * w.y - half_q_4 * w.z;
    q_omega.q_2 = half_q_1 * w.x + half_q_3 * w.z - half_q_4 * w.y;
    q_omega.q_3 = half_q_1 * w.y - half_q_2 * w.z + half_q_4 * w.x;
    q_omega.q_4 = -half_q_1 * w.z + half_q_2 * w.y - half_q_3 * w.x;

    this->q.q_1 += (q_omega.q_1 * this->deltat * oneMinusGain) - this->gain*(this->q.q_1 - q_am.q_1);
    this->q.q_2 += (q_omega.q_2 * this->deltat * oneMinusGain) - this->gain*(this->q.q_2 - q_am.q_2);
    this->q.q_3 += (q_omega.q_3 * this->deltat * oneMinusGain) - this->gain*(this->q.q_3 - q_am.q_3);
    this->q.q_4 += (q_omega.q_4 * this->deltat * oneMinusGain) - this->gain*(this->q.q_4 - q_am.q_4);

    this->q.norm();

    // std::cout << "q1: " << q_1 << std::endl;
    // std::cout << "q2: " << q_2 << std::endl;
    // std::cout << "q3: " << q_3 << std::endl;
    // std::cout << "q4: " << q_4 << std::endl;
}