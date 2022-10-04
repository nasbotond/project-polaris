#include "comp_filter.hpp"

void ComplementaryFilter::updateFilter(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z)
{
    float norm;
    float q_omega_1, q_omega_2, q_omega_3, q_omega_4;
    float q_am_1, q_am_2, q_am_3, q_am_4;
    float oneMinusGain = 1 - gain;

    float half_q_1 = 0.5f * q_1;
    float half_q_2 = 0.5f * q_2;
    float half_q_3 = 0.5f * q_3;
    float half_q_4 = 0.5f * q_4;

    float theta = atan2(a_y, a_z);
    float phi = atan2(-a_x, sqrt((a_y*a_y + a_z*a_z)));

    float b_x = m_x*cos(theta) + m_y*sin(theta)*sin(phi) + m_z*sin(theta)*cos(phi);
    float b_y = m_y*cos(phi) - m_z*sin(phi);
    float b_z = -m_x*sin(phi) + m_y*cos(theta)*sin(phi) + m_z*cos(theta)*cos(phi);
    
    float epsilon = atan2(-b_y, b_x);

    q_am_1 = cos(0.5f*phi)*cos(0.5f*theta)*cos(0.5f*epsilon) + sin(0.5f*phi)*sin(0.5f*theta)*sin(0.5f*epsilon);
    q_am_2 = sin(0.5f*phi)*cos(0.5f*theta)*cos(0.5f*epsilon) - cos(0.5f*phi)*sin(0.5f*theta)*sin(0.5f*epsilon);
    q_am_3 = cos(0.5f*phi)*sin(0.5f*theta)*cos(0.5f*epsilon) + sin(0.5f*phi)*cos(0.5f*theta)*sin(0.5f*epsilon);
    q_am_4 = cos(0.5f*phi)*cos(0.5f*theta)*sin(0.5f*epsilon) - sin(0.5f*phi)*sin(0.5f*theta)*cos(0.5f*epsilon);

    // q_omega_1 = 1 - half_q_2 * w_x - half_q_3 * w_y - half_q_4 * w_z;
    // q_omega_2 = half_q_1 * w_x + 1 + half_q_3 * w_z - half_q_4 * w_y;
    // q_omega_3 = half_q_1 * w_y - half_q_2 * w_z + 1 + half_q_4 * w_x;  
    // q_omega_4 = -half_q_1 * w_z + half_q_2 * w_y - half_q_3 * w_x + 1;
    q_omega_1 = -half_q_2 * w_x - half_q_3 * w_y - half_q_4 * w_z;
    q_omega_2 = half_q_1 * w_x + half_q_3 * w_z - half_q_4 * w_y;
    q_omega_3 = half_q_1 * w_y - half_q_2 * w_z + half_q_4 * w_x;
    q_omega_4 = -half_q_1 * w_z + half_q_2 * w_y - half_q_3 * w_x;

    q_1 += (q_omega_1 * deltat * oneMinusGain) - gain*(q_1 - q_am_1);
    q_2 += (q_omega_2 * deltat * oneMinusGain) - gain*(q_2 - q_am_2);
    q_3 += (q_omega_3 * deltat * oneMinusGain) - gain*(q_3 - q_am_3);
    q_4 += (q_omega_4 * deltat * oneMinusGain) - gain*(q_4 - q_am_4);
    // q_1 = oneMinusGain*q_omega_1*q_1 + gain*q_am_1;
    // q_2 = oneMinusGain*q_omega_2*q_2 + gain*q_am_2;
    // q_3 = oneMinusGain*q_omega_3*q_3 + gain*q_am_3;
    // q_4 = oneMinusGain*q_omega_4*q_4 + gain*q_am_4;

    norm = sqrt(q_1 * q_1 + q_2 * q_2 + q_3 * q_3 + q_4 * q_4); 
    q_1 /= norm;
    q_2 /= norm;
    q_3 /= norm;
    q_4 /= norm;

    // std::cout << "q1: " << q_1 << std::endl;
    // std::cout << "q2: " << q_2 << std::endl;
    // std::cout << "q3: " << q_3 << std::endl;
    // std::cout << "q4: " << q_4 << std::endl;
}