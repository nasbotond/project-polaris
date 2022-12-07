#include "naive_filter.hpp"

void NaiveFilter::setInitialState(Quaternion &initial)
{
    q.q_1 = initial.q_1;
    q.q_2 = initial.q_2;
    q.q_3 = initial.q_3;
    q.q_4 = initial.q_4;
}

void NaiveFilter::updateFilter(const Vec3 w, const Vec3 a, const Vec3 m)
{
    float one_minus_gain = 1.0 - gain;

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

    if((m.x == 0.0) && (m.y == 0.0) && (m.z == 0.0))
    {
        updateFilter(w, a);
        return;
    }

    q_am = Quaternion::getOrientationFromAccMag(a, m);
    
    // Vec3 Rz = a;
    // Rz.norm();
    // // ENU
    // // Vec3 Rx = Vec3::cross(m, Rz);
    // // Vec3 Ry = Vec3::cross(Rz, Rx);
    // // NED
    // Vec3 tmp = Vec3::cross(Rz, m);
    // Vec3 Rx = Vec3::cross(tmp, Rz);
    // Vec3 Ry = Vec3::cross(Rz, Rx);
    // Ry.norm();
    // Rx.norm();

    // // // Chiaverini method
    // q_am.q_1 = (Rx.x + Ry.y + Rz.z + 1.0) > 0.0 ? 0.5 * sqrt(Rx.x + Ry.y + Rz.z + 1.0) : 1.0;
    // q_am.q_2 = 0.5 * sgn(Rz.y - Ry.z) * sqrt(std::clamp(Rx.x - Ry.y - Rz.z, -1.0f, 1.0f) + 1.0);
    // q_am.q_3 = 0.5 * sgn(Rx.z - Rz.x) * sqrt(std::clamp(Ry.y - Rz.z - Rx.x, -1.0f, 1.0f) + 1.0);
    // q_am.q_4 = 0.5 * sgn(Ry.x - Rx.y) * sqrt(std::clamp(Rz.z - Rx.x - Ry.y, -1.0f, 1.0f) + 1.0);

    q_am.norm();

    q_omega.q_1 = q.q_1 + (-half_q_2 * w.x - half_q_3 * w.y - half_q_4 * w.z)*deltat;
    q_omega.q_2 = q.q_2 + (half_q_1 * w.x + half_q_3 * w.z - half_q_4 * w.y)*deltat;
    q_omega.q_3 = q.q_3 + (half_q_1 * w.y - half_q_2 * w.z + half_q_4 * w.x)*deltat;
    q_omega.q_4 = q.q_4 + (half_q_1 * w.z + half_q_2 * w.y - half_q_3 * w.x)*deltat;

    q_omega.norm();

    if(sqrt((q_omega.q_1+q_am.q_1)*(q_omega.q_1+q_am.q_1) + (q_omega.q_2+q_am.q_2)*(q_omega.q_2+q_am.q_2) + (q_omega.q_3+q_am.q_3)*(q_omega.q_3+q_am.q_3) + (q_omega.q_4+q_am.q_4)*(q_omega.q_4+q_am.q_4)) < sqrt(2))
    {
        q.q_1 = q_omega.q_1 * one_minus_gain - gain*(q_am.q_1);
        q.q_2 = q_omega.q_2 * one_minus_gain - gain*(q_am.q_2);
        q.q_3 = q_omega.q_3 * one_minus_gain - gain*(q_am.q_3);
        q.q_4 = q_omega.q_4 * one_minus_gain - gain*(q_am.q_4);
    }
    else
    {
        q.q_1 = q_omega.q_1 * one_minus_gain + gain*(q_am.q_1);
        q.q_2 = q_omega.q_2 * one_minus_gain + gain*(q_am.q_2);
        q.q_3 = q_omega.q_3 * one_minus_gain + gain*(q_am.q_3);
        q.q_4 = q_omega.q_4 * one_minus_gain + gain*(q_am.q_4);
    }

    q.norm();
}

void NaiveFilter::updateFilter(const Vec3 w, const Vec3 a)
{
    float one_minus_gain = 1 - gain;

    Quaternion q_omega = Quaternion(0, 0, 0, 0);
    Quaternion q_am = Quaternion(1.0, 0, 0, 0);

    float half_q_1 = 0.5 * q.q_1;
    float half_q_2 = 0.5 * q.q_2;
    float half_q_3 = 0.5 * q.q_3;
    float half_q_4 = 0.5 * q.q_4;

    Vec3 acc = a;
    if((acc.x*acc.x + acc.y*acc.y + acc.z*acc.z) > 0)
    {
        acc.norm();

        float theta = atan2(acc.y, acc.z); // roll
        float phi = atan2(-acc.x, sqrt((acc.y*acc.y + acc.z*acc.z))); // pitch
        
        float psi = 0; // yaw

        q_am.q_1 = cos(0.5*theta)*cos(0.5*phi);
        q_am.q_2 = sin(0.5*theta)*cos(0.5*phi);
        q_am.q_3 = cos(0.5*theta)*sin(0.5*phi);
        q_am.q_4 = -sin(0.5*theta)*sin(0.5*phi);

        q_am.norm();
    }

    q_omega.q_1 = q.q_1 + (-half_q_2 * w.x - half_q_3 * w.y - half_q_4 * w.z)*deltat;
    q_omega.q_2 = q.q_2 + (half_q_1 * w.x + half_q_3 * w.z - half_q_4 * w.y)*deltat;
    q_omega.q_3 = q.q_3 + (half_q_1 * w.y - half_q_2 * w.z + half_q_4 * w.x)*deltat;
    q_omega.q_4 = q.q_4 + (half_q_1 * w.z + half_q_2 * w.y - half_q_3 * w.x)*deltat;

    q_omega.norm();

    if(sqrt((q_omega.q_1+q_am.q_1)*(q_omega.q_1+q_am.q_1) + (q_omega.q_2+q_am.q_2)*(q_omega.q_2+q_am.q_2) + (q_omega.q_3+q_am.q_3)*(q_omega.q_3+q_am.q_3) + (q_omega.q_4+q_am.q_4)*(q_omega.q_4+q_am.q_4)) < sqrt(2))
    {
        q.q_1 = q_omega.q_1 * one_minus_gain - gain*(q_am.q_1);
        q.q_2 = q_omega.q_2 * one_minus_gain - gain*(q_am.q_2);
        q.q_3 = q_omega.q_3 * one_minus_gain - gain*(q_am.q_3);
        q.q_4 = q_omega.q_4 * one_minus_gain - gain*(q_am.q_4);
    }
    else
    {
        q.q_1 = q_omega.q_1 * one_minus_gain + gain*(q_am.q_1);
        q.q_2 = q_omega.q_2 * one_minus_gain + gain*(q_am.q_2);
        q.q_3 = q_omega.q_3 * one_minus_gain + gain*(q_am.q_3);
        q.q_4 = q_omega.q_4 * one_minus_gain + gain*(q_am.q_4);
    }

    q.norm();
}