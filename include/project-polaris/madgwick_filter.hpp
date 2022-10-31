#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <numeric>
#include <vector>

#define PI 3.14159265

#define deltat 0.005f //0.0035f
// #define gyroMeasError 3.14159265358979 * (5.0/180.0)
// #define gyroMeasDrift 3.14159265358979 * (0.2/180.0)
// #define beta sqrt(3.0/4.0) * gyroMeasError
// #define zeta sqrt(3.0/4.0) * gyroMeasDrift
#define beta 0.15
#define zeta 0.0

class MadgwickFilter
{
    private:
        // std::string sPath;
        // int sFps;
        // int sWidth;
        // int sHeight;
        // std::string outputFileName;

    public:

        // MadgwickFilter(const std::string &sPath, const int &sFps, const int &sWidth, const int &sHeight, const std::string &outputFileName) : sPath(sPath), sFps(sFps), sWidth(sWidth), sHeight(sHeight), outputFileName(outputFileName) {}
        // MadgwickFilter() : q_1(1.0f), q_2(0.0f), q_3(0.0f), q_4(0.0f) {}
        MadgwickFilter() : q_1(0.99886), q_2(0.0072345), q_3(-0.00048461), q_4(-0.04713) {}
        ~MadgwickFilter() {}

        // Main functions
        void updateMARGFilter(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z);
        void updateIMUFilter(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);

        // Helper functions

        // Variables
        float a_x, a_y, a_z;
        float w_x, w_y, w_z;
        float m_x, m_y, m_z;
        float q_1, q_2, q_3, q_4;
        float b_x = 1.0, b_z = 0;
        float w_bx = 0, w_by = 0, w_bz = 0;
};