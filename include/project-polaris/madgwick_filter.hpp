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
namespace fs = std::filesystem;

class MadgwickFilter
{
    private:

        float a_x, a_y, a_z;
        float w_x, w_y, w_z;
        float m_x, m_y, m_z;
        float q_1, q_2, q_3, q_4;
        float b_x = 1.0, b_z = 0;
        float w_bx = 0, w_by = 0, w_bz = 0;

        std::string sPath;
        int sFps;
        int sWidth;
        int sHeight;
        std::string outputFileName;

    public:

        MadgwickFilter(const std::string &sPath, const int &sFps, const int &sWidth, const int &sHeight, const std::string &outputFileName) : sPath(sPath), sFps(sFps), sWidth(sWidth), sHeight(sHeight), outputFileName(outputFileName) {}
        ~MadgwickFilter() {}

        // Main functions
        void updateMARGFilter(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z);
        void updateIMUFilter(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);

        // Helper functions
};