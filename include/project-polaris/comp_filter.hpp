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
#include <math.h>
#define PI 3.14159265

#define deltat 0.0035f
#define gyroMeasError 3.14159265358979 * (5.0/180.0)
#define gain 0.5 //sqrt(3.0/4.0) * gyroMeasError

class ComplementaryFilter
{
    private:
        // std::string sPath;
        // int sFps;
        // int sWidth;
        // int sHeight;
        // std::string outputFileName;

    public:

        // ComplementaryFilter(const std::string &sPath, const int &sFps, const int &sWidth, const int &sHeight, const std::string &outputFileName) : sPath(sPath), sFps(sFps), sWidth(sWidth), sHeight(sHeight), outputFileName(outputFileName) {}
        ComplementaryFilter() : q_1(1.0f), q_2(0.0f), q_3(0.0f), q_4(0.0f) {}
        ~ComplementaryFilter() {}

        // Main functions
        void updateFilter(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z);

        // Helper functions

        // Variables
        float a_x, a_y, a_z;
        float w_x, w_y, w_z;
        float m_x, m_y, m_z;
        float q_1, q_2, q_3, q_4;
};