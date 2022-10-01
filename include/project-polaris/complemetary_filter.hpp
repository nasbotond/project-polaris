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

class ComplementaryFilter
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

        ComplementaryFilter(const std::string &sPath, const int &sFps, const int &sWidth, const int &sHeight, const std::string &outputFileName) : sPath(sPath), sFps(sFps), sWidth(sWidth), sHeight(sHeight), outputFileName(outputFileName) {}
        ~ComplementaryFilter() {}

        // Main functions
        void updateFilter(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z);

        // Helper functions
};