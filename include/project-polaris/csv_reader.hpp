#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <numeric>
#include <vector>
#include "quaternion.hpp"
#include "vec3.hpp"

class CsvReader
{
    private:
        std::string sPath;

    public:

        CsvReader(const std::string &sPath) : sPath(sPath) {}
        ~CsvReader() {}

        // Main functions
        void retrieveFileItems();

        std::vector<Vec3> getRMSE();

        // Helper functions
        std::vector<float> split(const std::string &s, char delim) const;

        // Variables
        std::vector<Vec3> a;
        std::vector<Vec3> w;
        std::vector<Vec3> m;
        std::vector<Quaternion> gt;
};