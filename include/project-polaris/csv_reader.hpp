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

class CsvReader
{
    private:
        std::string sPath;

    public:

        CsvReader(const std::string &sPath) : sPath(sPath) {}
        ~CsvReader() {}

        // Main functions
        void retrieveFileItems();

        // Helper functions
        std::vector<float> split(const std::string &s, char delim) const;

        // Variables
        std::vector<std::vector<float>> a;
        std::vector<std::vector<float>> w;
        std::vector<std::vector<float>> m;
        std::vector<std::vector<float>> gt;
};