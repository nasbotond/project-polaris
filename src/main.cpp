#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string>

#include "madgwick_filter.hpp"
#include "complementary_filter.hpp"

int main(int argc, char* argv[]) 
{
    if (argc < 6 || argc > 7) 
    {
        std::cerr << "Usage: " << argv[0] << " <path to dir of images> <fps> <image width> <image height> <output file prefix> [camera index]" << std::endl;
        return 1;
    }

    std::string sPath = argv[1];
    int sFps = std::stoi(argv[2]);
    int sWidth = std::stoi(argv[3]);
    int sHeight = std::stoi(argv[4]);
    std::string outputFileName = argv[5];

    // Check if path is valid
    if(!std::filesystem::exists(sPath)) 
    {
        std::cout << "Path is not valid!" << std::endl;
        return 1;
    }

    std::cout << "------------------ Parameters -------------------" << std::endl;
    std::cout << "Path = " << sPath << std::endl;
    std::cout << "FPS = " << sFps << std::endl;
    std::cout << "Image width = " << sWidth << std::endl;
    std::cout << "Image height = " << sHeight << std::endl;
    std::cout << "Output file prefix = " << outputFileName << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;

    // Input
    ComplementaryFilter videoGenerator = ComplementaryFilter(sPath, sFps, sWidth, sHeight, outputFileName);
    
    try 
    {

    } 
    catch(const std::exception &e) 
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    return 0;
}