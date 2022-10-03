#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string>

#include "madgwick_filter.hpp"
#include "comp_filter.hpp"
#include "csv_reader.hpp"

int main(int argc, char* argv[]) 
{
    // if (argc < 6 || argc > 7) 
    // {
    //     std::cerr << "Usage: " << argv[0] << " <path to dir of images> <fps> <image width> <image height> <output file prefix> [camera index]" << std::endl;
    //     return 1;
    // }

    // std::string sPath = argv[1];
    // int sFps = std::stoi(argv[2]);
    // int sWidth = std::stoi(argv[3]);
    // int sHeight = std::stoi(argv[4]);
    // std::string outputFileName = argv[5];

    // // Check if path is valid
    // if(!std::filesystem::exists(sPath)) 
    // {
    //     std::cout << "Path is not valid!" << std::endl;
    //     return 1;
    // }

    // std::cout << "------------------ Parameters -------------------" << std::endl;
    // std::cout << "Path = " << sPath << std::endl;
    // std::cout << "FPS = " << sFps << std::endl;
    // std::cout << "Image width = " << sWidth << std::endl;
    // std::cout << "Image height = " << sHeight << std::endl;
    // std::cout << "Output file prefix = " << outputFileName << std::endl;
    // std::cout << "-------------------------------------------------" << std::endl;

    // Input
    ComplementaryFilter comp = ComplementaryFilter();
    MadgwickFilter madg = MadgwickFilter();
    CsvReader read = CsvReader();
    
    try 
    {
        read.retrieveFileItems();
        std::cout << read.a.at(0).at(1) << std::endl;
        std::cout << read.a.size() << std::endl;
        comp.updateFilter(-0.0011, -0.0043, 0.0053, -0.2362, -0.3632, 9.9193, 0.2226, 15.8218, -38.3379);
        madg.updateMARGFilter(-0.0011, -0.0043, 0.0053, -0.2362, -0.3632, 9.9193, 0.0, 0.0 ,0.0);
    } 
    catch(const std::exception &e) 
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    return 0;
}