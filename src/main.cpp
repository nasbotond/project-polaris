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

        for (int i = 0 ; i < read.a.size(); ++i)
        // for (int i = 0 ; i < 100; ++i)
        {
            comp.updateFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2), read.m.at(i).at(0), read.m.at(i).at(1), read.m.at(i).at(2));
            madg.updateMARGFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2), read.m.at(i).at(0), read.m.at(i).at(1), read.m.at(i).at(2));
        }
        // comp.updateFilter(-0.0011, -0.0043, 0.0053, -0.2362, -0.3632, 9.9193, 0.2226, 15.8218, -38.3379);
        // madg.updateMARGFilter(-0.0011, -0.0043, 0.0053, -0.2362, -0.3632, 9.9193, 0.0, 0.0 ,0.0);
        float cyaw = atan2(2*comp.q_2*comp.q_3-2*comp.q_1*comp.q_4, 2*comp.q_1 *comp.q_1+2*comp.q_2*comp.q_2-1);
        float croll = -asin(2*comp.q_2*comp.q_4+2*comp.q_1*comp.q_3);
        float cpitch = atan2(2*comp.q_3*comp.q_4-2*comp.q_1*comp.q_2, 2*comp.q_1*comp.q_1 + 2*comp.q_4*comp.q_4-1);
        std::cout << "COMP" << std::endl;
        std::cout << "y: " << cyaw << std::endl;
        std::cout << "p: " << croll << std::endl;
        std::cout << "r: " << cpitch << std::endl;
        // std::cout << "q1: " << comp.q_1 << std::endl;
        // std::cout << "q2: " << comp.q_2 << std::endl;
        // std::cout << "q3: " << comp.q_3 << std::endl;
        // std::cout << "q4: " << comp.q_4 << std::endl;

        float myaw = atan2(2*madg.q_2*madg.q_3-2*madg.q_1*madg.q_4, 2*madg.q_1 *madg.q_1+2*madg.q_2*madg.q_2-1);
        float mroll = -asin(2*madg.q_2*madg.q_4+2*madg.q_1*madg.q_3);
        float mpitch = atan2(2*madg.q_3*madg.q_4-2*madg.q_1*madg.q_2, 2*madg.q_1*madg.q_1 + 2*madg.q_4*madg.q_4-1);
        std::cout << "MADG" << std::endl;
        std::cout << "y: " << myaw << std::endl;
        std::cout << "p: " << mroll << std::endl;
        std::cout << "r: " << mpitch << std::endl;
        // std::cout << "q1: " << madg.q_1 << std::endl;
        // std::cout << "q2: " << madg.q_2 << std::endl;
        // std::cout << "q3: " << madg.q_3 << std::endl;
        // std::cout << "q4: " << madg.q_4 << std::endl;

        float gyaw = atan2(2*read.gt.back().at(1)*read.gt.back().at(2)-2*read.gt.back().at(0)*read.gt.back().at(3), 2*read.gt.back().at(0) *read.gt.back().at(0)+2*read.gt.back().at(1)*read.gt.back().at(1)-1);
        float groll = -asin(2*read.gt.back().at(1)*read.gt.back().at(3)+2*read.gt.back().at(0)*read.gt.back().at(2));
        float gpitch = atan2(2*read.gt.back().at(2)*read.gt.back().at(3)-2*read.gt.back().at(0)*read.gt.back().at(1), 2*read.gt.back().at(0)*read.gt.back().at(0) + 2*read.gt.back().at(3)*read.gt.back().at(3)-1);
        std::cout << "GT" << std::endl;
        std::cout << "y: " << gyaw << std::endl;
        std::cout << "p: " << groll << std::endl;
        std::cout << "r: " << gpitch << std::endl;
        // std::cout << "q1: " << read.gt.back().at(0) << std::endl;
        // std::cout << "q2: " << read.gt.back().at(1) << std::endl;
        // std::cout << "q3: " << read.gt.back().at(2) << std::endl;
        // std::cout << "q4: " << read.gt.back().at(3) << std::endl;
    } 
    catch(const std::exception &e) 
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    return 0;
}