#pragma once

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
    public:

        /**
         * @brief Tokenize string using provided delimiter.
         *
         * @param s
         * @param delim
         * @return std::vector<std::string>
         */
        static std::vector<float> split(const std::string &s, char delim)
        {
            std::vector<float> result;
            std::stringstream ss(s);
            std::string item;

            while(getline(ss, item, delim))
            {
                if(item.length() < 1)
                {
                    result.push_back(0.0);
                }
                else
                {
                    result.push_back(std::stof(item));
                }
            }
            return result;
        }

        /**
         * @brief Read Vec3 file contents into vector line by line.
         *
         * The format of the file is consistent with a ',' used as a delimiter. This function essentailly tokenizes the data line by line.
         */
        static std::vector<Vec3> getVec3Data(const std::string &path)
        {
            std::vector<Vec3> result;

            std::fstream file(path, std::ios::in);
            // std::fstream afile(path, std::ios::in);
            std::string line;
            if(file.is_open())
            {
                while(getline(file, line))
                {
                    std::vector<float> s = split(line, ',');
                    result.push_back(Vec3(s.at(0), s.at(1), s.at(2)));
                }
            }
            else
            {
                throw std::runtime_error("Error: failed to open file");
            }
            return result;
        }

        /**
         * @brief Read gt file contents into vector line by line.
         *
         * The format of the file is consistent with a ',' used as a delimiter. This function essentailly tokenizes the data line by line.
         */
        static std::vector<Quaternion> getQuatData(const std::string &path)
        {
            std::vector<Quaternion> gt;
            std::fstream file(path, std::ios::in);
            std::string line;
            if(file.is_open())
            {
                while(getline(file, line))
                {
                    std::vector<float> gt_split = split(line, ',');
                    gt.push_back(Quaternion(gt_split.at(0), gt_split.at(1), gt_split.at(2), gt_split.at(3)));
                }
            }
            else
            {
                throw std::runtime_error("Error: failed to open file");
            }
            return gt;
        }

        /**
         * @brief Read 1D file contents into vector line by line.
         *
         * The format of the file is consistent with a ',' used as a delimiter. This function essentailly tokenizes the data line by line.
         */
        static std::vector<int> get1DData(const std::string &path)
        {
            std::vector<int> result;

            std::fstream file(path, std::ios::in);
            // std::fstream afile(path, std::ios::in);
            std::string line;
            if(file.is_open())
            {
                while(getline(file, line))
                {
                    std::vector<float> s = split(line, ',');
                    result.push_back((int)s.at(0));
                }
            }
            else
            {
                throw std::runtime_error("Error: failed to open file");
            }
            return result;
        }      

        static std::vector<Vec3> getRMSE(const std::string &path, std::string &results_suffix)
        {
            std::vector<Vec3> result = std::vector<Vec3>(4);
            int rows = 0;
            float sum_total_madg_mag = 0.0;
            float sum_total_madg_no_mag = 0.0;
            float sum_total_naive_mag = 0.0;
            float sum_total_naive_no_mag = 0.0;

            float sum_inc_madg_mag = 0.0;
            float sum_inc_madg_no_mag = 0.0;
            float sum_inc_naive_mag = 0.0;
            float sum_inc_naive_no_mag = 0.0;

            float sum_head_madg_mag = 0.0;
            float sum_head_madg_no_mag = 0.0;
            float sum_head_naive_mag = 0.0;
            float sum_head_naive_no_mag = 0.0;

            std::fstream afile(path + results_suffix + "/error_madg_mag.csv", std::ios::in);
            std::string aline;
            if(afile.is_open())
            {
                while(getline(afile, aline))
                {
                    std::vector<float> a_split = split(aline, ',');
                    if(a_split.at(0) != 9999)
                    {
                        rows++;
                        sum_total_madg_mag += a_split.at(0)*a_split.at(0);
                        sum_inc_madg_mag += a_split.at(1)*a_split.at(1);
                        sum_head_madg_mag += a_split.at(2)*a_split.at(2);
                    }
                }
            }
            else
            {
                throw std::runtime_error("Error: failed to open error_madg_mag file");
            }

            std::fstream wfile(path + results_suffix + "/error_madg_no_mag.csv", std::ios::in);
            std::string gline;
            if(wfile.is_open())
            {
                while (getline(wfile, gline))
                {
                    std::vector<float> g_split = split(gline, ',');
                    if(g_split.at(0) != 9999)
                    {
                        sum_total_madg_no_mag += g_split.at(0)*g_split.at(0);
                        sum_inc_madg_no_mag += g_split.at(1)*g_split.at(1);
                        sum_head_madg_no_mag += g_split.at(2)*g_split.at(2);
                    }
                }
            }
            else
            {
                throw std::runtime_error("Error: failed to open error_madg_no_mag file");
            }

            std::fstream mfile(path + results_suffix + "/error_naive_mag.csv", std::ios::in);
            std::string mline;
            if(mfile.is_open())
            {
                while (getline(mfile, mline))
                {
                    std::vector<float> m_split = split(mline, ',');
                    if(m_split.at(0) != 9999)
                    {
                        sum_total_naive_mag += m_split.at(0)*m_split.at(0);
                        sum_inc_naive_mag += m_split.at(1)*m_split.at(1);
                        sum_head_naive_mag += m_split.at(2)*m_split.at(2);
                    }
                }
            }
            else
            {
                throw std::runtime_error("Error: failed to open error_naive_mag file");
            }

            std::fstream file(path + results_suffix + "/error_naive_no_mag.csv", std::ios::in);
            std::string line;
            if(file.is_open())
            {
                while(getline(file, line))
                {
                    std::vector<float> gt_split = split(line, ',');
                    if(gt_split.at(0) != 9999)
                    {
                        sum_total_naive_no_mag += gt_split.at(0)*gt_split.at(0);
                        sum_inc_naive_no_mag += gt_split.at(1)*gt_split.at(1);
                        sum_head_naive_no_mag += gt_split.at(2)*gt_split.at(2);
                    }
                }
            }
            else
            {
                throw std::runtime_error("Error: failed to open error_naive_no_mag file");
            }

            result.at(0) = Vec3(sqrt(sum_total_madg_mag/rows), sqrt(sum_inc_madg_mag/rows), sqrt(sum_head_madg_mag/rows));
            result.at(1) = Vec3(sqrt(sum_total_madg_no_mag/rows), sqrt(sum_inc_madg_no_mag/rows), sqrt(sum_head_madg_no_mag/rows));
            result.at(2) = Vec3(sqrt(sum_total_naive_mag/rows), sqrt(sum_inc_naive_mag/rows), sqrt(sum_head_naive_mag/rows));
            result.at(3) = Vec3(sqrt(sum_total_naive_no_mag/rows), sqrt(sum_inc_naive_no_mag/rows), sqrt(sum_head_naive_no_mag/rows));

            std::ofstream rmse_out;
            rmse_out.open (path + results_suffix + "/rmse_out.csv");
            for(int i = 0; i < result.size(); ++i)
            {  
                rmse_out << result.at(i).to_csv_string() << "\n";
            }
            rmse_out.close();

            return result;
        }
};