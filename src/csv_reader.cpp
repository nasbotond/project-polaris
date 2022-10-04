#include "csv_reader.hpp"

/**
 * @brief Read file contents into vector line by line.
 *
 * The format of the file is consistent with a ';' used as a delimiter. This function essentailly tokenizes the data line by line.
 */
void CsvReader::retrieveFileItems()
{
    std::fstream afile("../test_data/imu_acc.csv", std::ios::in);
    std::string aline;
    if (afile.is_open())
    {
        while (getline(afile, aline))
        {
            this->a.push_back(split(aline, ','));
        }
    }
    else
    {
        throw std::runtime_error("Error: failed to open file");
    }

    std::fstream wfile("../test_data/imu_gyr.csv", std::ios::in);
    std::string gline;
    if (wfile.is_open())
    {
        while (getline(wfile, gline))
        {
            this->w.push_back(split(gline, ','));
        }
    }
    else
    {
        throw std::runtime_error("Error: failed to open file");
    }

    std::fstream mfile("../test_data/imu_mag.csv", std::ios::in);
    std::string mline;
    if (mfile.is_open())
    {
        while (getline(mfile, mline))
        {
            this->m.push_back(split(mline, ','));
        }
    }
    else
    {
        throw std::runtime_error("Error: failed to open file");
    }

    std::fstream file("../test_data/opt_quat.csv", std::ios::in);
    std::string line;
    if (file.is_open())
    {
        while (getline(file, line))
        {
            this->gt.push_back(split(line, ','));
        }
    }
    else
    {
        throw std::runtime_error("Error: failed to open file");
    }
}

/**
 * @brief Tokenize string using provided delimiter.
 *
 * @param s
 * @param delim
 * @return std::vector<std::string>
 */
std::vector<float> CsvReader::split(const std::string &s, char delim) const
{
    std::vector<float> result;
    std::stringstream ss(s);
    std::string item;

    while (getline(ss, item, delim))
    {
        if (item.length() < 1)
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