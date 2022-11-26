#include "csv_reader.hpp"

/**
 * @brief Read file contents into vector line by line.
 *
 * The format of the file is consistent with a ',' used as a delimiter. This function essentailly tokenizes the data line by line.
 */
void CsvReader::retrieveFileItems()
{
    std::fstream afile("../test_data/imu_acc.csv", std::ios::in);
    // std::fstream afile(sPath, std::ios::in);
    std::string aline;
    if (afile.is_open())
    {
        while (getline(afile, aline))
        {
            std::vector<float> a_split = split(aline, ',');
            a.push_back(Vec3(a_split.at(0), a_split.at(1), a_split.at(2)));
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
            std::vector<float> g_split = split(gline, ',');
            w.push_back(Vec3(g_split.at(0), g_split.at(1), g_split.at(2)));
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
            std::vector<float> m_split = split(mline, ',');
            m.push_back(Vec3(m_split.at(0), m_split.at(1), m_split.at(2)));
        }
    }
    else
    {
        throw std::runtime_error("Error: failed to open file");
    }

    std::fstream file("../results/opt_quat.csv", std::ios::in);
    std::string line;
    if (file.is_open())
    {
        while (getline(file, line))
        {
            std::vector<float> gt_split = split(line, ',');
            gt.push_back(Quaternion(gt_split.at(0), gt_split.at(1), gt_split.at(2), gt_split.at(3)));
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