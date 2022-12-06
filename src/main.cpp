#include "main.hpp"
namespace fs = std::filesystem;
#define sPath "/Users/botond/Documents/thesis/data/"

int main(int argc, char* argv[])
{
    GUI::begin();
	return 0;
}

/**
 * @brief Average RMSE calculation
 *
 */
// int main(int argc, char* argv[])
// {
//     std::vector<std::vector<float>> grand_row_madg_mag_total( 29 , std::vector<float> (51, 0));
//     std::vector<std::vector<float>> grand_row_madg_mag_inc( 29 , std::vector<float> (51, 0));
//     std::vector<std::vector<float>> grand_row_madg_mag_head( 29 , std::vector<float> (51, 0));

//     std::vector<std::vector<float>> grand_row_madg_total( 29 , std::vector<float> (51, 0));
//     std::vector<std::vector<float>> grand_row_madg_inc( 29 , std::vector<float> (51, 0));
//     std::vector<std::vector<float>> grand_row_madg_head( 29 , std::vector<float> (51, 0));

//     std::vector<float> grand_row_comp_mag_total( 100,0 );
//     std::vector<float> grand_row_comp_mag_inc( 100,0 );
//     std::vector<float> grand_row_comp_mag_head( 100,0 );

//     std::vector<float> grand_row_comp_total( 100,0 );
//     std::vector<float> grand_row_comp_inc( 100,0 );
//     std::vector<float> grand_row_comp_head( 100,0 );

//     for(const auto & entry : fs::directory_iterator(sPath))
//     {
//         std::string fileName = entry.path().filename().string();

//         if(entry.is_directory())
//         {
//             float deltat = 1.0/286.0;

//             std::cout << fileName << '\n';
//             std::ofstream total_rmse_madg_mag_deg;
//             std::ofstream inc_rmse_madg_mag_deg;
//             std::ofstream head_rmse_madg_mag_deg;

//             std::ofstream total_rmse_madg_no_mag_deg;
//             std::ofstream inc_rmse_madg_no_mag_deg;
//             std::ofstream head_rmse_madg_no_mag_deg;

//             std::vector<Vec3> a = CsvReader::getVec3Data(sPath + fileName + "/imu_acc.csv");
//             std::vector<Vec3> w = CsvReader::getVec3Data(sPath + fileName + "/imu_gyr.csv");
//             std::vector<Vec3> m = CsvReader::getVec3Data(sPath + fileName + "/imu_mag.csv");
//             std::vector<Quaternion> gt = CsvReader::getQuatData(sPath + fileName + "/opt_quat.csv");
//             std::vector<int> mov = CsvReader::get1DData(sPath + fileName + "/movement.csv");

//             Quaternion initial_state = Quaternion::getOrientationFromAccMag(a.at(0), m.at(0)); 

//             total_rmse_madg_mag_deg.open(sPath + fileName + "/total_rmse_madg_mag_deg.csv");
//             inc_rmse_madg_mag_deg.open(sPath + fileName + "/inc_rmse_madg_mag_deg.csv");
//             head_rmse_madg_mag_deg.open(sPath + fileName + "/head_rmse_madg_mag_deg.csv");

//             total_rmse_madg_no_mag_deg.open(sPath + fileName + "/total_rmse_madg_no_mag_deg.csv");
//             inc_rmse_madg_no_mag_deg.open(sPath + fileName + "/inc_rmse_madg_no_mag_deg.csv");
//             head_rmse_madg_no_mag_deg.open(sPath + fileName + "/head_rmse_madg_no_mag_deg.csv");

//             total_rmse_madg_mag_deg << " ";
//             inc_rmse_madg_mag_deg << " ";
//             head_rmse_madg_mag_deg << " ";

//             total_rmse_madg_no_mag_deg << " ";
//             inc_rmse_madg_no_mag_deg << " ";
//             head_rmse_madg_no_mag_deg << " ";

//             for(float madg_zeta = 0.0; madg_zeta <= 0.005; madg_zeta=madg_zeta+0.0001)
//             {
//                 total_rmse_madg_mag_deg << "," << madg_zeta;
//                 inc_rmse_madg_mag_deg << "," << madg_zeta;
//                 head_rmse_madg_mag_deg << "," << madg_zeta;

//                 total_rmse_madg_no_mag_deg << "," << madg_zeta;
//                 inc_rmse_madg_no_mag_deg << "," << madg_zeta;
//                 head_rmse_madg_no_mag_deg << "," << madg_zeta;
//             }
//             total_rmse_madg_mag_deg << "\n";
//             inc_rmse_madg_mag_deg << "\n";
//             head_rmse_madg_mag_deg << "\n";

//             total_rmse_madg_no_mag_deg << "\n";
//             inc_rmse_madg_no_mag_deg << "\n";
//             head_rmse_madg_no_mag_deg << "\n";

//             int grand_row = 0;
//             int grand_col = 0;

//             // Madgwick
//             for(float madg_beta = 0.01; madg_beta <= 0.30; madg_beta=madg_beta+0.01)
//             {
//                 total_rmse_madg_mag_deg << madg_beta;
//                 inc_rmse_madg_mag_deg << madg_beta;
//                 head_rmse_madg_mag_deg << madg_beta;

//                 total_rmse_madg_no_mag_deg << madg_beta;
//                 inc_rmse_madg_no_mag_deg << madg_beta;
//                 head_rmse_madg_no_mag_deg << madg_beta;
//                 for(float madg_zeta = 0.0; madg_zeta <= 0.005; madg_zeta=madg_zeta+0.0001)
//                 {
//                     float sum_total_madg_mag = 0.0;
//                     float sum_total_madg_no_mag = 0.0;

//                     float sum_inc_madg_mag = 0.0;
//                     float sum_inc_madg_no_mag = 0.0;

//                     float sum_head_madg_mag = 0.0;
//                     float sum_head_madg_no_mag = 0.0;

//                     int rows = 0;

//                     MadgwickFilter madg = MadgwickFilter(deltat, madg_beta, madg_zeta);
//                     MadgwickFilter madg_mag = MadgwickFilter(deltat, madg_beta, madg_zeta);

//                     madg_mag.setInitialState(initial_state);
//                     madg.setInitialState(initial_state);
                    
//                     for (int i = 0; i < a.size(); ++i)
//                     {
//                         madg_mag.updateMARGFilter(w.at(i), a.at(i), m.at(i));        
//                         madg.updateIMUFilter(w.at(i), a.at(i));

//                         if((!gt.at(i).isNaN()) && (mov.at(i)==1))
//                         {
//                             rows++;
//                             // Convert to ENU from NED
//                             Quaternion enu_madg_mag = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg_mag.q);
//                             Quaternion enu_madg = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg.q);

//                             Quaternion err_quat_madg_mag = Metrics::error_quaternion_earth(gt.at(i), enu_madg_mag);
//                             Quaternion err_quat_madg_no_mag = Metrics::error_quaternion_earth(gt.at(i), enu_madg);

//                             sum_total_madg_mag += Metrics::total_error(err_quat_madg_mag)*Metrics::total_error(err_quat_madg_mag);
//                             sum_total_madg_no_mag += Metrics::total_error(err_quat_madg_no_mag)*Metrics::total_error(err_quat_madg_no_mag);

//                             sum_inc_madg_mag += Metrics::inclination_error(err_quat_madg_mag)*Metrics::inclination_error(err_quat_madg_mag);
//                             sum_inc_madg_no_mag += Metrics::inclination_error(err_quat_madg_no_mag)*Metrics::inclination_error(err_quat_madg_no_mag);

//                             sum_head_madg_mag += Metrics::heading_error(err_quat_madg_mag)*Metrics::heading_error(err_quat_madg_mag);
//                             sum_head_madg_no_mag += Metrics::heading_error(err_quat_madg_no_mag)*Metrics::heading_error(err_quat_madg_no_mag);
//                         }
//                     }
//                     total_rmse_madg_mag_deg << "," << sqrt(sum_total_madg_mag/rows)*180/M_PI;
//                     inc_rmse_madg_mag_deg << "," << sqrt(sum_inc_madg_mag/rows)*180/M_PI;
//                     head_rmse_madg_mag_deg << "," << sqrt(sum_head_madg_mag/rows)*180/M_PI;

//                     total_rmse_madg_no_mag_deg << "," << sqrt(sum_total_madg_no_mag/rows)*180/M_PI;
//                     inc_rmse_madg_no_mag_deg << "," << sqrt(sum_inc_madg_no_mag/rows)*180/M_PI;
//                     head_rmse_madg_no_mag_deg << "," << sqrt(sum_head_madg_no_mag/rows)*180/M_PI;

//                     grand_row_madg_mag_total.at(grand_row).at(grand_col) += sqrt(sum_total_madg_mag/rows)*180/M_PI;
//                     grand_row_madg_mag_inc.at(grand_row).at(grand_col) += sqrt(sum_inc_madg_mag/rows)*180/M_PI;
//                     grand_row_madg_mag_head.at(grand_row).at(grand_col) += sqrt(sum_head_madg_mag/rows)*180/M_PI;

//                     grand_row_madg_total.at(grand_row).at(grand_col) += sqrt(sum_total_madg_no_mag/rows)*180/M_PI;
//                     grand_row_madg_inc.at(grand_row).at(grand_col) += sqrt(sum_inc_madg_no_mag/rows)*180/M_PI;
//                     grand_row_madg_head.at(grand_row).at(grand_col) += sqrt(sum_head_madg_no_mag/rows)*180/M_PI;

//                     grand_col++;
//                 }
//                 total_rmse_madg_mag_deg << "\n";
//                 inc_rmse_madg_mag_deg << "\n";
//                 head_rmse_madg_mag_deg << "\n";

//                 total_rmse_madg_no_mag_deg << "\n";
//                 inc_rmse_madg_no_mag_deg << "\n";
//                 head_rmse_madg_no_mag_deg << "\n";

//                 grand_row++;
//                 grand_col = 0;
//             }
//             total_rmse_madg_mag_deg.close();
//             inc_rmse_madg_mag_deg.close();
//             head_rmse_madg_mag_deg.close();
//             total_rmse_madg_no_mag_deg.close();
//             inc_rmse_madg_no_mag_deg.close();
//             head_rmse_madg_no_mag_deg.close();

//             // Complementary

//             std::ofstream total_rmse_comp_mag_deg;
//             std::ofstream inc_rmse_comp_mag_deg;
//             std::ofstream head_rmse_comp_mag_deg;

//             std::ofstream total_rmse_comp_no_mag_deg;
//             std::ofstream inc_rmse_comp_no_mag_deg;
//             std::ofstream head_rmse_comp_no_mag_deg;

//             total_rmse_comp_mag_deg.open(sPath + fileName + "/total_rmse_comp_mag_deg.csv");
//             inc_rmse_comp_mag_deg.open(sPath + fileName + "/inc_rmse_comp_mag_deg.csv");
//             head_rmse_comp_mag_deg.open(sPath + fileName + "/head_rmse_comp_mag_deg.csv");

//             total_rmse_comp_no_mag_deg.open(sPath + fileName + "/total_rmse_comp_no_mag_deg.csv");
//             inc_rmse_comp_no_mag_deg.open(sPath + fileName + "/inc_rmse_comp_no_mag_deg.csv");
//             head_rmse_comp_no_mag_deg.open(sPath + fileName + "/head_rmse_comp_no_mag_deg.csv");

//             int grand_index = 0;

//             for(float comp_gain = 0.0001; comp_gain <= 0.01; comp_gain=comp_gain+0.0001)
//             {
//                 total_rmse_comp_mag_deg << comp_gain;
//                 inc_rmse_comp_mag_deg << comp_gain;
//                 head_rmse_comp_mag_deg << comp_gain;

//                 total_rmse_comp_no_mag_deg << comp_gain;
//                 inc_rmse_comp_no_mag_deg << comp_gain;
//                 head_rmse_comp_no_mag_deg << comp_gain;

//                 float sum_total_comp_mag = 0.0;
//                 float sum_total_comp_no_mag = 0.0;

//                 float sum_inc_comp_mag = 0.0;
//                 float sum_inc_comp_no_mag = 0.0;

//                 float sum_head_comp_mag = 0.0;
//                 float sum_head_comp_no_mag = 0.0;

//                 int rows = 0;

//                 ComplementaryFilter comp = ComplementaryFilter(deltat, comp_gain);
//                 ComplementaryFilter comp_mag = ComplementaryFilter(deltat, comp_gain);

//                 comp_mag.setInitialState(initial_state);
//                 comp.setInitialState(initial_state);

//                 for (int i = 0; i < a.size(); ++i)
//                 {
//                     comp_mag.updateFilter(w.at(i), a.at(i), m.at(i));        
//                     comp.updateFilter(w.at(i), a.at(i));

//                     if((!gt.at(i).isNaN()) && (mov.at(i)==1))
//                     {
//                         rows++;
//                         // Convert to ENU from NED
//                         Quaternion enu_comp_mag = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), comp_mag.q);
//                         Quaternion enu_comp = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), comp.q);

//                         Quaternion err_quat_comp_mag = Metrics::error_quaternion_earth(gt.at(i), enu_comp_mag);
//                         Quaternion err_quat_comp_no_mag = Metrics::error_quaternion_earth(gt.at(i), enu_comp);

//                         sum_total_comp_mag += Metrics::total_error(err_quat_comp_mag)*Metrics::total_error(err_quat_comp_mag);
//                         sum_total_comp_no_mag += Metrics::total_error(err_quat_comp_no_mag)*Metrics::total_error(err_quat_comp_no_mag);

//                         sum_inc_comp_mag += Metrics::inclination_error(err_quat_comp_mag)*Metrics::inclination_error(err_quat_comp_mag);
//                         sum_inc_comp_no_mag += Metrics::inclination_error(err_quat_comp_no_mag)*Metrics::inclination_error(err_quat_comp_no_mag);

//                         sum_head_comp_mag += Metrics::heading_error(err_quat_comp_mag)*Metrics::heading_error(err_quat_comp_mag);
//                         sum_head_comp_no_mag += Metrics::heading_error(err_quat_comp_no_mag)*Metrics::heading_error(err_quat_comp_no_mag);
//                     }
//                 }
//                 total_rmse_comp_mag_deg << "," << sqrt(sum_total_comp_mag/rows)*180/M_PI << "\n";
//                 inc_rmse_comp_mag_deg << "," << sqrt(sum_inc_comp_mag/rows)*180/M_PI << "\n";
//                 head_rmse_comp_mag_deg << "," << sqrt(sum_head_comp_mag/rows)*180/M_PI << "\n";

//                 total_rmse_comp_no_mag_deg << "," << sqrt(sum_total_comp_no_mag/rows)*180/M_PI << "\n";
//                 inc_rmse_comp_no_mag_deg << "," << sqrt(sum_inc_comp_no_mag/rows)*180/M_PI << "\n";
//                 head_rmse_comp_no_mag_deg << "," << sqrt(sum_head_comp_no_mag/rows)*180/M_PI << "\n";

//                 grand_row_comp_mag_total.at(grand_index) += sqrt(sum_total_comp_mag/rows)*180/M_PI;
//                 grand_row_comp_mag_inc.at(grand_index) += sqrt(sum_inc_comp_mag/rows)*180/M_PI;
//                 grand_row_comp_mag_head.at(grand_index) += sqrt(sum_head_comp_mag/rows)*180/M_PI;

//                 grand_row_comp_total.at(grand_index) += sqrt(sum_total_comp_no_mag/rows)*180/M_PI;
//                 grand_row_comp_inc.at(grand_index) += sqrt(sum_inc_comp_no_mag/rows)*180/M_PI;
//                 grand_row_comp_head.at(grand_index) += sqrt(sum_head_comp_no_mag/rows)*180/M_PI;
//                 grand_index++;
//             }
//             total_rmse_comp_mag_deg.close();
//             inc_rmse_comp_mag_deg.close();
//             head_rmse_comp_mag_deg.close();

//             total_rmse_comp_no_mag_deg.close();
//             inc_rmse_comp_no_mag_deg.close();
//             head_rmse_comp_no_mag_deg.close();
//         }
//     }

//     std::ofstream mmt;
//     std::ofstream mmi;
//     std::ofstream mmh;

//     std::ofstream mt;
//     std::ofstream mi;
//     std::ofstream mh;

//     std::ofstream cmt;
//     std::ofstream cmi;
//     std::ofstream cmh;

//     std::ofstream ct;
//     std::ofstream ci;
//     std::ofstream ch;

//     mmt.open("/Users/botond/Documents/thesis/data/mmt.csv");
//     mmi.open("/Users/botond/Documents/thesis/data/mmi.csv");
//     mmh.open("/Users/botond/Documents/thesis/data/mmh.csv");

//     mt.open("/Users/botond/Documents/thesis/data/mt.csv");
//     mi.open("/Users/botond/Documents/thesis/data/mi.csv");
//     mh.open("/Users/botond/Documents/thesis/data/mh.csv");

//     cmt.open("/Users/botond/Documents/thesis/data/cmt.csv");
//     cmi.open("/Users/botond/Documents/thesis/data/cmi.csv");
//     cmh.open("/Users/botond/Documents/thesis/data/cmh.csv");

//     ct.open("/Users/botond/Documents/thesis/data/ct.csv");
//     ci.open("/Users/botond/Documents/thesis/data/ci.csv");
//     ch.open("/Users/botond/Documents/thesis/data/ch.csv");

//     float divide_by = 39.0;

//     for(int i = 0; i < 29; ++i)
//     {
//         for(int j = 0; j < 51; ++j)
//         {
//             mmt << (i+1.0)/100.0 << "," << (j)/10000.0 << "," << grand_row_madg_mag_total.at(i).at(j)/divide_by << "\n";
//             mmi << (i+1.0)/100.0 << "," << (j)/10000.0 << "," << grand_row_madg_mag_inc.at(i).at(j)/divide_by << "\n";
//             mmh << (i+1.0)/100.0 << "," << (j)/10000.0 << "," << grand_row_madg_mag_head.at(i).at(j)/divide_by << "\n";

//             mt << (i+1.0)/100.0 << "," << (j)/10000.0 << "," << grand_row_madg_total.at(i).at(j)/divide_by << "\n";
//             mi << (i+1.0)/100.0 << "," << (j)/10000.0 << "," << grand_row_madg_inc.at(i).at(j)/divide_by << "\n";
//             mh << (i+1.0)/100.0 << "," << (j)/10000.0 << "," << grand_row_madg_head.at(i).at(j)/divide_by << "\n";
//         }
//     }

//     for(int i = 0; i < 100; ++i)
//     {
//         cmt << (i+1.0)/10000.0 << "," << grand_row_comp_mag_total.at(i)/divide_by << "\n";
//         cmi << (i+1.0)/10000.0 << "," << grand_row_comp_mag_inc.at(i)/divide_by << "\n";
//         cmh << (i+1.0)/10000.0 << "," << grand_row_comp_mag_head.at(i)/divide_by << "\n";

//         ct << (i+1.0)/10000.0 << "," << grand_row_comp_total.at(i)/divide_by << "\n";
//         ci << (i+1.0)/10000.0 << "," << grand_row_comp_inc.at(i)/divide_by << "\n";
//         ch << (i+1.0)/10000.0 << "," << grand_row_comp_head.at(i)/divide_by << "\n";
//     }

//     mmt.close();
//     mmi.close();
//     mmh.close();

//     mt.close();
//     mi.close();
//     mh.close();

//     cmt.close();
//     cmi.close();
//     cmh.close();

//     ct.close();
//     ci.close();
//     ch.close();

//     return 0;
// }