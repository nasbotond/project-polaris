#include "main.hpp"

int main(int argc, char* argv[])
{
    GUI::begin();
	return 0;
}

/**
 * @brief Execution time evaluation
 *
 */
// int main(int argc, char* argv[])
// {
//     float deltat = 1.0/286.0;

//     std::vector<Vec3> a = CsvReader::getVec3Data("/Users/botond/Documents/thesis/broad_data/37_disturbed_office_A/imu_acc.csv");
//     std::vector<Vec3> w = CsvReader::getVec3Data("/Users/botond/Documents/thesis/broad_data/37_disturbed_office_A/imu_gyr.csv");
//     std::vector<Vec3> m = CsvReader::getVec3Data("/Users/botond/Documents/thesis/broad_data/37_disturbed_office_A/imu_mag.csv");
//     std::vector<Quaternion> gt = CsvReader::getQuatData("/Users/botond/Documents/thesis/broad_data/37_disturbed_office_A/opt_quat.csv");
//     std::vector<int> mov = CsvReader::get1DData("/Users/botond/Documents/thesis/broad_data/37_disturbed_office_A/movement.csv");

//     Quaternion initial_state = Quaternion::getOrientationFromAccMag(a.at(0), m.at(0)); 
//     Quaternion initial_state_acc = Quaternion::getOrientationFromAcc(a.at(0)); 

//     // Input
//     NaiveFilter naive = NaiveFilter(deltat, 0.0001);
//     NaiveFilter naive_mag = NaiveFilter(deltat, 0.0003);
//     MadgwickFilter madg = MadgwickFilter(deltat, 0.11, 0);
//     MadgwickFilter madg_mag = MadgwickFilter(deltat, 0.08, 0.0003);

//     naive_mag.setInitialState(initial_state);
//     madg_mag.setInitialState(initial_state);
//     naive.setInitialState(initial_state_acc);
//     madg.setInitialState(initial_state_acc);

//     std::cout << a.size() << std::endl;

//     float d = static_cast<float>(a.size());

//         // Get starting timepoint
//     auto start4 = std::chrono::high_resolution_clock::now();
//     for (int i = 0; i < a.size(); ++i)
//     {      
//         madg.updateIMUFilter(w.at(i), a.at(i));
//     }
//     // Get ending timepoint
//     auto stop4 = std::chrono::high_resolution_clock::now();
 
//     auto duration4 = std::chrono::duration_cast<std::chrono::nanoseconds>(stop4 - start4);
 
//     std::cout << "Time taken by madg: "
//          << duration4.count()/d << " nanoseconds" << std::endl;

//     // Get starting timepoint
//     auto start3 = std::chrono::high_resolution_clock::now();
//     for (int i = 0; i < a.size(); ++i)
//     {
//         madg_mag.updateMARGFilter(w.at(i), a.at(i), m.at(i));
//     }
//     // Get ending timepoint
//     auto stop3 = std::chrono::high_resolution_clock::now();
 
//     auto duration3 = std::chrono::duration_cast<std::chrono::nanoseconds>(stop3 - start3);
 
//     std::cout << "Time taken by madg_mag: "
//          << duration3.count()/d << " nanoseconds" << std::endl;

//     // Get starting timepoint
//     auto start2 = std::chrono::high_resolution_clock::now();
//     for (int i = 0; i < a.size(); ++i)
//     {
//         naive.updateFilter(w.at(i), a.at(i));
//     }
//     // Get ending timepoint
//     auto stop2 = std::chrono::high_resolution_clock::now();
 
//     auto duration2 = std::chrono::duration_cast<std::chrono::nanoseconds>(stop2 - start2);
 
//     std::cout << "Time taken by naive: "
//          << duration2.count()/d << " nanoseconds" << std::endl;


//     // Get starting timepoint
//     auto start1 = std::chrono::high_resolution_clock::now();
//     for (int i = 0; i < a.size(); ++i)
//     {
//         naive_mag.updateFilter(w.at(i), a.at(i), m.at(i));
//     }
//     // Get ending timepoint
//     auto stop1 = std::chrono::high_resolution_clock::now();
 
//     auto duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(stop1 - start1);
 
//     std::cout << "Time taken by naive_mag: "
//          << duration1.count()/d << " nanoseconds" << std::endl;

    
//     return 0;
// }

/**
 * @brief Average RMSE calculation
 *
 */
// int main(int argc, char* argv[])
// {
//     std::string sPath = "/Users/botond/Documents/thesis/broad_data/";
//     std::vector<std::vector<float>> grand_row_madg_mag_total( 29 , std::vector<float> (51, 0));
//     std::vector<std::vector<float>> grand_row_madg_mag_inc( 29 , std::vector<float> (51, 0));
//     std::vector<std::vector<float>> grand_row_madg_mag_head( 29 , std::vector<float> (51, 0));

//     std::vector<float> grand_row_madg_total( 29 );
//     std::vector<float> grand_row_madg_inc( 29 );
//     std::vector<float> grand_row_madg_head( 29 );

//     std::vector<float> grand_row_naive_mag_total( 100,0 );
//     std::vector<float> grand_row_naive_mag_inc( 100,0 );
//     std::vector<float> grand_row_naive_mag_head( 100,0 );

//     std::vector<float> grand_row_naive_total( 100,0 );
//     std::vector<float> grand_row_naive_inc( 100,0 );
//     std::vector<float> grand_row_naive_head( 100,0 );

//     for(const auto & entry : std::filesystem::directory_iterator(sPath))
//     {
//         std::string fileName = entry.path().filename().string();

//         if(entry.is_directory())
//         {
//             float deltat = 1.0/286.0;

//             std::cout << fileName << '\n';
//             // std::ofstream total_rmse_madg_mag_deg;
//             // std::ofstream inc_rmse_madg_mag_deg;
//             // std::ofstream head_rmse_madg_mag_deg;

//             std::ofstream total_rmse_madg_no_mag_deg;
//             std::ofstream inc_rmse_madg_no_mag_deg;
//             std::ofstream head_rmse_madg_no_mag_deg;

//             std::vector<Vec3> a = CsvReader::getVec3Data(sPath + fileName + "/imu_acc.csv");
//             std::vector<Vec3> w = CsvReader::getVec3Data(sPath + fileName + "/imu_gyr.csv");
//             std::vector<Vec3> m = CsvReader::getVec3Data(sPath + fileName + "/imu_mag.csv");
//             std::vector<Quaternion> gt = CsvReader::getQuatData(sPath + fileName + "/opt_quat.csv");
//             std::vector<int> mov = CsvReader::get1DData(sPath + fileName + "/movement.csv");

//             Quaternion initial_state = Quaternion::getOrientationFromAccMag(a.at(0), m.at(0)); 
//             Quaternion initial_state_acc = Quaternion::getOrientationFromAcc(a.at(0)); 

//             // total_rmse_madg_mag_deg.open(sPath + fileName + "/total_rmse_madg_mag_deg.csv");
//             // inc_rmse_madg_mag_deg.open(sPath + fileName + "/inc_rmse_madg_mag_deg.csv");
//             // head_rmse_madg_mag_deg.open(sPath + fileName + "/head_rmse_madg_mag_deg.csv");

//             total_rmse_madg_no_mag_deg.open(sPath + fileName + "/total_rmse_madg_no_mag_deg.csv");
//             inc_rmse_madg_no_mag_deg.open(sPath + fileName + "/inc_rmse_madg_no_mag_deg.csv");
//             head_rmse_madg_no_mag_deg.open(sPath + fileName + "/head_rmse_madg_no_mag_deg.csv");

//             // total_rmse_madg_mag_deg << " ";
//             // inc_rmse_madg_mag_deg << " ";
//             // head_rmse_madg_mag_deg << " ";

//             // for(float madg_zeta = 0.0; madg_zeta <= 0.005; madg_zeta=madg_zeta+0.0001)
//             // {
//             //     total_rmse_madg_mag_deg << "," << madg_zeta;
//             //     inc_rmse_madg_mag_deg << "," << madg_zeta;
//             //     head_rmse_madg_mag_deg << "," << madg_zeta;
//             // }
//             // total_rmse_madg_mag_deg << "\n";
//             // inc_rmse_madg_mag_deg << "\n";
//             // head_rmse_madg_mag_deg << "\n";

//             int grand_row = 0;
//             int grand_col = 0;

//             // Madgwick no mag
//             for(float madg_beta = 0.01; madg_beta <= 0.30; madg_beta=madg_beta+0.01)
//             {
//                 total_rmse_madg_no_mag_deg << madg_beta;
//                 inc_rmse_madg_no_mag_deg << madg_beta;
//                 head_rmse_madg_no_mag_deg << madg_beta;

//                 float sum_total_madg_no_mag = 0.0;

//                 float sum_inc_madg_no_mag = 0.0;

//                 float sum_head_madg_no_mag = 0.0;

//                 int rows = 0;

//                 MadgwickFilter madg = MadgwickFilter(deltat, madg_beta, 0.0);
//                 madg.setInitialState(initial_state_acc);

//                 for (int i = 0; i < a.size(); ++i)
//                 {     
//                     madg.updateIMUFilter(w.at(i), a.at(i));

//                     if((!gt.at(i).isNaN()) && (mov.at(i)==1))
//                     {
//                         rows++;
//                         // Convert to ENU from NED
//                         Quaternion enu_madg = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg.q);

//                         Quaternion err_quat_madg_no_mag = Metrics::error_quaternion_earth(gt.at(i), enu_madg);

//                         sum_total_madg_no_mag += Metrics::total_error(err_quat_madg_no_mag)*Metrics::total_error(err_quat_madg_no_mag);

//                         sum_inc_madg_no_mag += Metrics::inclination_error(err_quat_madg_no_mag)*Metrics::inclination_error(err_quat_madg_no_mag);
//                         sum_head_madg_no_mag += Metrics::heading_error(err_quat_madg_no_mag)*Metrics::heading_error(err_quat_madg_no_mag);
//                     }
//                 }

//                 total_rmse_madg_no_mag_deg << "," << sqrt(sum_total_madg_no_mag/rows)*180/M_PI;
//                 inc_rmse_madg_no_mag_deg << "," << sqrt(sum_inc_madg_no_mag/rows)*180/M_PI;
//                 head_rmse_madg_no_mag_deg << "," << sqrt(sum_head_madg_no_mag/rows)*180/M_PI;

//                 grand_row_madg_total.at(grand_row) += sqrt(sum_total_madg_no_mag/rows)*180/M_PI;
//                 grand_row_madg_inc.at(grand_row) += sqrt(sum_inc_madg_no_mag/rows)*180/M_PI;
//                 grand_row_madg_head.at(grand_row) += sqrt(sum_head_madg_no_mag/rows)*180/M_PI;

//                 total_rmse_madg_no_mag_deg << "\n";
//                 inc_rmse_madg_no_mag_deg << "\n";
//                 head_rmse_madg_no_mag_deg << "\n";

//                 grand_row++;
//             }

//             total_rmse_madg_no_mag_deg.close();
//             inc_rmse_madg_no_mag_deg.close();
//             head_rmse_madg_no_mag_deg.close();

//             grand_row = 0;

//             // // Madgwick mag
//             // for(float madg_beta = 0.01; madg_beta <= 0.30; madg_beta=madg_beta+0.01)
//             // {

//             //     total_rmse_madg_no_mag_deg << madg_beta;
//             //     inc_rmse_madg_no_mag_deg << madg_beta;
//             //     head_rmse_madg_no_mag_deg << madg_beta;
//             //     for(float madg_zeta = 0.0; madg_zeta <= 0.005; madg_zeta=madg_zeta+0.0001)
//             //     {
//             //         float sum_total_madg_mag = 0.0;

//             //         float sum_inc_madg_mag = 0.0;

//             //         float sum_head_madg_mag = 0.0;

//             //         int rows = 0;

//             //         MadgwickFilter madg_mag = MadgwickFilter(deltat, madg_beta, madg_zeta);

//             //         madg_mag.setInitialState(initial_state);
                    
//             //         for (int i = 0; i < a.size(); ++i)
//             //         {
//             //             // madg_mag.updateMARGFilter(w.at(i), a.at(i), m.at(i));        

//             //             if((!gt.at(i).isNaN()) && (mov.at(i)==1))
//             //             {
//             //                 rows++;
//             //                 // Convert to ENU from NED
//             //                 Quaternion enu_madg_mag = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg_mag.q);

//             //                 Quaternion err_quat_madg_mag = Metrics::error_quaternion_earth(gt.at(i), enu_madg_mag);

//             //                 sum_total_madg_mag += Metrics::total_error(err_quat_madg_mag)*Metrics::total_error(err_quat_madg_mag);

//             //                 sum_inc_madg_mag += Metrics::inclination_error(err_quat_madg_mag)*Metrics::inclination_error(err_quat_madg_mag);

//             //                 sum_head_madg_mag += Metrics::heading_error(err_quat_madg_mag)*Metrics::heading_error(err_quat_madg_mag);
//             //             }
//             //         }
//             //         total_rmse_madg_mag_deg << "," << sqrt(sum_total_madg_mag/rows)*180/M_PI;
//             //         inc_rmse_madg_mag_deg << "," << sqrt(sum_inc_madg_mag/rows)*180/M_PI;
//             //         head_rmse_madg_mag_deg << "," << sqrt(sum_head_madg_mag/rows)*180/M_PI;


//             //         grand_row_madg_mag_total.at(grand_row).at(grand_col) += sqrt(sum_total_madg_mag/rows)*180/M_PI;
//             //         grand_row_madg_mag_inc.at(grand_row).at(grand_col) += sqrt(sum_inc_madg_mag/rows)*180/M_PI;
//             //         grand_row_madg_mag_head.at(grand_row).at(grand_col) += sqrt(sum_head_madg_mag/rows)*180/M_PI;

//             //         grand_col++;
//             //     }
//             //     total_rmse_madg_mag_deg << "\n";
//             //     inc_rmse_madg_mag_deg << "\n";
//             //     head_rmse_madg_mag_deg << "\n";

//             //     grand_row++;
//             //     grand_col = 0;
//             // }
//             // total_rmse_madg_mag_deg.close();
//             // inc_rmse_madg_mag_deg.close();
//             // head_rmse_madg_mag_deg.close();

//             // Naive

//             // std::ofstream total_rmse_naive_mag_deg;
//             // std::ofstream inc_rmse_naive_mag_deg;
//             // std::ofstream head_rmse_naive_mag_deg;

//             std::ofstream total_rmse_naive_no_mag_deg;
//             std::ofstream inc_rmse_naive_no_mag_deg;
//             std::ofstream head_rmse_naive_no_mag_deg;

//             // total_rmse_naive_mag_deg.open(sPath + fileName + "/total_rmse_naive_mag_deg.csv");
//             // inc_rmse_naive_mag_deg.open(sPath + fileName + "/inc_rmse_naive_mag_deg.csv");
//             // head_rmse_naive_mag_deg.open(sPath + fileName + "/head_rmse_naive_mag_deg.csv");

//             total_rmse_naive_no_mag_deg.open(sPath + fileName + "/total_rmse_naive_no_mag_deg.csv");
//             inc_rmse_naive_no_mag_deg.open(sPath + fileName + "/inc_rmse_naive_no_mag_deg.csv");
//             head_rmse_naive_no_mag_deg.open(sPath + fileName + "/head_rmse_naive_no_mag_deg.csv");

//             int grand_index = 0;

//             for(float naive_gain = 0.0001; naive_gain <= 0.01; naive_gain=naive_gain+0.0001)
//             {
//                 // total_rmse_naive_mag_deg << naive_gain;
//                 // inc_rmse_naive_mag_deg << naive_gain;
//                 // head_rmse_naive_mag_deg << naive_gain;

//                 total_rmse_naive_no_mag_deg << naive_gain;
//                 inc_rmse_naive_no_mag_deg << naive_gain;
//                 head_rmse_naive_no_mag_deg << naive_gain;

//                 // float sum_total_naive_mag = 0.0;
//                 float sum_total_naive_no_mag = 0.0;

//                 // float sum_inc_naive_mag = 0.0;
//                 float sum_inc_naive_no_mag = 0.0;

//                 // float sum_head_naive_mag = 0.0;
//                 float sum_head_naive_no_mag = 0.0;

//                 int rows = 0;

//                 NaiveFilter naive = NaiveFilter(deltat, naive_gain);
//                 // NaiveFilter naive_mag = NaiveFilter(deltat, naive_gain);

//                 // naive_mag.setInitialState(initial_state);
//                 naive.setInitialState(initial_state_acc);

//                 for (int i = 0; i < a.size(); ++i)
//                 {
//                     // naive_mag.updateFilter(w.at(i), a.at(i), m.at(i));        
//                     naive.updateFilter(w.at(i), a.at(i));

//                     if((!gt.at(i).isNaN()) && (mov.at(i)==1))
//                     {
//                         rows++;
//                         // Convert to ENU from NED
//                         // Quaternion enu_naive_mag = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), naive_mag.q);
//                         Quaternion enu_naive = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), naive.q);

//                         // Quaternion err_quat_naive_mag = Metrics::error_quaternion_earth(gt.at(i), enu_naive_mag);
//                         Quaternion err_quat_naive_no_mag = Metrics::error_quaternion_earth(gt.at(i), enu_naive);

//                         // sum_total_naive_mag += Metrics::total_error(err_quat_naive_mag)*Metrics::total_error(err_quat_naive_mag);
//                         sum_total_naive_no_mag += Metrics::total_error(err_quat_naive_no_mag)*Metrics::total_error(err_quat_naive_no_mag);

//                         // sum_inc_naive_mag += Metrics::inclination_error(err_quat_naive_mag)*Metrics::inclination_error(err_quat_naive_mag);
//                         sum_inc_naive_no_mag += Metrics::inclination_error(err_quat_naive_no_mag)*Metrics::inclination_error(err_quat_naive_no_mag);

//                         // sum_head_naive_mag += Metrics::heading_error(err_quat_naive_mag)*Metrics::heading_error(err_quat_naive_mag);
//                         sum_head_naive_no_mag += Metrics::heading_error(err_quat_naive_no_mag)*Metrics::heading_error(err_quat_naive_no_mag);
//                     }
//                 }
//                 // total_rmse_naive_mag_deg << "," << sqrt(sum_total_naive_mag/rows)*180/M_PI << "\n";
//                 // inc_rmse_naive_mag_deg << "," << sqrt(sum_inc_naive_mag/rows)*180/M_PI << "\n";
//                 // head_rmse_naive_mag_deg << "," << sqrt(sum_head_naive_mag/rows)*180/M_PI << "\n";

//                 total_rmse_naive_no_mag_deg << "," << sqrt(sum_total_naive_no_mag/rows)*180/M_PI << "\n";
//                 inc_rmse_naive_no_mag_deg << "," << sqrt(sum_inc_naive_no_mag/rows)*180/M_PI << "\n";
//                 head_rmse_naive_no_mag_deg << "," << sqrt(sum_head_naive_no_mag/rows)*180/M_PI << "\n";

//                 // grand_row_naive_mag_total.at(grand_index) += sqrt(sum_total_naive_mag/rows)*180/M_PI;
//                 // grand_row_naive_mag_inc.at(grand_index) += sqrt(sum_inc_naive_mag/rows)*180/M_PI;
//                 // grand_row_naive_mag_head.at(grand_index) += sqrt(sum_head_naive_mag/rows)*180/M_PI;

//                 grand_row_naive_total.at(grand_index) += sqrt(sum_total_naive_no_mag/rows)*180/M_PI;
//                 grand_row_naive_inc.at(grand_index) += sqrt(sum_inc_naive_no_mag/rows)*180/M_PI;
//                 grand_row_naive_head.at(grand_index) += sqrt(sum_head_naive_no_mag/rows)*180/M_PI;
//                 grand_index++;
//             }
//             // total_rmse_naive_mag_deg.close();
//             // inc_rmse_naive_mag_deg.close();
//             // head_rmse_naive_mag_deg.close();

//             total_rmse_naive_no_mag_deg.close();
//             inc_rmse_naive_no_mag_deg.close();
//             head_rmse_naive_no_mag_deg.close();
//         }
//     }

//     // std::ofstream mmt;
//     // std::ofstream mmi;
//     // std::ofstream mmh;

//     std::ofstream mt;
//     std::ofstream mi;
//     std::ofstream mh;

//     // std::ofstream cmt;
//     // std::ofstream cmi;
//     // std::ofstream cmh;

//     std::ofstream ct;
//     std::ofstream ci;
//     std::ofstream ch;

//     // mmt.open("/Users/botond/Documents/thesis/broad_data/mmt.csv");
//     // mmi.open("/Users/botond/Documents/thesis/broad_data/mmi.csv");
//     // mmh.open("/Users/botond/Documents/thesis/broad_data/mmh.csv");

//     mt.open("/Users/botond/Documents/thesis/broad_data/mt.csv");
//     mi.open("/Users/botond/Documents/thesis/broad_data/mi.csv");
//     mh.open("/Users/botond/Documents/thesis/broad_data/mh.csv");

//     // cmt.open("/Users/botond/Documents/thesis/broad_data/cmt.csv");
//     // cmi.open("/Users/botond/Documents/thesis/broad_data/cmi.csv");
//     // cmh.open("/Users/botond/Documents/thesis/broad_data/cmh.csv");

//     ct.open("/Users/botond/Documents/thesis/broad_data/ct.csv");
//     ci.open("/Users/botond/Documents/thesis/broad_data/ci.csv");
//     ch.open("/Users/botond/Documents/thesis/broad_data/ch.csv");

//     float divide_by = 39.0;

//     for(int i = 0; i < 29; ++i)
//     {
//         mt << (i+1.0)/100.0 << "," << grand_row_madg_total.at(i)/divide_by << "\n";
//         mi << (i+1.0)/100.0 << "," << grand_row_madg_inc.at(i)/divide_by << "\n";
//         mh << (i+1.0)/100.0 << "," << grand_row_madg_head.at(i)/divide_by << "\n";
//     }

//     // for(int i = 0; i < 29; ++i)
//     // {
//     //     for(int j = 0; j < 51; ++j)
//     //     {
//     //         mmt << (i+1.0)/100.0 << "," << (j)/10000.0 << "," << grand_row_madg_mag_total.at(i).at(j)/divide_by << "\n";
//     //         mmi << (i+1.0)/100.0 << "," << (j)/10000.0 << "," << grand_row_madg_mag_inc.at(i).at(j)/divide_by << "\n";
//     //         mmh << (i+1.0)/100.0 << "," << (j)/10000.0 << "," << grand_row_madg_mag_head.at(i).at(j)/divide_by << "\n";
//     //     }
//     // }

//     for(int i = 0; i < 100; ++i)
//     {
//         // cmt << (i+1.0)/10000.0 << "," << grand_row_naive_mag_total.at(i)/divide_by << "\n";
//         // cmi << (i+1.0)/10000.0 << "," << grand_row_naive_mag_inc.at(i)/divide_by << "\n";
//         // cmh << (i+1.0)/10000.0 << "," << grand_row_naive_mag_head.at(i)/divide_by << "\n";

//         ct << (i+1.0)/10000.0 << "," << grand_row_naive_total.at(i)/divide_by << "\n";
//         ci << (i+1.0)/10000.0 << "," << grand_row_naive_inc.at(i)/divide_by << "\n";
//         ch << (i+1.0)/10000.0 << "," << grand_row_naive_head.at(i)/divide_by << "\n";
//     }

//     // mmt.close();
//     // mmi.close();
//     // mmh.close();

//     mt.close();
//     mi.close();
//     mh.close();

//     // cmt.close();
//     // cmi.close();
//     // cmh.close();

//     ct.close();
//     ci.close();
//     ch.close();

//     return 0;
// }