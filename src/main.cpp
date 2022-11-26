#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <array>

#include <vtkCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

#include "madgwick_filter.hpp"
#include "comp_filter.hpp"
#include "csv_reader.hpp"
#include "animation.hpp"
#include "metrics.hpp"

int main(int argc, char* argv[])
{
    float deltat = 1.0f/286.0f;
    float compGain = 0.2;

    // Input
    ComplementaryFilter comp = ComplementaryFilter(deltat, compGain);
    ComplementaryFilter comp_mag = ComplementaryFilter(deltat, compGain);
    MadgwickFilter madg = MadgwickFilter(deltat);
    MadgwickFilter madg_mag = MadgwickFilter(deltat);

    CsvReader read = CsvReader("../test_data/");
    std::vector<std::vector<double>> gravity_vectors;

    std::ofstream est_madg_mag;
    std::ofstream est_madg_no_mag;
    std::ofstream est_comp_mag;
    std::ofstream est_comp_no_mag;

    std::ofstream quat_err;
    std::ofstream e;
    std::ofstream euler_diff;
    
    try
    {
        read.retrieveFileItems();

        est_madg_mag.open ("../results/est_madg_mag.csv");
        est_madg_no_mag.open ("../results/est_madg_no_mag.csv");
        est_comp_mag.open ("../results/est_comp_mag.csv");
        est_comp_no_mag.open ("../results/est_comp_no_mag.csv");

        quat_err.open ("../results/quat_err.csv");
        e.open ("../results/e.csv");

        euler_diff.open ("../results/euler_diff.csv");

        double sum = 0.0;
        double sum_inc = 0.0;
        double sum_head = 0.0;

        // for (int i = 10000; i < 10005; ++i)
        for (int i = 10000; i < 45001; ++i)
        // for (int i = 10000; i < 11801; ++i)
        // for (int i = 10000; i < 41795; ++i)
        {
            // with Magnetometer
            // comp_mag.updateFilter(read.w.at(i), read.a.at(i), read.m.at(i));
            madg_mag.updateMARGFilter(read.w.at(i), read.a.at(i), read.m.at(i));
            // without Magnetometer
            // comp.updateFilter(read.w.at(i), read.a.at(i));
            // madg.updateIMUFilter(read.w.at(i), read.a.at(i));

            est_madg_mag << madg_mag.q.q_1 << "," << madg_mag.q.q_2 << "," << madg_mag.q.q_3 << "," << madg_mag.q.q_4 << "\n";
            est_comp_mag << comp_mag.q.q_1 << "," << comp_mag.q.q_2 << "," << comp_mag.q.q_3 << "," << comp_mag.q.q_4 << "\n";

            est_madg_no_mag << madg.q.q_1 << "," << madg.q.q_2 << "," << madg.q.q_3 << "," << madg.q.q_4 << "\n";
            est_comp_no_mag << comp.q.q_1 << "," << comp.q.q_2 << "," << comp.q.q_3 << "," << comp.q.q_4 << "\n";


            Quaternion enu_est = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg_mag.q);
            // std::cout << read.gt.at(i).q_1 << "," << read.gt.at(i).q_2 << "," << read.gt.at(i).q_3 << "," << read.gt.at(i).q_4 << std::endl;
            // std::cout << madg_mag.q.q_1 << "," << madg_mag.q.q_2 << "," << madg_mag.q.q_3 << "," << madg_mag.q.q_4 << std::endl;
            // std::cout << enu_est.q_1 << "," << enu_est.q_2 << "," << enu_est.q_3 << "," << enu_est.q_4 << std::endl;
            // Quaternion err_quat = Metrics::error_quaternion(read.gt.at(i), madg_mag.q);
            // Quaternion err_quat = Metrics::error_quaternion_earth(read.gt.at(i), madg_mag.q);
            // Quaternion err_quat = Metrics::error_quaternion(read.gt.at(i), enu_est);
            Quaternion err_quat = Metrics::error_quaternion_earth(read.gt.at(i), enu_est);
            quat_err << err_quat.q_1 << "," << err_quat.q_2 << "," << err_quat.q_3 << "," << err_quat.q_4 << "\n";
            double err = Metrics::total_error(err_quat);
            double err_inc = Metrics::inclination_error(err_quat);
            double err_head = Metrics::heading_error(err_quat);

            sum += err*err;
            sum_inc += err_inc*err_inc;
            sum_head += err_head*err_head;

            e << err << "," << err_inc << "," << err_head << "\n";

            // float roll_diff = Metrics::euler_roll_diff(read.gt.at(i), madg_mag.q);
            // float pitch_diff = Metrics::euler_pitch_diff(read.gt.at(i), madg_mag.q);
            // float yaw_diff = Metrics::euler_yaw_diff(read.gt.at(i), madg_mag.q);
            float roll_diff = Metrics::euler_roll_diff(read.gt.at(i), enu_est);
            float pitch_diff = Metrics::euler_pitch_diff(read.gt.at(i), enu_est);
            float yaw_diff = Metrics::euler_yaw_diff(read.gt.at(i), enu_est);

            euler_diff << roll_diff << "," << pitch_diff << "," << yaw_diff << "\n";

            // float mq1 = madg.q_1*1/sqrt(2) - madg.q_4*1/sqrt(2);
            // float mq2 = madg.q_2*1/sqrt(2) - madg.q_3*1/sqrt(2);
            // float mq3 = madg.q_3*1/sqrt(2) + madg.q_2*1/sqrt(2);
            // float mq4 = madg.q_4*1/sqrt(2) + madg.q_1*1/sqrt(2);
            // madg.q_1 = mq1;
            // madg.q_2 = mq2;
            // madg.q_3 = mq3;
            // madg.q_4 = mq4;
            // float myaw = atan2(2*madg.q_2*madg.q_3-2*madg.q_1*madg.q_4, 2*madg.q_1 *madg.q_1+2*madg.q_2*madg.q_2-1);
            // float mpitch = -asin(2*madg.q_2*madg.q_4+2*madg.q_1*madg.q_3);
            // float mroll = atan2(2*madg.q_3*madg.q_4-2*madg.q_1*madg.q_2, 2*madg.q_1*madg.q_1 + 2*madg.q_4*madg.q_4-1);
            float myaw = madg.q.yaw();
            float mpitch = madg.q.pitch();
            float mroll = madg.q.roll();
            gravity_vectors.push_back({-sin(mpitch), cos(mpitch)*sin(mroll), cos(mpitch)*cos(mroll)});

            // float cyaw = atan2(2*comp.q_2*comp.q_3-2*comp.q_1*comp.q_4, 2*comp.q_1 *comp.q_1+2*comp.q_2*comp.q_2-1);
            // float cpitch = -asin(2*comp.q_2*comp.q_4+2*comp.q_1*comp.q_3);
            // float croll = atan2(2*comp.q_3*comp.q_4-2*comp.q_1*comp.q_2, 2*comp.q_1*comp.q_1 + 2*comp.q_4*comp.q_4-1);
            // gravity_vectors.push_back({-sin(cpitch), cos(cpitch)*sin(croll), cos(cpitch)*cos(croll)});

            // float gyaw = atan2(2*read.gt.at(i)x*read.gt.at(i)z-2*read.gt.at(i)x*read.gt.at(i).at(3), 2*read.gt.at(i)x *read.gt.at(i)x+2*read.gt.at(i)x*read.gt.at(i)x-1);
            // float gpitch = -asin(2*read.gt.at(i)x*read.gt.at(i).at(3)+2*read.gt.at(i)x*read.gt.at(i)z);
            // float groll = atan2(2*read.gt.at(i)z*read.gt.at(i).at(3)-2*read.gt.at(i)x*read.gt.at(i)x, 2*read.gt.at(i)x*read.gt.at(i)x + 2*read.gt.at(i).at(3)*read.gt.at(i).at(3)-1);
            // gravity_vectors.push_back({-sin(gpitch), cos(gpitch)*sin(groll), cos(gpitch)*cos(groll)});
        }

        float rmse = sqrt(sum/35001.0f);
        float rmse_inc = sqrt(sum_inc/35001.0f);
        float rmse_head = sqrt(sum_head/35001.0f);
        // float rmse = sqrt(sum/1801.0f);
        // float rmse_inc = sqrt(sum_inc/1801.0f);
        // float rmse_head = sqrt(sum_head/1801.0f);
        std::cout << sum << std::endl;
        std::cout << rmse << std::endl;
        // std::cout << sum_inc << std::endl;
        std::cout << rmse_inc << std::endl;
        std::cout << rmse_head << std::endl;

        est_madg_mag.close();
        est_madg_no_mag.close();
        est_comp_mag.close();
        est_comp_no_mag.close();

        quat_err.close();
        e.close();

        euler_diff.close();
    } 
    catch(const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    int endTime = 200;
    double frameRate = 5;

    vtkNew<vtkNamedColors> colors;
    std::array<unsigned char, 4> bkg{{26, 51, 77, 255}};
    colors->SetColor("BkgColor", bkg.data());

    // Create the graphics structure. The renderer renders into the render window
    vtkNew<vtkRenderWindowInteractor> iren;
    vtkNew<vtkRenderer> ren1;
    vtkNew<vtkRenderWindow> renWin;
    renWin->SetMultiSamples(0);
    renWin->SetWindowName("Gravity Vector");
    
    iren->SetRenderWindow(renWin);
    renWin->AddRenderer(ren1);
    // ren1->SetBackground(colors->GetColor3d("MistyRose").GetData());
    ren1->SetBackground(colors->GetColor3d("BkgColor").GetData());
    ren1->GetActiveCamera()->SetPosition(-1, 0, 0);
    ren1->GetActiveCamera()->SetFocalPoint(0, 0, 1.0);
    ren1->GetActiveCamera()->SetViewUp(0, 0, -1);    
    
    renWin->SetSize(1200, 1200);
    renWin->Render();

    // iren->Start();

    // Create an Animation Scene
    vtkNew<vtkAnimationScene> scene;

    scene->SetModeToRealTime();
    // scene->SetModeToSequence();

    scene->SetLoop(0);
    scene->SetFrameRate(frameRate); // FPS
    scene->SetStartTime(0);
    scene->SetEndTime(endTime); // how many seconds for it to run for
 
    // Create an Animation Cue
    vtkNew<vtkAnimationCue> cue1;
    cue1->SetStartTime(0);
    cue1->SetEndTime(endTime); // how many seconds for it to run for
    scene->AddCue(cue1);

    // Create cue animator
    CueAnimator animator;
    animator.setGrav(gravity_vectors);

    // Create Cue observer
    vtkNew<vtkAnimationCueObserver> observer;
    observer->Renderer = ren1;
    observer->Animator = &animator;
    observer->RenWin = renWin;
    observer->Inter = iren;

    cue1->AddObserver(vtkCommand::StartAnimationCueEvent, observer);
    cue1->AddObserver(vtkCommand::EndAnimationCueEvent, observer);
    cue1->AddObserver(vtkCommand::AnimationCueTickEvent, observer);
    
    scene->Play();
    scene->Stop();

    // iren->Start();

    return EXIT_SUCCESS;
}