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

    std::ofstream error_madg_mag;
    std::ofstream error_madg_no_mag;
    std::ofstream error_comp_mag;
    std::ofstream error_comp_no_mag;
    // std::ofstream euler_diff;
    
    try
    {
        read.retrieveFileItems();

        est_madg_mag.open ("../results/est_madg_mag.csv");
        est_madg_no_mag.open ("../results/est_madg_no_mag.csv");
        est_comp_mag.open ("../results/est_comp_mag.csv");
        est_comp_no_mag.open ("../results/est_comp_no_mag.csv");

        error_madg_mag.open ("../results/error_madg_mag.csv");
        error_madg_no_mag.open ("../results/error_madg_no_mag.csv");
        error_comp_mag.open ("../results/error_comp_mag.csv");
        error_comp_no_mag.open ("../results/error_comp_no_mag.csv");

        // euler_diff.open ("../results/euler_diff.csv");

        double sum = 0.0;
        double sum_inc = 0.0;
        double sum_head = 0.0;

        // for (int i = 0; i < read.a.size(); ++i)
        for (int i = 10000; i < 45001; ++i)
        // for (int i = 1000; i < 20000; ++i)
        // for (int i = 10000; i < 41795; ++i)
        {
            // TODO: initial starting quat?????
            // with Magnetometer
            comp_mag.updateFilter(read.w.at(i), read.a.at(i), read.m.at(i));
            madg_mag.updateMARGFilter(read.w.at(i), read.a.at(i), read.m.at(i));

            // without Magnetometer
            comp.updateFilter(read.w.at(i), read.a.at(i));
            madg.updateIMUFilter(read.w.at(i), read.a.at(i));

            est_madg_mag << madg_mag.q.q_1 << "," << madg_mag.q.q_2 << "," << madg_mag.q.q_3 << "," << madg_mag.q.q_4 << "\n";
            est_comp_mag << comp_mag.q.q_1 << "," << comp_mag.q.q_2 << "," << comp_mag.q.q_3 << "," << comp_mag.q.q_4 << "\n";

            est_madg_no_mag << madg.q.q_1 << "," << madg.q.q_2 << "," << madg.q.q_3 << "," << madg.q.q_4 << "\n";
            est_comp_no_mag << comp.q.q_1 << "," << comp.q.q_2 << "," << comp.q.q_3 << "," << comp.q.q_4 << "\n";

            // Quaternion enu_est = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg_mag.q);
            // Quaternion err_quat = Metrics::error_quaternion(read.gt.at(i), madg_mag.q);
            // Quaternion err_quat = Metrics::error_quaternion_earth(read.gt.at(i), madg_mag.q);
            // Quaternion err_quat = Metrics::error_quaternion(read.gt.at(i), enu_est);
            Quaternion err_quat_madg_mag = Metrics::error_quaternion_earth(read.gt.at(i), madg_mag.q);
            Quaternion err_quat_madg_no_mag = Metrics::error_quaternion_earth(read.gt.at(i), madg.q);
            Quaternion err_quat_comp_mag = Metrics::error_quaternion_earth(read.gt.at(i), comp_mag.q);
            Quaternion err_quat_comp_no_mag = Metrics::error_quaternion_earth(read.gt.at(i), comp.q);

            error_madg_mag << Metrics::total_error(err_quat_madg_mag) << "," << Metrics::inclination_error(err_quat_madg_mag) << "," << Metrics::heading_error(err_quat_madg_mag) << "\n";
            error_madg_no_mag << Metrics::total_error(err_quat_madg_no_mag) << "," << Metrics::inclination_error(err_quat_madg_no_mag) << "," << Metrics::heading_error(err_quat_madg_no_mag) << "\n";
            error_comp_mag << Metrics::total_error(err_quat_comp_mag) << "," << Metrics::inclination_error(err_quat_comp_mag) << "," << Metrics::heading_error(err_quat_comp_mag) << "\n";
            error_comp_no_mag << Metrics::total_error(err_quat_comp_no_mag) << "," << Metrics::inclination_error(err_quat_comp_no_mag) << "," << Metrics::heading_error(err_quat_comp_no_mag) << "\n";

            // float roll_diff = Metrics::euler_roll_diff(read.gt.at(i), madg_mag.q);
            // float pitch_diff = Metrics::euler_pitch_diff(read.gt.at(i), madg_mag.q);
            // float yaw_diff = Metrics::euler_yaw_diff(read.gt.at(i), madg_mag.q);
            // float roll_diff = Metrics::euler_roll_diff(read.gt.at(i), enu_est);
            // float pitch_diff = Metrics::euler_pitch_diff(read.gt.at(i), enu_est);
            // float yaw_diff = Metrics::euler_yaw_diff(read.gt.at(i), enu_est);

            // euler_diff << roll_diff << "," << pitch_diff << "," << yaw_diff << "\n";

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
        }

        est_madg_mag.close();
        est_madg_no_mag.close();
        est_comp_mag.close();
        est_comp_no_mag.close();

        error_madg_mag.close();
        error_madg_no_mag.close();
        error_comp_mag.close();
        error_comp_no_mag.close();

        // euler_diff.close();

        std::vector<Vec3> rmse = read.getRMSE();
        std::cout << "Madg Mag Total: " << rmse.at(0).x << std::endl;
        std::cout << "Madg Mag Inc: " << rmse.at(0).y << std::endl;
        std::cout << "Madg Mag Head: " << rmse.at(0).z << std::endl;

        std::cout << "Madg No Mag Total: " << rmse.at(1).x << std::endl;
        std::cout << "Madg No Mag Inc: " << rmse.at(1).y << std::endl;
        std::cout << "Madg No Mag Head: " << rmse.at(1).z << std::endl;

        std::cout << "Comp Mag Total: " << rmse.at(2).x << std::endl;
        std::cout << "Comp Mag Inc: " << rmse.at(2).y << std::endl;
        std::cout << "Comp Mag Head: " << rmse.at(2).z << std::endl;

        std::cout << "Comp No Mag Total: " << rmse.at(3).x << std::endl;
        std::cout << "Comp No Mag Inc: " << rmse.at(3).y << std::endl;
        std::cout << "Comp No Mag Head: " << rmse.at(3).z << std::endl;
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