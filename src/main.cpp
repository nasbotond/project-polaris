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

int main(int argc, char* argv[]) 
{
    // Input
    ComplementaryFilter comp = ComplementaryFilter();
    MadgwickFilter madg = MadgwickFilter();
    CsvReader read = CsvReader("../test_data/");
    std::vector<std::vector<double>> gravity_vectors;
    
    try 
    {
        read.retrieveFileItems();

        // for (int i = 0; i < 40001; ++i)
        for (int i = 10000; i < 50001; ++i)
        {
            // with Magnetometer
            // comp.updateFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2), read.m.at(i).at(0), read.m.at(i).at(1), read.m.at(i).at(2));
            // madg.updateMARGFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2), read.m.at(i).at(0), read.m.at(i).at(1), read.m.at(i).at(2));
            // without Magnetometer
            comp.updateFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2));
            madg.updateIMUFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2));

            // float mq1 = madg.q_1*1/sqrt(2) - madg.q_4*1/sqrt(2);
            // float mq2 = madg.q_2*1/sqrt(2) - madg.q_3*1/sqrt(2);
            // float mq3 = madg.q_3*1/sqrt(2) + madg.q_2*1/sqrt(2);
            // float mq4 = madg.q_4*1/sqrt(2) + madg.q_1*1/sqrt(2);
            // madg.q_1 = mq1;
            // madg.q_2 = mq2;
            // madg.q_3 = mq3;
            // madg.q_4 = mq4;
            float myaw = atan2(2*madg.q_2*madg.q_3-2*madg.q_1*madg.q_4, 2*madg.q_1 *madg.q_1+2*madg.q_2*madg.q_2-1);
            float mpitch = -asin(2*madg.q_2*madg.q_4+2*madg.q_1*madg.q_3);
            float mroll = atan2(2*madg.q_3*madg.q_4-2*madg.q_1*madg.q_2, 2*madg.q_1*madg.q_1 + 2*madg.q_4*madg.q_4-1);
            gravity_vectors.push_back({-sin(mpitch), cos(mpitch)*sin(mroll), cos(mpitch)*cos(mroll)});

            // float cyaw = atan2(2*comp.q_2*comp.q_3-2*comp.q_1*comp.q_4, 2*comp.q_1 *comp.q_1+2*comp.q_2*comp.q_2-1);
            // float cpitch = -asin(2*comp.q_2*comp.q_4+2*comp.q_1*comp.q_3);
            // float croll = atan2(2*comp.q_3*comp.q_4-2*comp.q_1*comp.q_2, 2*comp.q_1*comp.q_1 + 2*comp.q_4*comp.q_4-1);
            // gravity_vectors.push_back({-sin(cpitch), cos(cpitch)*sin(croll), cos(cpitch)*cos(croll)});

            // float gyaw = atan2(2*read.gt.at(i).at(1)*read.gt.at(i).at(2)-2*read.gt.at(i).at(0)*read.gt.at(i).at(3), 2*read.gt.at(i).at(0) *read.gt.at(i).at(0)+2*read.gt.at(i).at(1)*read.gt.at(i).at(1)-1);
            // float gpitch = -asin(2*read.gt.at(i).at(1)*read.gt.at(i).at(3)+2*read.gt.at(i).at(0)*read.gt.at(i).at(2));
            // float groll = atan2(2*read.gt.at(i).at(2)*read.gt.at(i).at(3)-2*read.gt.at(i).at(0)*read.gt.at(i).at(1), 2*read.gt.at(i).at(0)*read.gt.at(i).at(0) + 2*read.gt.at(i).at(3)*read.gt.at(i).at(3)-1);
            // gravity_vectors.push_back({-sin(gpitch), cos(gpitch)*sin(groll), cos(gpitch)*cos(groll)});
        }
        // int i = 40000;
        // // comp.updateFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2), read.m.at(i).at(0), read.m.at(i).at(1), read.m.at(i).at(2));
        // // madg.updateMARGFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2), read.m.at(i).at(0), read.m.at(i).at(1), read.m.at(i).at(2));
        // float cq1 = comp.q_1*1/sqrt(2) - comp.q_4*1/sqrt(2);
        // float cq2 = comp.q_2*1/sqrt(2) - comp.q_3*1/sqrt(2);
        // float cq3 = comp.q_3*1/sqrt(2) + comp.q_2*1/sqrt(2);
        // float cq4 = comp.q_4*1/sqrt(2) + comp.q_1*1/sqrt(2);
        // comp.q_1 = cq1;
        // comp.q_2 = cq2;
        // comp.q_3 = cq3;
        // comp.q_4 = cq4;
        
        // float cyaw = atan2(2*comp.q_2*comp.q_3-2*comp.q_1*comp.q_4, 2*comp.q_1 *comp.q_1+2*comp.q_2*comp.q_2-1);
        // float cpitch = -asin(2*comp.q_2*comp.q_4+2*comp.q_1*comp.q_3);
        // float croll = atan2(2*comp.q_3*comp.q_4-2*comp.q_1*comp.q_2, 2*comp.q_1*comp.q_1 + 2*comp.q_4*comp.q_4-1);
        // std::cout << "COMP" << std::endl;
        // std::cout << "y: " << cyaw*(180/M_PI) << std::endl;
        // std::cout << "p: " << cpitch*(180/M_PI) << std::endl;
        // std::cout << "r: " << croll*(180/M_PI) << std::endl;
        // std::cout << "q1: " << comp.q_1 << std::endl;
        // std::cout << "q2: " << comp.q_2 << std::endl;
        // std::cout << "q3: " << comp.q_3 << std::endl;
        // std::cout << "q4: " << comp.q_4 << std::endl;

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
        // std::cout << "MADG" << std::endl;
        // std::cout << "y: " << myaw*(180/M_PI) << std::endl;
        // std::cout << "p: " << mpitch*(180/M_PI) << std::endl;
        // std::cout << "r: " << mroll*(180/M_PI) << std::endl;
        // std::cout << "q1: " << madg.q_1 << std::endl;
        // std::cout << "q2: " << madg.q_2 << std::endl;
        // std::cout << "q3: " << madg.q_3 << std::endl;
        // std::cout << "q4: " << madg.q_4 << std::endl;        

        // float gyaw = atan2(2*read.gt.at(i).at(1)*read.gt.at(i).at(2)-2*read.gt.at(i).at(0)*read.gt.at(i).at(3), 2*read.gt.at(i).at(0) *read.gt.at(i).at(0)+2*read.gt.at(i).at(1)*read.gt.at(i).at(1)-1);
        // float gpitch = -asin(2*read.gt.at(i).at(1)*read.gt.at(i).at(3)+2*read.gt.at(i).at(0)*read.gt.at(i).at(2));
        // float groll = atan2(2*read.gt.at(i).at(2)*read.gt.at(i).at(3)-2*read.gt.at(i).at(0)*read.gt.at(i).at(1), 2*read.gt.at(i).at(0)*read.gt.at(i).at(0) + 2*read.gt.at(i).at(3)*read.gt.at(i).at(3)-1);
        // std::cout << "GT" << std::endl;
        // std::cout << "y: " << gyaw*(180/M_PI) << std::endl;
        // std::cout << "p: " << gpitch*(180/M_PI) << std::endl;
        // std::cout << "r: " << groll*(180/M_PI) << std::endl;
        // std::cout << "q1: " << read.gt.at(i).at(0) << std::endl;
        // std::cout << "q2: " << read.gt.at(i).at(1) << std::endl;
        // std::cout << "q3: " << read.gt.at(i).at(2) << std::endl;
        // std::cout << "q4: " << read.gt.at(i).at(3) << std::endl;
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