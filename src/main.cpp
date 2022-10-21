#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string>

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkArrowSource.h>
#include <vtkCamera.h>
#include <vtkPlaneSource.h>
#include <vtkMath.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <array>

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

        // for (int i = 932; i < 120000; ++i)
        for (int i = 5000; i < 100001; ++i)
        {
            comp.updateFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2), read.m.at(i).at(0), read.m.at(i).at(1), read.m.at(i).at(2));
            madg.updateMARGFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2), read.m.at(i).at(0), read.m.at(i).at(1), read.m.at(i).at(2));
            // madg.updateIMUFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2));
        }
        int i = 100000;
        // comp.updateFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2), read.m.at(i).at(0), read.m.at(i).at(1), read.m.at(i).at(2));
        // madg.updateMARGFilter(read.w.at(i).at(0), read.w.at(i).at(1), read.w.at(i).at(2), read.a.at(i).at(0), read.a.at(i).at(1), read.a.at(i).at(2), read.m.at(i).at(0), read.m.at(i).at(1), read.m.at(i).at(2));
        float cq1 = comp.q_1*1/sqrt(2) - comp.q_4*1/sqrt(2);
        float cq2 = comp.q_2*1/sqrt(2) - comp.q_3*1/sqrt(2);
        float cq3 = comp.q_3*1/sqrt(2) + comp.q_2*1/sqrt(2);
        float cq4 = comp.q_4*1/sqrt(2) + comp.q_1*1/sqrt(2);
        comp.q_1 = cq1;
        comp.q_2 = cq2;
        comp.q_3 = cq3;
        comp.q_4 = cq4;
        
        float cyaw = atan2(2*comp.q_2*comp.q_3-2*comp.q_1*comp.q_4, 2*comp.q_1 *comp.q_1+2*comp.q_2*comp.q_2-1);
        float cpitch = -asin(2*comp.q_2*comp.q_4+2*comp.q_1*comp.q_3);
        float croll = atan2(2*comp.q_3*comp.q_4-2*comp.q_1*comp.q_2, 2*comp.q_1*comp.q_1 + 2*comp.q_4*comp.q_4-1);
        std::cout << "COMP" << std::endl;
        std::cout << "y: " << cyaw*(180/M_PI) << std::endl;
        std::cout << "p: " << cpitch*(180/M_PI) << std::endl;
        std::cout << "r: " << croll*(180/M_PI) << std::endl;
        std::cout << "q1: " << comp.q_1 << std::endl;
        std::cout << "q2: " << comp.q_2 << std::endl;
        std::cout << "q3: " << comp.q_3 << std::endl;
        std::cout << "q4: " << comp.q_4 << std::endl;

        float mq1 = madg.q_1*1/sqrt(2) - madg.q_4*1/sqrt(2);
        float mq2 = madg.q_2*1/sqrt(2) - madg.q_3*1/sqrt(2);
        float mq3 = madg.q_3*1/sqrt(2) + madg.q_2*1/sqrt(2);
        float mq4 = madg.q_4*1/sqrt(2) + madg.q_1*1/sqrt(2);
        madg.q_1 = mq1;
        madg.q_2 = mq2;
        madg.q_3 = mq3;
        madg.q_4 = mq4;
        float myaw = atan2(2*madg.q_2*madg.q_3-2*madg.q_1*madg.q_4, 2*madg.q_1 *madg.q_1+2*madg.q_2*madg.q_2-1);
        float mpitch = -asin(2*madg.q_2*madg.q_4+2*madg.q_1*madg.q_3);
        float mroll = atan2(2*madg.q_3*madg.q_4-2*madg.q_1*madg.q_2, 2*madg.q_1*madg.q_1 + 2*madg.q_4*madg.q_4-1);
        std::cout << "MADG" << std::endl;
        std::cout << "y: " << myaw*(180/M_PI) << std::endl;
        std::cout << "p: " << mpitch*(180/M_PI) << std::endl;
        std::cout << "r: " << mroll*(180/M_PI) << std::endl;
        std::cout << "q1: " << madg.q_1 << std::endl;
        std::cout << "q2: " << madg.q_2 << std::endl;
        std::cout << "q3: " << madg.q_3 << std::endl;
        std::cout << "q4: " << madg.q_4 << std::endl;

        float gyaw = atan2(2*read.gt.at(i).at(1)*read.gt.at(i).at(2)-2*read.gt.at(i).at(0)*read.gt.at(i).at(3), 2*read.gt.at(i).at(0) *read.gt.at(i).at(0)+2*read.gt.at(i).at(1)*read.gt.at(i).at(1)-1);
        float gpitch = -asin(2*read.gt.at(i).at(1)*read.gt.at(i).at(3)+2*read.gt.at(i).at(0)*read.gt.at(i).at(2));
        float groll = atan2(2*read.gt.at(i).at(2)*read.gt.at(i).at(3)-2*read.gt.at(i).at(0)*read.gt.at(i).at(1), 2*read.gt.at(i).at(0)*read.gt.at(i).at(0) + 2*read.gt.at(i).at(3)*read.gt.at(i).at(3)-1);
        std::cout << "GT" << std::endl;
        std::cout << "y: " << gyaw*(180/M_PI) << std::endl;
        std::cout << "p: " << gpitch*(180/M_PI) << std::endl;
        std::cout << "r: " << groll*(180/M_PI) << std::endl;
        std::cout << "q1: " << read.gt.at(i).at(0) << std::endl;
        std::cout << "q2: " << read.gt.at(i).at(1) << std::endl;
        std::cout << "q3: " << read.gt.at(i).at(2) << std::endl;
        std::cout << "q4: " << read.gt.at(i).at(3) << std::endl;
    } 
    catch(const std::exception &e) 
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    // VISUALIZATION

    vtkNew<vtkNamedColors> colors;

    // Set the background color
    std::array<unsigned char, 4> bkg{{26, 51, 77, 255}};
    colors->SetColor("BkgColor", bkg.data());

    // Create an arrow
    vtkNew<vtkArrowSource> arrowSource;
    arrowSource->SetShaftRadius(0.01);
    arrowSource->SetTipRadius(0.05);
    arrowSource->SetTipLength(0.1);

    // Generate a random start and end point
    double startPoint[3] = {0, 0, 0};;
    double endPoint[3] = {0.0, 0.0, 1.0};
    vtkNew<vtkMinimalStandardRandomSequence> rng;

    // Compute a basis
    double normalizedX[3];
    double normalizedY[3];
    double normalizedZ[3];

    // The X axis is a vector from start to end
    vtkMath::Subtract(endPoint, startPoint, normalizedX);
    double length = vtkMath::Norm(normalizedX);
    vtkMath::Normalize(normalizedX);

    // The Z axis is an arbitrary vector cross X
    double arbitrary[3];
    for (auto i = 0; i < 3; ++i)
    {
        rng->Next();
        arbitrary[i] = rng->GetRangeValue(-10, 10);
    }
    vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
    vtkMath::Normalize(normalizedZ);

    // The Y axis is Z cross X
    vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
    vtkNew<vtkMatrix4x4> matrix;

    // Create the direction cosine matrix
    matrix->Identity();
    for (auto i = 0; i < 3; i++)
    {
        matrix->SetElement(i, 0, normalizedX[i]);
        matrix->SetElement(i, 1, normalizedY[i]);
        matrix->SetElement(i, 2, normalizedZ[i]);
    }

    // Apply the transforms
    vtkNew<vtkTransform> transform;
    transform->Translate(startPoint);
    transform->Concatenate(matrix);
    transform->Scale(length, length, length);

    // Create a mapper and actor for the arrow
    vtkNew<vtkPolyDataMapper> mapper;
    vtkNew<vtkActor> actor;
    mapper->SetInputConnection(arrowSource->GetOutputPort());
    actor->SetUserMatrix(transform->GetMatrix());
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d("White").GetData());

    // Create spheres for start and end point
    vtkNew<vtkSphereSource> sphereStartSource;
    sphereStartSource->SetCenter(startPoint);
    sphereStartSource->SetRadius(0.01);
    vtkNew<vtkPolyDataMapper> sphereStartMapper;
    sphereStartMapper->SetInputConnection(sphereStartSource->GetOutputPort());
    vtkNew<vtkActor> sphereStart;
    sphereStart->SetMapper(sphereStartMapper);
    sphereStart->GetProperty()->SetColor(colors->GetColor3d("Yellow").GetData());

    vtkNew<vtkSphereSource> sphereEndSource;
    sphereEndSource->SetCenter(endPoint);
    sphereEndSource->SetRadius(0.01);
    vtkNew<vtkPolyDataMapper> sphereEndMapper;
    sphereEndMapper->SetInputConnection(sphereEndSource->GetOutputPort());
    vtkNew<vtkActor> sphereEnd;
    sphereEnd->SetMapper(sphereEndMapper);
    sphereEnd->GetProperty()->SetColor(colors->GetColor3d("Magenta").GetData());

    // AXES
    vtkNew<vtkTransform> transformA;
    transformA->Translate(0.0, 0.0, 0.0);

    vtkNew<vtkAxesActor> axes;

    // The axes are positioned with a user transform
    axes->SetUserTransform(transformA);

    // PLANE
    // Create a plane
    vtkNew<vtkPlaneSource> planeSource;
    planeSource->SetOrigin(0.0, 0.0, 0.0);
    planeSource->SetPoint1(0.5, 0.0, 0.0);
    planeSource->SetPoint2(0.0, 0.25, 0.0);
    planeSource->SetCenter(0.0, 0.0, 0.0);
    planeSource->SetNormal(endPoint[0], endPoint[1], endPoint[2]);    
    planeSource->Update();

    vtkPolyData* plane = planeSource->GetOutput();

    // Create a mapper and actor
    vtkNew<vtkPolyDataMapper> planeMapper;
    planeMapper->SetInputData(plane);

    vtkNew<vtkActor> planeActor;
    planeActor->SetMapper(planeMapper);
    planeActor->GetProperty()->SetColor(colors->GetColor3d("Banana").GetData());

    // Create a renderer, render window, and interactor
    vtkNew<vtkRenderer> renderer;
    vtkNew<vtkRenderWindow> renderWindow;
    renderWindow->AddRenderer(renderer);
    renderWindow->SetWindowName("Gravity Vector");
    renderWindow->SetSize(600, 600);

    vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Add the actor to the scene
    renderer->AddActor(planeActor);
    renderer->AddActor(axes);
    renderer->AddActor(actor);
    renderer->AddActor(sphereStart);
    renderer->AddActor(sphereEnd);
    renderer->SetBackground(colors->GetColor3d("BkgColor").GetData());

    renderer->GetActiveCamera()->SetPosition(-1, 0, 0);
    renderer->GetActiveCamera()->SetFocalPoint(0, 0, 1.0);
    renderer->GetActiveCamera()->SetViewUp(0, 0, -1);

    // // Render and interact
    // renderWindow->Render();
    // renderWindowInteractor->Start();
    renderer->ResetCamera();
    renderWindow->SetWindowName("Axes");
    renderWindow->Render();
 
    // begin mouse interaction
    renderWindowInteractor->Start();

    return EXIT_SUCCESS;
    // return 0;
}