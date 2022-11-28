#pragma once
#include <vtkNew.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkContourFilter.h>
#include <vtkMath.h>
#include <vtkNamedColors.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkShortArray.h>
#include <vtkStructuredPoints.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAxesActor.h>
#include <vtkArrowSource.h>
#include <vtkPlaneSource.h>
#include <vtkMinimalStandardRandomSequence.h>

static vtkSmartPointer<vtkActor> getArrowActor(std::vector<double> vector)
{
    auto colors = vtkSmartPointer<vtkNamedColors>::New();

    auto arrowSource = vtkSmartPointer<vtkArrowSource>::New();
    arrowSource->SetShaftRadius(0.01);
    arrowSource->SetTipRadius(0.05);
    arrowSource->SetTipLength(0.1);
    arrowSource->Update();

    // Generate a random start and end point
    double startPoint[3] = {0, 0, 0};
    // double endPoint[3] = {0.0, 0.0, 1.0};
    float x = -sin(vector.at(1));
    float y = cos(vector.at(1))*sin(vector.at(0));
    float z = -cos(vector.at(1))*cos(vector.at(0));
    double endPoint[3] = {x, y, z};
    vtkNew<vtkMinimalStandardRandomSequence> rng;

    // Compute a basis
    double normalizedX[3];
    double normalizedY[3];
    double normalizedZ[3];

    // // The X axis is a vector from start to end
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
    auto transform = vtkSmartPointer<vtkTransform>::New();
    // vtkNew<vtkTransform> transform;
    transform->Translate(startPoint);
    transform->Concatenate(matrix);
    transform->Scale(length, length, length);

    // Create a mapper and actor for the arrow

    auto arrowMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    auto arrowActor = vtkSmartPointer<vtkActor>::New();
    arrowMapper->SetInputConnection(arrowSource->GetOutputPort());
    arrowActor->SetUserMatrix(transform->GetMatrix());
    arrowActor->SetMapper(arrowMapper);
    arrowActor->GetProperty()->SetColor(colors->GetColor3d("White").GetData());

    return arrowActor;
}

static vtkSmartPointer<vtkActor> getPlaneActor(std::vector<double> vector)
{
    auto colors = vtkSmartPointer<vtkNamedColors>::New();

    auto planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    // PLANE
    // Create a plane
    planeSource->SetOrigin(0.0, 0.0, 0.0);
    planeSource->SetPoint1(0.5, 0.0, 0.0);
    planeSource->SetPoint2(0.0, 0.25, 0.0);
    planeSource->SetCenter(0.0, 0.0, 0.0);

    float x = -sin(vector.at(1));
    float y = cos(vector.at(1))*sin(vector.at(0));
    float z = -cos(vector.at(1))*cos(vector.at(0));
    
    planeSource->SetNormal(x, y, z);    
    planeSource->Update();

    vtkSmartPointer<vtkPolyData> plane = planeSource->GetOutput();

    // Create a mapper and actor
    auto planeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    planeMapper->SetInputData(plane);

    auto planeActor = vtkSmartPointer<vtkActor>::New();
    planeActor->SetMapper(planeMapper);
    planeActor->GetProperty()->SetColor(colors->GetColor3d("Banana").GetData());

    return planeActor;
}