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

    auto arrow_source = vtkSmartPointer<vtkArrowSource>::New();
    arrow_source->SetShaftRadius(0.01);
    arrow_source->SetTipRadius(0.05);
    arrow_source->SetTipLength(0.1);
    arrow_source->Update();

    // Generate a random start and end point
    double start_point[3] = {0, 0, 0};
    // double end_point[3] = {0.0, 0.0, 1.0};
    float x = sin(vector.at(1));
    float y = -cos(vector.at(1))*sin(vector.at(0));
    // Negate z to make arrow point down (towards the earth)
    float z = cos(vector.at(1))*cos(vector.at(0));

    double end_point[3] = {x, y, z};
    vtkNew<vtkMinimalStandardRandomSequence> rng;

    // Compute a basis
    double normalized_x[3];
    double normalized_y[3];
    double normalized_z[3];

    // // The X axis is a vector from start to end
    vtkMath::Subtract(end_point, start_point, normalized_x);
    double length = vtkMath::Norm(normalized_x);
    vtkMath::Normalize(normalized_x);

    // The Z axis is an arbitrary vector cross X
    double arbitrary[3];
    for (auto i = 0; i < 3; ++i)
    {
        rng->Next();
        arbitrary[i] = rng->GetRangeValue(-10, 10);
    }
    vtkMath::Cross(normalized_x, arbitrary, normalized_z);
    vtkMath::Normalize(normalized_z);

    // The Y axis is Z cross X
    vtkMath::Cross(normalized_z, normalized_x, normalized_y);
    vtkNew<vtkMatrix4x4> matrix;

    // Create the direction cosine matrix
    matrix->Identity();
    for (auto i = 0; i < 3; i++)
    {
        matrix->SetElement(i, 0, normalized_x[i]);
        matrix->SetElement(i, 1, normalized_y[i]);
        matrix->SetElement(i, 2, normalized_z[i]);
    }

    // Apply the transforms
    auto transform = vtkSmartPointer<vtkTransform>::New();
    // vtkNew<vtkTransform> transform;
    transform->Translate(start_point);
    transform->Concatenate(matrix);
    transform->Scale(length, length, length);

    // Create a mapper and actor for the arrow

    auto arrow_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    auto arrow_actor = vtkSmartPointer<vtkActor>::New();
    arrow_mapper->SetInputConnection(arrow_source->GetOutputPort());
    arrow_actor->SetUserMatrix(transform->GetMatrix());
    arrow_actor->SetMapper(arrow_mapper);
    arrow_actor->GetProperty()->SetColor(colors->GetColor3d("White").GetData());

    return arrow_actor;
}

static vtkSmartPointer<vtkActor> getPlaneActor(std::vector<double> vector)
{
    auto colors = vtkSmartPointer<vtkNamedColors>::New();

    auto plane_source = vtkSmartPointer<vtkPlaneSource>::New();
    // PLANE
    // Create a plane
    plane_source->SetOrigin(0.0, 0.0, 0.0);
    // plane_source->SetPoint1(0.5*cos(vector.at(1))*cos(vector.at(2)), 0.5*(cos(vector.at(2))*sin(vector.at(1))*sin(vector.at(0)) - cos(vector.at(0))*sin(vector.at(2))), 0.5*(cos(vector.at(0))*cos(vector.at(2))*sin(vector.at(1)) + sin(vector.at(0))*sin(vector.at(2))));
    // plane_source->SetPoint2(0.25*cos(vector.at(1))*sin(vector.at(2)), 0.25*(cos(vector.at(0))*cos(vector.at(2)) + sin(vector.at(1))*sin(vector.at(0))*sin(vector.at(2))), 0.25*(cos(vector.at(0))*sin(vector.at(1))*sin(vector.at(2)) - cos(vector.at(2))*sin(vector.at(0))));
    plane_source->SetPoint1(0.5*cos(vector.at(1))*cos(vector.at(2)), 0.5*(cos(vector.at(0))*sin(vector.at(2)) + sin(vector.at(0))*sin(vector.at(1))*cos(vector.at(2))), 0.5*(sin(vector.at(0))*sin(vector.at(2)) - cos(vector.at(0))*sin(vector.at(1))*cos(vector.at(2))));
    plane_source->SetPoint2(-0.25*cos(vector.at(1))*sin(vector.at(2)), 0.25*(cos(vector.at(0))*cos(vector.at(2)) - sin(vector.at(1))*sin(vector.at(0))*sin(vector.at(2))), 0.25*(cos(vector.at(2))*sin(vector.at(0)) + cos(vector.at(0))*sin(vector.at(1))*sin(vector.at(2))));
    plane_source->SetCenter(0.0, 0.0, 0.0);

    // float x = -sin(vector.at(1));
    // float y = cos(vector.at(1))*sin(vector.at(0));
    // float z = -cos(vector.at(1))*cos(vector.at(0));
    
    // plane_source->SetNormal(x, y, z);
    plane_source->Update();

    vtkSmartPointer<vtkPolyData> plane = plane_source->GetOutput();

    // Create a mapper and actor
    auto plane_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    plane_mapper->SetInputData(plane);

    auto plane_actor = vtkSmartPointer<vtkActor>::New();
    plane_actor->SetMapper(plane_mapper);
    plane_actor->GetProperty()->SetColor(colors->GetColor3d("Banana").GetData());

    return plane_actor;
}