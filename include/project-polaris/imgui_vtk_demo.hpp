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
//   double Pr = 10.0; // The Lorenz parameters
//   double b = 2.667;
//   double r = 28.0;
//   double x, y, z;       // starting (and current) x, y, z
//   double h = 0.01;      // integration step size
//   int resolution = 200; // slice resolution
//   int iter = 10000000;  // number of iterations
//   double xmin = -30.0;  // x, y, z range for voxels
//   double xmax = 30.0;
//   double ymin = -30.0;
//   double ymax = 30.0;
//   double zmin = -10.0;
//   double zmax = 60.0;

//   // take a stab at an integration step size
//   auto xIncr = resolution / (xmax - xmin);
//   auto yIncr = resolution / (ymax - ymin);
//   auto zIncr = resolution / (zmax - zmin);

//   printf("The Lorenz Attractor\n");
//   printf("  Pr = %f\n", Pr);
//   printf("  b = %f\n", b);
//   printf("  r = %f\n", r);
//   printf("  integration step size = %f\n", h);
//   printf("  slice resolution = %d\n", resolution);
//   printf("  # of iterations = %d\n", iter);
//   printf("  specified range:\n");
//   printf("      x: %f, %f\n", xmin, xmax);
//   printf("      y: %f, %f\n", ymin, ymax);
//   printf("      z: %f, %f\n", zmin, zmax);

//   x = vtkMath::Random(xmin, xmax);
//   y = vtkMath::Random(ymin, ymax);
//   z = vtkMath::Random(zmin, zmax);
//   printf("  starting at %f, %f, %f\n", x, y, z);

//   // allocate memory for the slices
//   auto sliceSize = resolution * resolution;
//   auto numPts = sliceSize * resolution;
//   auto scalars =
//     vtkSmartPointer<vtkShortArray>::New();
//   auto s = scalars->WritePointer(0, numPts);
//   for (auto i = 0; i < numPts; i++)
//   {
//     s[i] = 0;
//   }
//   for (auto j = 0; j < iter; j++)
//   {
//     // integrate to next time step
//     auto xx = x + h * Pr * (y - x);
//     auto yy = y + h * (x * (r - z) - y);
//     auto zz = z + h * (x * y - (b * z));

//     x = xx;
//     y = yy;
//     z = zz;

//     // calculate voxel index
//     if (x < xmax && x > xmin && y < ymax && y > ymin && z < zmax && z > zmin)
//     {
//       auto xxx = static_cast<short>(static_cast<double>(xx - xmin) * xIncr);
//       auto yyy = static_cast<short>(static_cast<double>(yy - ymin) * yIncr);
//       auto zzz = static_cast<short>(static_cast<double>(zz - zmin) * zIncr);
//       auto index = xxx + yyy * resolution + zzz * sliceSize;
//       s[index] += 1;
//     }
//   }

//   auto colors =
//     vtkSmartPointer<vtkNamedColors>::New();

//   auto volume =
//     vtkSmartPointer<vtkStructuredPoints>::New();
//   volume->GetPointData()->SetScalars(scalars);
//   volume->SetDimensions(resolution, resolution, resolution);
//   volume->SetOrigin(xmin, ymin, zmin);
//   volume->SetSpacing((xmax - xmin) / resolution, (ymax - ymin) / resolution,
//     (zmax - zmin) / resolution);

//   printf("  contouring...\n");

//   // create iso-surface
//   auto contour =
//     vtkSmartPointer<vtkContourFilter>::New();
//   contour->SetInputData(volume);
//   contour->SetValue(0, 50);

//   // create mapper
//   auto mapper =
//     vtkSmartPointer<vtkPolyDataMapper>::New();
//   mapper->SetInputConnection(contour->GetOutputPort());
//   mapper->ScalarVisibilityOff();

//   // create actor
//   auto actor =
//     vtkSmartPointer<vtkActor>::New();
//   actor->SetMapper(mapper);
// #if VTK_MAJOR_VERSION >= 9
//   actor->GetProperty()->SetColor(colors->GetColor3d("PaleTurquoise").GetData());
// #endif

  auto colors = vtkSmartPointer<vtkNamedColors>::New();

  auto arrowSource = vtkSmartPointer<vtkArrowSource>::New();
  arrowSource->SetShaftRadius(0.01);
  arrowSource->SetTipRadius(0.05);
  arrowSource->SetTipLength(0.1);
  arrowSource->Update();

  // Generate a random start and end point
  double startPoint[3] = {0, 0, 0};
  // double endPoint[3] = {0.0, 0.0, 1.0};
  double endPoint[3] = {vector.at(0), vector.at(1), vector.at(2)};
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
  planeSource->SetNormal(vector.at(0), vector.at(1), vector.at(2));    
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