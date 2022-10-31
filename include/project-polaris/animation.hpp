#include <vtkAnimationCue.h>
#include <vtkAnimationScene.h>
#include <vtkCommand.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkArrowSource.h>
#include <vtkPlaneSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkMinimalStandardRandomSequence.h>

class CueAnimator
{
private:
  std::vector<std::vector<double>> gravity_vectors;

public:
  CueAnimator()
  {
    this->ArrowSource = 0;
    this->ArrowMapper = 0;
    this->ArrowActor = 0;

    this->PlaneSource = 0;
    this->PlaneMapper = 0;
    this->PlaneActor = 0;
  }

  ~CueAnimator()
  {
    this->Cleanup();
  }

  void setGrav(std::vector<std::vector<double>> gravity_vectors)
  {
    this->gravity_vectors = gravity_vectors;
  }

  int tick = 0;

  void StartCue(vtkAnimationCue::AnimationCueInfo* vtkNotUsed(info),
                vtkRenderer* ren)
  {
    // std::cout << "*** IN StartCue " << std::endl;

    vtkNew<vtkNamedColors> colors;

    // ARROW
    // Create an arrow source
    this->ArrowSource = vtkArrowSource::New();
    this->ArrowSource->SetShaftRadius(0.01);
    this->ArrowSource->SetTipRadius(0.05);
    this->ArrowSource->SetTipLength(0.1);
    this->ArrowSource->Update();

    // Generate a random start and end point
    double startPoint[3] = {0, 0, 0};
    double endPoint[3] = {0.0, 0.0, 1.0};
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
    vtkNew<vtkTransform> transform;
    transform->Translate(startPoint);
    transform->Concatenate(matrix);
    transform->Scale(length, length, length);

    // Create a mapper and actor for the arrow
    this->ArrowMapper = vtkPolyDataMapper::New();
    this->ArrowActor = vtkActor::New();
    this->ArrowMapper->SetInputConnection(this->ArrowSource->GetOutputPort());
    this->ArrowActor->SetUserMatrix(transform->GetMatrix());
    this->ArrowActor->SetMapper(this->ArrowMapper);
    this->ArrowActor->GetProperty()->SetColor(colors->GetColor3d("White").GetData());

    // AXES
    vtkNew<vtkTransform> transformA;
    transformA->Translate(0.0, 0.0, 0.0);

    vtkNew<vtkAxesActor> axes;

    // The axes are positioned with a user transform
    axes->SetUserTransform(transformA);

    // PLANE
    // Create a plane
    this->PlaneSource = vtkPlaneSource::New();
    this->PlaneSource->SetOrigin(0.0, 0.0, 0.0);
    this->PlaneSource->SetPoint1(0.5, 0.0, 0.0);
    this->PlaneSource->SetPoint2(0.0, 0.25, 0.0);
    this->PlaneSource->SetCenter(0.0, 0.0, 0.0);
    this->PlaneSource->SetNormal(endPoint[0], endPoint[1], endPoint[2]);    
    this->PlaneSource->Update();

    vtkPolyData* plane = this->PlaneSource->GetOutput();

    // Create a mapper and actor
    this->PlaneMapper = vtkPolyDataMapper::New();
    this->PlaneMapper->SetInputData(plane);

    this->PlaneActor = vtkActor::New();
    this->PlaneActor->SetMapper(this->PlaneMapper);
    this->PlaneActor->GetProperty()->SetColor(colors->GetColor3d("Banana").GetData());

    // Add the actor to the scene
    ren->AddActor(this->PlaneActor);
    ren->AddActor(axes);
    ren->AddActor(this->ArrowActor);

    // Render and interact
    ren->ResetCamera();
    ren->Render();
  }

  void Tick(vtkAnimationCue::AnimationCueInfo* info, vtkRenderer* ren)
  {
    // std::cout << "*** IN Tick " << std::endl;
    // std::cout << gravity_vectors[tick][0] << gravity_vectors[tick][1]<< gravity_vectors[tick][2] << std::endl;

    this->ArrowSource->SetShaftRadius(0.01);
    this->ArrowSource->SetTipRadius(0.05);
    this->ArrowSource->SetTipLength(0.1);

    double startPoint[3] = {0, 0, 0};
    double endPoint[3] = {gravity_vectors[tick][0], gravity_vectors[tick][1], gravity_vectors[tick][2]};
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
    vtkNew<vtkTransform> transform;
    transform->Translate(startPoint);
    transform->Concatenate(matrix);
    transform->Scale(length, length, length);

    this->ArrowActor->SetUserMatrix(transform->GetMatrix());
    this->ArrowSource->Update();

    this->PlaneSource->SetNormal(endPoint[0], endPoint[1], endPoint[2]); 
    this->PlaneSource->Update();
    ren->Render();

    tick++;
  }

  void EndCue(vtkAnimationCue::AnimationCueInfo* vtkNotUsed(info),
              vtkRenderer* ren)
  {
    // std::cout << "*** IN EndCue " << std::endl;
    (void)ren;
    // don't remove the actor for the regression image.
    //      ren->RemoveActor(this->Actor);
    this->Cleanup();
  }

protected:
  vtkArrowSource* ArrowSource;
  vtkPolyDataMapper* ArrowMapper;
  vtkActor* ArrowActor;
  vtkPlaneSource* PlaneSource;
  vtkPolyDataMapper* PlaneMapper;
  vtkActor* PlaneActor;

  void Cleanup()
  {
    if (this->ArrowSource != 0)
    {
      this->ArrowSource->Delete();
      this->ArrowSource = 0;
    }
    if (this->PlaneSource != 0)
    {
      this->PlaneSource->Delete();
      this->PlaneSource = 0;
    }
    if (this->ArrowMapper != 0)
    {
      this->ArrowMapper->Delete();
      this->ArrowMapper = 0;
    }
    if (this->PlaneMapper != 0)
    {
      this->PlaneMapper->Delete();
      this->PlaneMapper = 0;
    }
    if (this->ArrowActor != 0)
    {
      this->ArrowActor->Delete();
      this->ArrowActor = 0;
    }
    if (this->PlaneActor != 0)
    {
      this->PlaneActor->Delete();
      this->PlaneActor = 0;
    }
  }
};

class vtkAnimationCueObserver : public vtkCommand
{
public:
  static vtkAnimationCueObserver* New()
  {
    return new vtkAnimationCueObserver;
  }

  virtual void Execute(vtkObject* vtkNotUsed(caller), unsigned long event,
                       void* calldata)
  {
    if (this->Animator != 0 && this->Renderer != 0)
    {
      vtkAnimationCue::AnimationCueInfo* info =
          static_cast<vtkAnimationCue::AnimationCueInfo*>(calldata);
      switch (event)
      {
      case vtkCommand::StartAnimationCueEvent:
        this->Animator->StartCue(info, this->Renderer);
        break;
      case vtkCommand::EndAnimationCueEvent:
        this->Animator->EndCue(info, this->Renderer);
        break;
      case vtkCommand::AnimationCueTickEvent:
        this->Animator->Tick(info, this->Renderer);
        break;
      }
    }
    if (this->RenWin != 0)
    {
      this->RenWin->Render();
    }
  }

  vtkRenderer* Renderer;
  vtkRenderWindow* RenWin;
  CueAnimator* Animator;

protected:
  vtkAnimationCueObserver()
  {
    this->Renderer = 0;
    this->Animator = 0;
    this->RenWin = 0;
  }
};