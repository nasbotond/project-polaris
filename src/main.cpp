#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <array>

// OpenGL Loader
// This can be replaced with another loader, e.g. glad, but
// remember to also change the corresponding initialize call!
#include <GL/gl3w.h>            // GL3w, initialized with gl3wInit() below

// Include glfw3.h after our OpenGL definitions
#include <GLFW/glfw3.h>

// ImGui + imgui-vtk
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "VtkViewer.hpp"

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAxesActor.h>

#include "madgwick_filter.hpp"
#include "comp_filter.hpp"
#include "csv_reader.hpp"
#include "animation.hpp"
#include "metrics.hpp"
// File-Specific Includes
#include "imgui_vtk_demo.hpp" // Actor generator for this demo

static void glfw_error_callback(int error, const char* description)
{
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

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

    // Setup pipeline
    auto actor = getArrowActor(gravity_vectors.at(0));
    auto planeActor = getPlaneActor(gravity_vectors.at(0));

    // AXES
    auto transformA = vtkSmartPointer<vtkTransform>::New();
    transformA->Translate(0.0, 0.0, 0.0);

    auto axes = vtkSmartPointer<vtkAxesActor>::New();

    // AXES
    // vtkNew<vtkTransform> transformA;
    // transformA->Translate(0.0, 0.0, 0.0);

    // vtkNew<vtkAxesActor> axes;

    // // The axes are positioned with a user transform
    // axes->SetUserTransform(transformA);

    // The axes are positioned with a user transform
    axes->SetUserTransform(transformA);


    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
    {
        return 1;
    }

    // Use GL 3.2 (All Platforms)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    // Decide GLSL version
    #ifdef __APPLE__
    // GLSL 150
    const char* glsl_version = "#version 150";
    #else
    // GLSL 130
    const char* glsl_version = "#version 130";
    #endif

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(1280, 720, "Dear ImGui VTKViewer Example", NULL, NULL);
    if (window == NULL)
    {
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Initialize OpenGL loader
    if (gl3wInit() != 0)
    {
        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
        return 1;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows'

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Initialize VtkViewer objects
    // VtkViewer vtkViewer1;
    // vtkViewer1.addActor(actor);

    vtkNew<vtkNamedColors> colors;
    std::array<unsigned char, 4> bkg{{26, 51, 77, 255}};
    colors->SetColor("BkgColor", bkg.data());

    VtkViewer vtkViewer2;
    vtkViewer2.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData()); // Black background
    vtkViewer2.addActor(actor);
    vtkViewer2.addActor(planeActor);
    vtkViewer2.addActor(axes);

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    bool vtk_2_open = true;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    bool isPlaying = false;
    bool isManuelPlayback = true;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        if (show_demo_window)
        {
            ImGui::ShowDemoWindow(&show_demo_window);
        }

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
        {
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.
            ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
            ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
            ImGui::Checkbox("Another Window", &show_another_window);
            ImGui::Checkbox("VTK Viewer #2", &vtk_2_open);

            ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

            if (ImGui::Button("Button")){                            // Buttons return true when clicked (most widgets return true when edited/activated)
                counter++;
            }
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        }
        ImGui::End();

        // 3. Show another simple window.
        if (show_another_window)
        {
            ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
            ImGui::Text("Hello from another window!");
            if (ImGui::Button("Close Me"))
            {
                show_another_window = false;
            }
            ImGui::End();
        }

        // 4. Show a simple VtkViewer Instance (Always Open)
        // ImGui::SetNextWindowSize(ImVec2(360, 240), ImGuiCond_FirstUseEver);
        // ImGui::Begin("Vtk Viewer 1", nullptr, VtkViewer::NoScrollFlags());
        // vtkViewer1.render(); // default render size = ImGui::GetContentRegionAvail()
        // ImGui::End();

        // 5. Show a more complex VtkViewer Instance (Closable, Widgets in Window)
        ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
        if (vtk_2_open)
        {
            static bool loop = false;
            static int vectorIndex = 0;

            ImGui::Begin("Vtk Viewer 2", &vtk_2_open, VtkViewer::NoScrollFlags());

            // Other widgets can be placed in the same window as the VTKViewer
            // However, since the VTKViewer is rendered to size ImGui::GetContentRegionAvail(),
            // it is best to put all widgets first (i.e., render the VTKViewer last).
            // If you want the VTKViewer to be at the top of a window, you can manually calculate
            // and define its size, accounting for the space taken up by other widgets

            auto renderer = vtkViewer2.getRenderer();
            if (ImGui::Button("VTK Background: Black"))
            {
                renderer->SetBackground(0, 0, 0);
            }
            // ImGui::SameLine();
            // if (ImGui::Button("VTK Background: Red"))
            // {
            //     renderer->SetBackground(1, 0, 0);
            // }
            // ImGui::SameLine();
            // if (ImGui::Button("VTK Background: Green"))
            // {
            //     renderer->SetBackground(0, 1, 0);
            // }
            // ImGui::SameLine();
            // if (ImGui::Button("VTK Background: Blue"))
            // {
            //     renderer->SetBackground(0, 0, 1);
            // }

            static float vtk2BkgAlpha = 0.2f;
            ImGui::SliderFloat("Background Alpha", &vtk2BkgAlpha, 0.0f, 1.0f);
            renderer->SetBackgroundAlpha(vtk2BkgAlpha);

            if(isManuelPlayback)
            {
                if(ImGui::SliderInt("Vector Index", &vectorIndex, 0, gravity_vectors.size()-1))
                {
                    vtkViewer2.removeActor(actor);
                    vtkViewer2.removeActor(planeActor);
                    actor = getArrowActor(gravity_vectors.at(vectorIndex));
                    planeActor = getPlaneActor(gravity_vectors.at(vectorIndex));
                    vtkViewer2.addActor(actor);
                    vtkViewer2.addActor(planeActor);
                }

                ImGui::Checkbox("Loop", &loop);
                if(isPlaying)
                {
                    if (ImGui::Button("Stop")) 
                    {
                        isPlaying = false;
                    }
                }
                else if(ImGui::Button("Play"))
                {
                    isPlaying = true;
                }

                if(ImGui::Button("Next Index"))
                {
                    if(vectorIndex >= gravity_vectors.size()-1) 
                    {
                        vectorIndex = 0;
                        vtkViewer2.removeActor(actor);
                        vtkViewer2.removeActor(planeActor);
                        actor = getArrowActor(gravity_vectors.at(vectorIndex));
                        planeActor = getPlaneActor(gravity_vectors.at(vectorIndex));
                        vtkViewer2.addActor(actor);
                        vtkViewer2.addActor(planeActor);
                    }
                    else
                    {
                        vectorIndex++;
                        vtkViewer2.removeActor(actor);
                        vtkViewer2.removeActor(planeActor);
                        actor = getArrowActor(gravity_vectors.at(vectorIndex));
                        planeActor = getPlaneActor(gravity_vectors.at(vectorIndex));
                        vtkViewer2.addActor(actor);
                        vtkViewer2.addActor(planeActor);
                    }
                }
            }

            if(isPlaying)
            {
                if(vectorIndex >= gravity_vectors.size()-1) 
                {
                    if(!loop)
                    {
                        isPlaying = false;
                    }

                    vectorIndex = 0;
                    vtkViewer2.removeActor(actor);
                    vtkViewer2.removeActor(planeActor);
                    actor = getArrowActor(gravity_vectors.at(vectorIndex));
                    planeActor = getPlaneActor(gravity_vectors.at(vectorIndex));
                    vtkViewer2.addActor(actor);
                    vtkViewer2.addActor(planeActor);
                }
                vectorIndex++;
                vtkViewer2.removeActor(actor);
                vtkViewer2.removeActor(planeActor);
                actor = getArrowActor(gravity_vectors.at(vectorIndex));
                planeActor = getPlaneActor(gravity_vectors.at(vectorIndex));
                vtkViewer2.addActor(actor);
                vtkViewer2.addActor(planeActor);
            }
            
            vtkViewer2.render();

            ImGui::End();
        }

        ImGui::Render();

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Update and Render additional Platform Windows
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;

    // int endTime = 200;
    // double frameRate = 5;

    // vtkNew<vtkNamedColors> colors;
    // std::array<unsigned char, 4> bkg{{26, 51, 77, 255}};
    // colors->SetColor("BkgColor", bkg.data());

    // // Create the graphics structure. The renderer renders into the render window
    // vtkNew<vtkRenderWindowInteractor> iren;
    // vtkNew<vtkRenderer> ren1;
    // vtkNew<vtkRenderWindow> renWin;
    // renWin->SetMultiSamples(0);
    // renWin->SetWindowName("Gravity Vector");
    
    // iren->SetRenderWindow(renWin);
    // renWin->AddRenderer(ren1);
    // // ren1->SetBackground(colors->GetColor3d("MistyRose").GetData());
    // ren1->SetBackground(colors->GetColor3d("BkgColor").GetData());
    // ren1->GetActiveCamera()->SetPosition(-1, 0, 0);
    // ren1->GetActiveCamera()->SetFocalPoint(0, 0, 1.0);
    // ren1->GetActiveCamera()->SetViewUp(0, 0, -1);    
    
    // renWin->SetSize(1200, 1200);
    // renWin->Render();

    // // iren->Start();

    // // Create an Animation Scene
    // vtkNew<vtkAnimationScene> scene;

    // scene->SetModeToRealTime();
    // // scene->SetModeToSequence();

    // scene->SetLoop(0);
    // scene->SetFrameRate(frameRate); // FPS
    // scene->SetStartTime(0);
    // scene->SetEndTime(endTime); // how many seconds for it to run for
 
    // // Create an Animation Cue
    // vtkNew<vtkAnimationCue> cue1;
    // cue1->SetStartTime(0);
    // cue1->SetEndTime(endTime); // how many seconds for it to run for
    // scene->AddCue(cue1);

    // // Create cue animator
    // CueAnimator animator;
    // animator.setGrav(gravity_vectors);

    // // Create Cue observer
    // vtkNew<vtkAnimationCueObserver> observer;
    // observer->Renderer = ren1;
    // observer->Animator = &animator;
    // observer->RenWin = renWin;
    // observer->Inter = iren;

    // cue1->AddObserver(vtkCommand::StartAnimationCueEvent, observer);
    // cue1->AddObserver(vtkCommand::EndAnimationCueEvent, observer);
    // cue1->AddObserver(vtkCommand::AnimationCueTickEvent, observer);
    
    // scene->Play();
    // scene->Stop();

    // // iren->Start();

    // return EXIT_SUCCESS;
}