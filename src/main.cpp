#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <array>

// OpenGL Loader
#include <GL/gl3w.h> // GL3w, initialized with gl3wInit() below

// Include glfw3.h after our OpenGL definitions
#include <GLFW/glfw3.h>

// ImGui + imgui-vtk
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "vtk_viewer.hpp"

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

#include "vtk_actor_generator.hpp"

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
    std::vector<std::vector<double>> gravity_vectors_madg;
    std::vector<std::vector<double>> gravity_vectors_gt;
    std::vector<std::vector<double>> gravity_vectors_comp;

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
            // TODO: ENU frame???
            Quaternion enu_est = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg_mag.q);
            float myaw = enu_est.yaw();
            float mpitch = enu_est.pitch();
            float mroll = enu_est.roll();
            gravity_vectors_madg.push_back({-sin(mpitch), cos(mpitch)*sin(mroll), -cos(mpitch)*cos(mroll)});

            Quaternion enu_gt = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), read.gt.at(i));
            float gyaw = enu_gt.yaw();
            float gpitch = enu_gt.pitch();
            float groll = enu_gt.roll();
            gravity_vectors_gt.push_back({-sin(gpitch), cos(gpitch)*sin(groll), -cos(gpitch)*cos(groll)});

            Quaternion enu_comp = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), comp_mag.q);
            float cyaw = enu_comp.yaw();
            float cpitch = enu_comp.pitch();
            float croll = enu_comp.roll();
            gravity_vectors_comp.push_back({-sin(cpitch), cos(cpitch)*sin(croll), -cos(cpitch)*cos(croll)});
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
    auto arrowActor_madg = getArrowActor(gravity_vectors_madg.at(0));
    auto planeActor_madg = getPlaneActor(gravity_vectors_madg.at(0));

    auto arrowActor_gt = getArrowActor(gravity_vectors_gt.at(0));
    auto planeActor_gt = getPlaneActor(gravity_vectors_gt.at(0));

    auto arrowActor_comp = getArrowActor(gravity_vectors_comp.at(0));
    auto planeActor_comp = getPlaneActor(gravity_vectors_comp.at(0));

    // AXES
    auto transformA = vtkSmartPointer<vtkTransform>::New();
    transformA->Translate(0.0, 0.0, 0.0);

    auto axes = vtkSmartPointer<vtkAxesActor>::New();

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
    GLFWwindow* window = glfwCreateWindow(1360, 720, "Dear ImGui VTKViewer Example", NULL, NULL);
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
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable; // Enable Docking
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable; // Enable Multi-Viewport / Platform Windows

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    vtkNew<vtkNamedColors> colors;
    std::array<unsigned char, 4> bkg{{26, 51, 77, 255}};
    colors->SetColor("BkgColor", bkg.data());

    VtkViewer vtkViewer_madg;
    vtkViewer_madg.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
    vtkViewer_madg.addActor(arrowActor_madg);
    vtkViewer_madg.addActor(planeActor_madg);
    vtkViewer_madg.addActor(axes);

    VtkViewer vtkViewer_gt;
    vtkViewer_gt.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
    vtkViewer_gt.addActor(arrowActor_gt);
    vtkViewer_gt.addActor(planeActor_gt);
    vtkViewer_gt.addActor(axes);

    VtkViewer vtkViewer_comp;
    vtkViewer_comp.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
    vtkViewer_comp.addActor(arrowActor_comp);
    vtkViewer_comp.addActor(planeActor_comp);
    vtkViewer_comp.addActor(axes);

    // Our state
    bool vtk_madg_open = false;
    bool vtk_gt_open = false;
    bool vtk_comp_open = false;

    bool isPlaying = false;
    bool isManuelPlayback = true;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
        {
        
            static bool loop = false;
            static int vectorIndex = 0;
            static bool black = false;
            ImGui::Begin("Menu");

            // Other widgets can be placed in the same window as the VTKViewer
            // However, since the VTKViewer is rendered to size ImGui::GetContentRegionAvail(),
            // it is best to put all widgets first (i.e., render the VTKViewer last).
            // If you want the VTKViewer to be at the top of a window, you can manually calculate
            // and define its size, accounting for the space taken up by other widgets

            if(vtk_madg_open)
            {
                ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
                ImGui::Begin("Madgwick Estimation", nullptr, VtkViewer::NoScrollFlags());
                vtkViewer_madg.render();
                ImGui::End();
            }

            if(vtk_gt_open)
            {
                ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
                ImGui::Begin("Ground truth", nullptr, VtkViewer::NoScrollFlags());
                vtkViewer_gt.render();
                ImGui::End();
            }

            if(vtk_comp_open)
            {
                ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
                ImGui::Begin("Complementary Estimation", nullptr, VtkViewer::NoScrollFlags());
                vtkViewer_comp.render();
                ImGui::End();
            }

            auto renderer = vtkViewer_madg.getRenderer();
            auto renderer_gt = vtkViewer_gt.getRenderer();
            auto renderer_comp = vtkViewer_comp.getRenderer();

            ImGui::Checkbox("Black VTK Background", &black);
            if(black)
            {
                renderer->SetBackground(0, 0, 0);
                renderer_gt->SetBackground(0, 0, 0);
                renderer_comp->SetBackground(0, 0, 0);
            }

            if(!black)
            {
                renderer->SetBackground(colors->GetColor3d("BkgColor").GetData());
                renderer_gt->SetBackground(colors->GetColor3d("BkgColor").GetData());
                renderer_comp->SetBackground(colors->GetColor3d("BkgColor").GetData());
            }

            static float vtk2BkgAlpha = 0.2f;
            ImGui::SliderFloat("Background Alpha", &vtk2BkgAlpha, 0.0f, 1.0f);
            renderer->SetBackgroundAlpha(vtk2BkgAlpha);
            renderer_gt->SetBackgroundAlpha(vtk2BkgAlpha);
            renderer_comp->SetBackgroundAlpha(vtk2BkgAlpha);

            if(isManuelPlayback)
            {
                if(ImGui::SliderInt("Vector Index", &vectorIndex, 0, gravity_vectors_madg.size()-1))
                {
                    vtkViewer_madg.removeActor(arrowActor_madg);
                    vtkViewer_madg.removeActor(planeActor_madg);
                    arrowActor_madg = getArrowActor(gravity_vectors_madg.at(vectorIndex));
                    planeActor_madg = getPlaneActor(gravity_vectors_madg.at(vectorIndex));
                    vtkViewer_madg.addActor(arrowActor_madg);
                    vtkViewer_madg.addActor(planeActor_madg);

                    vtkViewer_gt.removeActor(arrowActor_gt);
                    vtkViewer_gt.removeActor(planeActor_gt);
                    arrowActor_gt = getArrowActor(gravity_vectors_gt.at(vectorIndex));
                    planeActor_gt = getPlaneActor(gravity_vectors_gt.at(vectorIndex));
                    vtkViewer_gt.addActor(arrowActor_gt);
                    vtkViewer_gt.addActor(planeActor_gt);

                    vtkViewer_comp.removeActor(arrowActor_comp);
                    vtkViewer_comp.removeActor(planeActor_comp);
                    arrowActor_comp = getArrowActor(gravity_vectors_comp.at(vectorIndex));
                    planeActor_comp = getPlaneActor(gravity_vectors_comp.at(vectorIndex));
                    vtkViewer_comp.addActor(arrowActor_comp);
                    vtkViewer_comp.addActor(planeActor_comp);
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
                    // std::cout << gravity_vectors.at(vectorIndex).at(0) << " " << gravity_vectors.at(vectorIndex).at(1) << " " << gravity_vectors.at(vectorIndex).at(2) << std::endl;
                    if(vectorIndex >= gravity_vectors_madg.size()-1) 
                    {
                        vectorIndex = 0;
                        vtkViewer_madg.removeActor(arrowActor_madg);
                        vtkViewer_madg.removeActor(planeActor_madg);
                        arrowActor_madg = getArrowActor(gravity_vectors_madg.at(vectorIndex));
                        planeActor_madg = getPlaneActor(gravity_vectors_madg.at(vectorIndex));
                        vtkViewer_madg.addActor(arrowActor_madg);
                        vtkViewer_madg.addActor(planeActor_madg);

                        vtkViewer_gt.removeActor(arrowActor_gt);
                        vtkViewer_gt.removeActor(planeActor_gt);
                        arrowActor_gt = getArrowActor(gravity_vectors_gt.at(vectorIndex));
                        planeActor_gt = getPlaneActor(gravity_vectors_gt.at(vectorIndex));
                        vtkViewer_gt.addActor(arrowActor_gt);
                        vtkViewer_gt.addActor(planeActor_gt);

                        vtkViewer_comp.removeActor(arrowActor_comp);
                        vtkViewer_comp.removeActor(planeActor_comp);
                        arrowActor_comp = getArrowActor(gravity_vectors_comp.at(vectorIndex));
                        planeActor_comp = getPlaneActor(gravity_vectors_comp.at(vectorIndex));
                        vtkViewer_comp.addActor(arrowActor_comp);
                        vtkViewer_comp.addActor(planeActor_comp);
                    }
                    else
                    {
                        vectorIndex++;
                        vtkViewer_madg.removeActor(arrowActor_madg);
                        vtkViewer_madg.removeActor(planeActor_madg);
                        arrowActor_madg = getArrowActor(gravity_vectors_madg.at(vectorIndex));
                        planeActor_madg = getPlaneActor(gravity_vectors_madg.at(vectorIndex));
                        vtkViewer_madg.addActor(arrowActor_madg);
                        vtkViewer_madg.addActor(planeActor_madg);

                        vtkViewer_gt.removeActor(arrowActor_gt);
                        vtkViewer_gt.removeActor(planeActor_gt);
                        arrowActor_gt = getArrowActor(gravity_vectors_gt.at(vectorIndex));
                        planeActor_gt = getPlaneActor(gravity_vectors_gt.at(vectorIndex));
                        vtkViewer_gt.addActor(arrowActor_gt);
                        vtkViewer_gt.addActor(planeActor_gt);

                        vtkViewer_comp.removeActor(arrowActor_comp);
                        vtkViewer_comp.removeActor(planeActor_comp);
                        arrowActor_comp = getArrowActor(gravity_vectors_comp.at(vectorIndex));
                        planeActor_comp = getPlaneActor(gravity_vectors_comp.at(vectorIndex));
                        vtkViewer_comp.addActor(arrowActor_comp);
                        vtkViewer_comp.addActor(planeActor_comp);
                    }
                }
            }

            if(isPlaying)
            {
                if(vectorIndex >= gravity_vectors_madg.size()-1) 
                {
                    if(!loop)
                    {
                        isPlaying = false;
                    }

                    vectorIndex = 0;
                    vtkViewer_madg.removeActor(arrowActor_madg);
                    vtkViewer_madg.removeActor(planeActor_madg);
                    arrowActor_madg = getArrowActor(gravity_vectors_madg.at(vectorIndex));
                    planeActor_madg = getPlaneActor(gravity_vectors_madg.at(vectorIndex));
                    vtkViewer_madg.addActor(arrowActor_madg);
                    vtkViewer_madg.addActor(planeActor_madg);

                    vtkViewer_gt.removeActor(arrowActor_gt);
                    vtkViewer_gt.removeActor(planeActor_gt);
                    arrowActor_gt = getArrowActor(gravity_vectors_gt.at(vectorIndex));
                    planeActor_gt = getPlaneActor(gravity_vectors_gt.at(vectorIndex));
                    vtkViewer_gt.addActor(arrowActor_gt);
                    vtkViewer_gt.addActor(planeActor_gt);

                    vtkViewer_comp.removeActor(arrowActor_comp);
                    vtkViewer_comp.removeActor(planeActor_comp);
                    arrowActor_comp = getArrowActor(gravity_vectors_comp.at(vectorIndex));
                    planeActor_comp = getPlaneActor(gravity_vectors_comp.at(vectorIndex));
                    vtkViewer_comp.addActor(arrowActor_comp);
                    vtkViewer_comp.addActor(planeActor_comp);
                }
                vectorIndex++;
                vtkViewer_madg.removeActor(arrowActor_madg);
                vtkViewer_madg.removeActor(planeActor_madg);
                arrowActor_madg = getArrowActor(gravity_vectors_madg.at(vectorIndex));
                planeActor_madg = getPlaneActor(gravity_vectors_madg.at(vectorIndex));
                vtkViewer_madg.addActor(arrowActor_madg);
                vtkViewer_madg.addActor(planeActor_madg);

                vtkViewer_gt.removeActor(arrowActor_gt);
                vtkViewer_gt.removeActor(planeActor_gt);
                arrowActor_gt = getArrowActor(gravity_vectors_gt.at(vectorIndex));
                planeActor_gt = getPlaneActor(gravity_vectors_gt.at(vectorIndex));
                vtkViewer_gt.addActor(arrowActor_gt);
                vtkViewer_gt.addActor(planeActor_gt);

                vtkViewer_comp.removeActor(arrowActor_comp);
                vtkViewer_comp.removeActor(planeActor_comp);
                arrowActor_comp = getArrowActor(gravity_vectors_comp.at(vectorIndex));
                planeActor_comp = getPlaneActor(gravity_vectors_comp.at(vectorIndex));
                vtkViewer_comp.addActor(arrowActor_comp);
                vtkViewer_comp.addActor(planeActor_comp);
            }

            ImGui::Text("VTK Windows:");
            ImGui::Checkbox("Ground Truth", &vtk_gt_open);
            ImGui::Checkbox("Madgwick", &vtk_madg_open);
            ImGui::Checkbox("Complementary", &vtk_comp_open);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        }
        ImGui::End();

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
}