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
#include "metrics.hpp"

#include "vtk_actor_generator.hpp"
#include "tinyfiledialogs.h"

char const* selectedfolderPath;
std::string fPath;

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void runFilter(const int &freq, const float &comp_gain, const float &madg_beta, const int &start_index, const int &end_index, std::vector<std::vector<double>> &gravity_vectors_madg,
 std::vector<std::vector<double>> &gravity_vectors_gt, std::vector<std::vector<double>> &gravity_vectors_comp, std::string &results_suffix)
{
        float deltat = 1.0f/(float)freq;

    // Input
    ComplementaryFilter comp = ComplementaryFilter(deltat, comp_gain);
    ComplementaryFilter comp_mag = ComplementaryFilter(deltat, comp_gain);
    MadgwickFilter madg = MadgwickFilter(deltat, madg_beta);
    MadgwickFilter madg_mag = MadgwickFilter(deltat, madg_beta);

    // CsvReader read = CsvReader("../test_data/");

    std::ofstream est_madg_mag;
    std::ofstream est_madg_no_mag;
    std::ofstream est_comp_mag;
    std::ofstream est_comp_no_mag;

    std::ofstream error_madg_mag;
    std::ofstream error_madg_no_mag;
    std::ofstream error_comp_mag;
    std::ofstream error_comp_no_mag;
    // std::ofstream euler_diff;

    std::vector<Vec3> a = CsvReader::getVec3Data(fPath + "imu_acc.csv");
    std::vector<Vec3> w = CsvReader::getVec3Data(fPath + "imu_gyr.csv");
    std::vector<Vec3> m = CsvReader::getVec3Data(fPath + "imu_mag.csv");
    std::vector<Quaternion> gt = CsvReader::getQuatData(fPath + "opt_quat.csv");

    std::filesystem::create_directories(fPath + results_suffix);

    est_madg_mag.open (fPath + results_suffix + "/est_madg_mag.csv");
    est_madg_no_mag.open (fPath + results_suffix + "/est_madg_no_mag.csv");
    est_comp_mag.open (fPath + results_suffix + "/est_comp_mag.csv");
    est_comp_no_mag.open (fPath + results_suffix + "/est_comp_no_mag.csv");

    error_madg_mag.open (fPath + results_suffix + "/error_madg_mag.csv");
    error_madg_no_mag.open (fPath + results_suffix + "/error_madg_no_mag.csv");
    error_comp_mag.open (fPath + results_suffix + "/error_comp_mag.csv");
    error_comp_no_mag.open (fPath + results_suffix + "/error_comp_no_mag.csv");

    // euler_diff.open ("../results/euler_diff.csv");

    Quaternion initial_state = Quaternion::getOrientationFromAccMag(a.at(start_index-1), m.at(start_index-1));
    comp_mag.setInitialState(initial_state);
    madg_mag.setInitialState(initial_state);
    comp.setInitialState(initial_state);
    madg.setInitialState(initial_state);

    for (int i = start_index; i < end_index+1; ++i)
    {
        comp_mag.updateFilter(w.at(i), a.at(i), m.at(i));
        comp.updateFilter(w.at(i), a.at(i));

        madg_mag.updateMARGFilter(w.at(i), a.at(i), m.at(i));        
        madg.updateIMUFilter(w.at(i), a.at(i));

        est_madg_mag << madg_mag.q.q_1 << "," << madg_mag.q.q_2 << "," << madg_mag.q.q_3 << "," << madg_mag.q.q_4 << "\n";
        est_comp_mag << comp_mag.q.q_1 << "," << comp_mag.q.q_2 << "," << comp_mag.q.q_3 << "," << comp_mag.q.q_4 << "\n";

        est_madg_no_mag << madg.q.q_1 << "," << madg.q.q_2 << "," << madg.q.q_3 << "," << madg.q.q_4 << "\n";
        est_comp_no_mag << comp.q.q_1 << "," << comp.q.q_2 << "," << comp.q.q_3 << "," << comp.q.q_4 << "\n";

        // Quaternion enu_est = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg_mag.q);
        // Quaternion err_quat = Metrics::error_quaternion(read.gt.at(i), madg_mag.q);
        // Quaternion err_quat = Metrics::error_quaternion_earth(read.gt.at(i), madg_mag.q);
        // Quaternion err_quat = Metrics::error_quaternion(read.gt.at(i), enu_est);
        // These are in ENU
        Quaternion err_quat_madg_mag = Metrics::error_quaternion_earth(gt.at(i), madg_mag.q);
        Quaternion err_quat_madg_no_mag = Metrics::error_quaternion_earth(gt.at(i), madg.q);
        Quaternion err_quat_comp_mag = Metrics::error_quaternion_earth(gt.at(i), comp_mag.q);
        Quaternion err_quat_comp_no_mag = Metrics::error_quaternion_earth(gt.at(i), comp.q);

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

        Quaternion enu_est = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg_mag.q);
        float myaw = enu_est.yaw();
        float mpitch = enu_est.pitch();
        float mroll = enu_est.roll();
        // gravity_vectors_madg.push_back({-sin(mpitch), cos(mpitch)*sin(mroll), -cos(mpitch)*cos(mroll)});
        gravity_vectors_madg.push_back({mroll, mpitch, myaw});

        float gyaw = gt.at(i).yaw();
        float gpitch = gt.at(i).pitch();
        float groll = gt.at(i).roll();
        // gravity_vectors_gt.push_back({-sin(gpitch), cos(gpitch)*sin(groll), -cos(gpitch)*cos(groll)});
        gravity_vectors_gt.push_back({groll, gpitch, gyaw});

        // TODO: Something wrong here, is it in ENU?
        Quaternion enu_comp = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), comp_mag.q);
        float cyaw = enu_comp.yaw();
        float cpitch = enu_comp.pitch();
        float croll = enu_comp.roll();
        // gravity_vectors_comp.push_back({-sin(cpitch), cos(cpitch)*sin(croll), -cos(cpitch)*cos(croll)});
        gravity_vectors_comp.push_back({croll, cpitch, cyaw});
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
}

int main(int argc, char* argv[])
{
    std::vector<std::vector<double>> gravity_vectors_madg;
    std::vector<std::vector<double>> gravity_vectors_gt;
    std::vector<std::vector<double>> gravity_vectors_comp;

    // Setup pipeline
    vtkSmartPointer<vtkActor> arrowActor_madg;
    vtkSmartPointer<vtkActor> planeActor_madg;

    vtkSmartPointer<vtkActor> arrowActor_gt;
    vtkSmartPointer<vtkActor> planeActor_gt;

    vtkSmartPointer<vtkActor> arrowActor_comp;
    vtkSmartPointer<vtkActor> planeActor_comp;

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
    GLFWwindow* window = glfwCreateWindow(1360, 720, "Orientation Estimation Visualization", NULL, NULL);
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

    VtkViewer vtkViewer_gt;

    VtkViewer vtkViewer_comp;

    // Our state
    bool vtk_madg_open = false;
    bool vtk_gt_open = false;
    bool vtk_comp_open = false;

    bool isPlaying = false;
    bool isManuelPlayback = true;

    bool show_demo_window = true;
    bool isCalculated = false;
    static int vectorIndex = 0;
    std::vector<Vec3> rmse;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
        // ImGui::ShowDemoWindow(&show_demo_window);

        {        
            static bool loop = false;
            static bool black = false;
            ImGui::Begin("Menu");

            if (ImGui::Button("Select Folder"))
            {
                selectedfolderPath = NULL;
                fPath = "";

                // If the dialog is open and we try to open it again, stop the program until it is closed
                selectedfolderPath = tinyfd_selectFolderDialog("Select Folder with data files in it", "");
                if (!selectedfolderPath)
                {

                }
                else
                {
                    fPath = selectedfolderPath;
                }
            }

            ImGui::Text("%s", fPath.c_str());

            static int start_index = 10000;
            ImGui::InputInt("Starting index", &start_index);

            static int end_index = 45000;
            ImGui::InputInt("Ending index", &end_index);

            static int freq = 286;
            ImGui::InputInt("Sample frequency", &freq);

            static float comp_gain = 0.2f;
            ImGui::InputFloat("Comp. gain", &comp_gain, 0.01f, 1.0f, "%.3f");

            static float madg_beta = 0.2f;
            ImGui::InputFloat("Madg. beta", &madg_beta, 0.01f, 1.0f, "%.3f");

            if(ImGui::Button("Calculate"))
            {
                if(fPath.length() != 0)
                {
                    // ImGui::OpenPopup("ERROR");
                    vtkViewer_madg.removeActor(arrowActor_madg);
                    vtkViewer_madg.removeActor(planeActor_madg);
                    vtkViewer_gt.removeActor(arrowActor_gt);
                    vtkViewer_gt.removeActor(planeActor_gt);
                    vtkViewer_comp.removeActor(arrowActor_comp);
                    vtkViewer_comp.removeActor(planeActor_comp);
                    gravity_vectors_madg.clear();
                    gravity_vectors_gt.clear();
                    gravity_vectors_comp.clear();

                    std::string results_suffix = "_" + std::to_string(freq) + "_" + std::to_string(comp_gain) + "_" + std::to_string(madg_beta) + "_" + std::to_string(start_index) + "_" + std::to_string(end_index);

                    std::cout << fPath << std::endl;
                    std::string str = fPath.substr(0, fPath.length()-2);
                    char ch = '/';            
                    size_t index = str.rfind(ch);
                    std::cout << index << std::endl;
                    if (index != std::string::npos) 
                    {
                        results_suffix = "out_" + fPath.substr(index+1, fPath.length()-2) + results_suffix;
                    }
                    else
                    {
                        results_suffix = "out" + results_suffix;
                    }
                
                    runFilter(freq, comp_gain, madg_beta, start_index, end_index, gravity_vectors_madg, gravity_vectors_gt, gravity_vectors_comp, results_suffix);
                    arrowActor_madg = getArrowActor(gravity_vectors_madg.at(0));
                    planeActor_madg = getPlaneActor(gravity_vectors_madg.at(0));

                    arrowActor_gt = getArrowActor(gravity_vectors_gt.at(0));
                    planeActor_gt = getPlaneActor(gravity_vectors_gt.at(0));

                    arrowActor_comp = getArrowActor(gravity_vectors_comp.at(0));
                    planeActor_comp = getPlaneActor(gravity_vectors_comp.at(0));

                    vtkViewer_madg.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
                    vtkViewer_madg.addActor(arrowActor_madg);
                    vtkViewer_madg.addActor(planeActor_madg);
                    vtkViewer_madg.addActor(axes);

                    vtkViewer_gt.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
                    vtkViewer_gt.addActor(arrowActor_gt);
                    vtkViewer_gt.addActor(planeActor_gt);
                    vtkViewer_gt.addActor(axes);

                    vtkViewer_comp.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
                    vtkViewer_comp.addActor(arrowActor_comp);
                    vtkViewer_comp.addActor(planeActor_comp);
                    vtkViewer_comp.addActor(axes);

                    rmse = CsvReader::getRMSE(fPath, results_suffix);
                    
                    isCalculated = true;
                    vectorIndex = 0;
                }
                else
                {
                    ImGui::OpenPopup("No data, no problem...?");
                }
            }

            // Always center this window when appearing
            ImVec2 center = ImGui::GetMainViewport()->GetCenter();
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            if (ImGui::BeginPopupModal("No data, no problem...?", NULL, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::Text("Please select a folder containing data files!\n\n");
                ImGui::Separator();

                if (ImGui::Button("OK", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }
                ImGui::SetItemDefaultFocus();
                ImGui::EndPopup();
            }

            if(isCalculated)
            {
                // Other widgets can be placed in the same window as the VTKViewer
                // However, since the VTKViewer is rendered to size ImGui::GetContentRegionAvail(),
                // it is best to put all widgets first (i.e., render the VTKViewer last).
                // If you want the VTKViewer to be at the top of a window, you can manually calculate
                // and define its size, accounting for the space taken up by other widgets

                auto renderer = vtkViewer_madg.getRenderer();
                auto renderer_gt = vtkViewer_gt.getRenderer();
                auto renderer_comp = vtkViewer_comp.getRenderer();

                ImGui::Checkbox("Black background", &black);
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
                        if(ImGui::Button("Stop")) 
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

                ImGui::Text("Show vector visualization windows:");
                ImGui::Checkbox("Ground truth", &vtk_gt_open);
                ImGui::Checkbox("Madgwick Filter estimation", &vtk_madg_open);
                ImGui::Checkbox("Complementary filter estimation", &vtk_comp_open);

                ImGui::Text("RMSE Values (degrees): ");
                if (ImGui::BeginTable("table1", 5))
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("Madg. w/ mag");
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("Madg.");
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("Comp. w/ mag");
                    ImGui::TableSetColumnIndex(4);
                    ImGui::Text("Comp.");
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Incl. error");
                    for (int column = 1; column < 5; column++)
                    {
                        ImGui::TableSetColumnIndex(column);
                        ImGui::Text("%f", rmse.at(column-1).y*180/M_PI);
                    }
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Heading error");
                    for (int column = 1; column < 5; column++)
                    {
                        ImGui::TableSetColumnIndex(column);
                        ImGui::Text("%f", rmse.at(column-1).z*180/M_PI);
                    }
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Total error");
                    for (int column = 1; column < 5; column++)
                    {
                        ImGui::TableSetColumnIndex(column);
                        ImGui::Text("%f", rmse.at(column-1).x*180/M_PI);
                    }
                    ImGui::EndTable();
                }

                ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            }
            ImGui::End();
        }

        if(vtk_madg_open)
        {
            ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
            ImGui::Begin("Madgwick Filter estimation", &vtk_madg_open, VtkViewer::NoScrollFlags());
            ImGui::Text("Euler angles (degrees): ");
            if (ImGui::BeginTable("table2", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", gravity_vectors_madg.at(vectorIndex).at(0)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", gravity_vectors_madg.at(vectorIndex).at(1)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", gravity_vectors_madg.at(vectorIndex).at(2)*180/M_PI);
                ImGui::EndTable();
            }
            ImGui::Text("Euler angle errors (degrees): ");
            if (ImGui::BeginTable("table5", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", (gravity_vectors_madg.at(vectorIndex).at(0)-gravity_vectors_gt.at(vectorIndex).at(0))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", (gravity_vectors_madg.at(vectorIndex).at(1)-gravity_vectors_gt.at(vectorIndex).at(1))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", (gravity_vectors_madg.at(vectorIndex).at(2)-gravity_vectors_gt.at(vectorIndex).at(2))*180/M_PI);
                ImGui::EndTable();
            }
            vtkViewer_madg.render();
            ImGui::End();
        }

        if(vtk_gt_open)
        {
            ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
            ImGui::Begin("Ground truth", &vtk_gt_open, VtkViewer::NoScrollFlags());
            ImGui::Text("Euler angles (degrees): ");
            if (ImGui::BeginTable("table3", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", gravity_vectors_gt.at(vectorIndex).at(0)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", gravity_vectors_gt.at(vectorIndex).at(1)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", gravity_vectors_gt.at(vectorIndex).at(2)*180/M_PI);
                ImGui::EndTable();
            }
            vtkViewer_gt.render();
            ImGui::End();
        }

        if(vtk_comp_open)
        {
            ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
            ImGui::Begin("Complementary Filter estimation", &vtk_comp_open, VtkViewer::NoScrollFlags());
            ImGui::Text("Euler angles (degrees): ");
            if (ImGui::BeginTable("table4", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", gravity_vectors_comp.at(vectorIndex).at(0)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", gravity_vectors_comp.at(vectorIndex).at(1)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", gravity_vectors_comp.at(vectorIndex).at(2)*180/M_PI);
                ImGui::EndTable();
            }
            ImGui::Text("Euler angle errors (degrees): ");
            if (ImGui::BeginTable("table6", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", (gravity_vectors_comp.at(vectorIndex).at(0)-gravity_vectors_gt.at(vectorIndex).at(0))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", (gravity_vectors_comp.at(vectorIndex).at(1)-gravity_vectors_gt.at(vectorIndex).at(1))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", (gravity_vectors_comp.at(vectorIndex).at(2)-gravity_vectors_gt.at(vectorIndex).at(2))*180/M_PI);
                ImGui::EndTable();
            }
            vtkViewer_comp.render();
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
}