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

static std::string _labelPrefix(const char* const label)
{
	float width = ImGui::CalcItemWidth();

	float x = ImGui::GetCursorPosX();
	ImGui::Text("%s", label);
	ImGui::SameLine();
	ImGui::SetCursorPosX(x + width * 0.6f + ImGui::GetStyle().ItemInnerSpacing.x);
	ImGui::SetNextItemWidth(-1);

	std::string labelID = "##";
	labelID += label;

	return labelID;
}

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

std::string runFilter(const int &freq, const float &comp_gain, const float &madg_beta, int &start_index, int &end_index, const bool &max_index, std::vector<std::vector<double>> &gravity_vectors_madg_mag,
 std::vector<std::vector<double>> &gravity_vectors_madg, std::vector<std::vector<double>> &gravity_vectors_gt, std::vector<std::vector<double>> &gravity_vectors_comp_mag, std::vector<std::vector<double>> &gravity_vectors_comp)
{
    float deltat = 1.0f/(float)freq;

    // Input
    ComplementaryFilter comp = ComplementaryFilter(deltat, comp_gain);
    ComplementaryFilter comp_mag = ComplementaryFilter(deltat, comp_gain);
    MadgwickFilter madg = MadgwickFilter(deltat, madg_beta);
    MadgwickFilter madg_mag = MadgwickFilter(deltat, madg_beta);

    std::ofstream est_madg_mag;
    std::ofstream est_madg_no_mag;
    std::ofstream est_comp_mag;
    std::ofstream est_comp_no_mag;

    std::ofstream error_madg_mag;
    std::ofstream error_madg_no_mag;
    std::ofstream error_comp_mag;
    std::ofstream error_comp_no_mag;

    std::vector<Vec3> a = CsvReader::getVec3Data(fPath + "imu_acc.csv");
    std::vector<Vec3> w = CsvReader::getVec3Data(fPath + "imu_gyr.csv");
    std::vector<Vec3> m = CsvReader::getVec3Data(fPath + "imu_mag.csv");
    std::vector<Quaternion> gt = CsvReader::getQuatData(fPath + "opt_quat.csv");

    if(max_index)
    {
        end_index = a.size();
    }

    Quaternion initial_state;
    if(start_index < 1)
    {
        start_index = 0;
        initial_state = Quaternion::getOrientationFromAccMag(a.at(0), m.at(0));        
    }
    else
    {
        initial_state = Quaternion::getOrientationFromAccMag(a.at(start_index-1), m.at(start_index-1));
    }

    std::string results_suffix = "_" +std::to_string(freq) + "_" + std::to_string(comp_gain) + "_" + std::to_string(madg_beta) + "_" + std::to_string(start_index) + "_" + std::to_string(end_index);

    std::string str = fPath.substr(0, fPath.length()-2);
    char ch = '/';
    size_t index = str.rfind(ch);

    if (index != std::string::npos)
    {
        results_suffix = "out_" + fPath.substr(index+1, fPath.length()-2-index) + results_suffix;
    }
    else
    {
        results_suffix = "out" + results_suffix;
    }

    std::filesystem::create_directories(fPath + results_suffix);

    est_madg_mag.open(fPath + results_suffix + "/est_madg_mag.csv");
    est_madg_no_mag.open(fPath + results_suffix + "/est_madg_no_mag.csv");
    est_comp_mag.open(fPath + results_suffix + "/est_comp_mag.csv");
    est_comp_no_mag.open(fPath + results_suffix + "/est_comp_no_mag.csv");

    error_madg_mag.open(fPath + results_suffix + "/error_madg_mag.csv");
    error_madg_no_mag.open(fPath + results_suffix + "/error_madg_no_mag.csv");
    error_comp_mag.open(fPath + results_suffix + "/error_comp_mag.csv");
    error_comp_no_mag.open(fPath + results_suffix + "/error_comp_no_mag.csv");  

    comp_mag.setInitialState(initial_state);
    madg_mag.setInitialState(initial_state);
    comp.setInitialState(initial_state);
    madg.setInitialState(initial_state);

    Quaternion enu_madg_mag;
    Quaternion enu_madg;
    Quaternion enu_comp_mag;
    Quaternion enu_comp;

    for (int i = start_index; i < end_index; ++i)
    {
        comp_mag.updateFilter(w.at(i), a.at(i), m.at(i));
        comp.updateFilter(w.at(i), a.at(i));

        madg_mag.updateMARGFilter(w.at(i), a.at(i), m.at(i));        
        madg.updateIMUFilter(w.at(i), a.at(i));

        // Convert to ENU from NED
        enu_madg_mag = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg_mag.q);
        enu_madg = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg.q);
        enu_comp_mag = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), comp_mag.q);
        enu_comp = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), comp.q);

        // est_madg_mag << madg_mag.q.q_1 << "," << madg_mag.q.q_2 << "," << madg_mag.q.q_3 << "," << madg_mag.q.q_4 << "\n";
        // est_comp_mag << comp_mag.q.q_1 << "," << comp_mag.q.q_2 << "," << comp_mag.q.q_3 << "," << comp_mag.q.q_4 << "\n";

        // est_madg_no_mag << madg.q.q_1 << "," << madg.q.q_2 << "," << madg.q.q_3 << "," << madg.q.q_4 << "\n";
        // est_comp_no_mag << comp.q.q_1 << "," << comp.q.q_2 << "," << comp.q.q_3 << "," << comp.q.q_4 << "\n";

        est_madg_mag << enu_madg_mag.q_1 << "," << enu_madg_mag.q_2 << "," << enu_madg_mag.q_3 << "," << enu_madg_mag.q_4 << "\n";
        est_comp_mag << enu_comp_mag.q_1 << "," << enu_comp_mag.q_2 << "," << enu_comp_mag.q_3 << "," << enu_comp_mag.q_4 << "\n";

        est_madg_no_mag << enu_madg.q_1 << "," << enu_madg.q_2 << "," << enu_madg.q_3 << "," << enu_madg.q_4 << "\n";
        est_comp_no_mag << enu_comp.q_1 << "," << enu_comp.q_2 << "," << enu_comp.q_3 << "," << enu_comp.q_4 << "\n";

        // Quaternion enu_est = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg_mag.q);
        // Quaternion err_quat = Metrics::error_quaternion(read.gt.at(i), madg_mag.q);
        // Quaternion err_quat = Metrics::error_quaternion_earth(read.gt.at(i), madg_mag.q);
        // Quaternion err_quat = Metrics::error_quaternion(read.gt.at(i), enu_est);
        // These are in ENU
        if(!gt.at(i).isNaN())
        {
            Quaternion err_quat_madg_mag = Metrics::error_quaternion_earth(gt.at(i), enu_madg_mag);
            Quaternion err_quat_madg_no_mag = Metrics::error_quaternion_earth(gt.at(i), enu_madg);
            Quaternion err_quat_comp_mag = Metrics::error_quaternion_earth(gt.at(i), enu_comp_mag);
            Quaternion err_quat_comp_no_mag = Metrics::error_quaternion_earth(gt.at(i), enu_comp);

            error_madg_mag << Metrics::total_error(err_quat_madg_mag) << "," << Metrics::inclination_error(err_quat_madg_mag) << "," << Metrics::heading_error(err_quat_madg_mag) << "\n";
            error_madg_no_mag << Metrics::total_error(err_quat_madg_no_mag) << "," << Metrics::inclination_error(err_quat_madg_no_mag) << "," << Metrics::heading_error(err_quat_madg_no_mag) << "\n";
            error_comp_mag << Metrics::total_error(err_quat_comp_mag) << "," << Metrics::inclination_error(err_quat_comp_mag) << "," << Metrics::heading_error(err_quat_comp_mag) << "\n";
            error_comp_no_mag << Metrics::total_error(err_quat_comp_no_mag) << "," << Metrics::inclination_error(err_quat_comp_no_mag) << "," << Metrics::heading_error(err_quat_comp_no_mag) << "\n";
        }
        else
        {
            error_madg_mag << 9999 << "," << 9999 << "," << 9999 << "\n";
            error_madg_no_mag << 9999 << "," << 9999 << "," << 9999 << "\n";
            error_comp_mag << 9999 << "," << 9999 << "," << 9999 << "\n";
            error_comp_no_mag << 9999 << "," << 9999 << "," << 9999 << "\n";
        }

        // float roll_diff = Metrics::euler_roll_diff(read.gt.at(i), madg_mag.q);
        // float pitch_diff = Metrics::euler_pitch_diff(read.gt.at(i), madg_mag.q);
        // float yaw_diff = Metrics::euler_yaw_diff(read.gt.at(i), madg_mag.q);
        // float roll_diff = Metrics::euler_roll_diff(read.gt.at(i), enu_est);
        // float pitch_diff = Metrics::euler_pitch_diff(read.gt.at(i), enu_est);
        // float yaw_diff = Metrics::euler_yaw_diff(read.gt.at(i), enu_est);

        // euler_diff << roll_diff << "," << pitch_diff << "," << yaw_diff << "\n";

        gravity_vectors_madg.push_back({enu_madg.roll(), enu_madg.pitch(), enu_madg.yaw()});
        gravity_vectors_madg_mag.push_back({enu_madg_mag.roll(), enu_madg_mag.pitch(), enu_madg_mag.yaw()});
        gravity_vectors_gt.push_back({gt.at(i).roll(), gt.at(i).pitch(), gt.at(i).yaw()});
        gravity_vectors_comp.push_back({enu_comp.roll(), enu_comp.pitch(), enu_comp.yaw()});
        gravity_vectors_comp_mag.push_back({enu_comp_mag.roll(), enu_comp_mag.pitch(), enu_comp_mag.yaw()});
    }

    est_madg_mag.close();
    est_madg_no_mag.close();
    est_comp_mag.close();
    est_comp_no_mag.close();

    error_madg_mag.close();
    error_madg_no_mag.close();
    error_comp_mag.close();
    error_comp_no_mag.close();

    return results_suffix;
}

int main(int argc, char* argv[])
{
    // Set background color
    vtkNew<vtkNamedColors> colors;
    std::array<unsigned char, 4> bkg{{26, 51, 77, 255}};
    colors->SetColor("BkgColor", bkg.data());

    // Init vtkViewers
    VtkViewer vtk_viewer_madg_mag;
    VtkViewer vtk_viewer_gt;
    VtkViewer vtk_viewer_comp_mag;
    VtkViewer vtk_viewer_madg;
    VtkViewer vtk_viewer_comp;

    std::vector<std::vector<double>> gravity_vectors_madg_mag;
    std::vector<std::vector<double>> gravity_vectors_madg;
    std::vector<std::vector<double>> gravity_vectors_gt;
    std::vector<std::vector<double>> gravity_vectors_comp_mag;
    std::vector<std::vector<double>> gravity_vectors_comp;

    // Initialize actors
    vtkSmartPointer<vtkActor> arrowActor_madg = getArrowActor({0, 0, 0});
    vtkSmartPointer<vtkActor> planeActor_madg = getPlaneActor({0, 0, 0});

    vtkSmartPointer<vtkActor> arrowActor_comp = getArrowActor({0, 0, 0});
    vtkSmartPointer<vtkActor> planeActor_comp = getPlaneActor({0, 0, 0});

    vtkSmartPointer<vtkActor> arrowActor_madg_mag = getArrowActor({0, 0, 0});
    vtkSmartPointer<vtkActor> planeActor_madg_mag = getPlaneActor({0, 0, 0});

    vtkSmartPointer<vtkActor> arrowActor_comp_mag = getArrowActor({0, 0, 0});
    vtkSmartPointer<vtkActor> planeActor_comp_mag = getPlaneActor({0, 0, 0});

    vtkSmartPointer<vtkActor> arrowActor_gt = getArrowActor({0, 0, 0});
    vtkSmartPointer<vtkActor> planeActor_gt = getPlaneActor({0, 0, 0});

    vtkSmartPointer<vtkActorCollection> actors_madg = vtkSmartPointer<vtkActorCollection>::New();
    vtkSmartPointer<vtkActorCollection> actors_comp = vtkSmartPointer<vtkActorCollection>::New();
    vtkSmartPointer<vtkActorCollection> actors_madg_mag = vtkSmartPointer<vtkActorCollection>::New();
    vtkSmartPointer<vtkActorCollection> actors_comp_mag = vtkSmartPointer<vtkActorCollection>::New();
    vtkSmartPointer<vtkActorCollection> actors_gt = vtkSmartPointer<vtkActorCollection>::New();

    actors_madg->AddItem(arrowActor_madg);
    actors_madg->AddItem(planeActor_madg);

    actors_gt->AddItem(arrowActor_gt);
    actors_gt->AddItem(planeActor_gt);

    actors_comp->AddItem(arrowActor_comp);
    actors_comp->AddItem(planeActor_comp);

    actors_madg_mag->AddItem(arrowActor_madg_mag);
    actors_madg_mag->AddItem(planeActor_madg_mag);

    actors_comp_mag->AddItem(arrowActor_comp_mag);
    actors_comp_mag->AddItem(planeActor_comp_mag);

    // AXES
    auto transformA = vtkSmartPointer<vtkTransform>::New();
    transformA->Translate(0.0, 0.0, 0.0);

    auto axes = vtkSmartPointer<vtkAxesActor>::New();

    // The axes are positioned with a user transform
    axes->SetUserTransform(transformA);

    // Add actors to vtkViewer instances
    vtk_viewer_madg.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
    vtk_viewer_madg.addActors(actors_madg);
    vtk_viewer_madg.addActor(axes);

    vtk_viewer_comp.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
    vtk_viewer_comp.addActors(actors_comp);
    vtk_viewer_comp.addActor(axes);

    vtk_viewer_gt.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
    vtk_viewer_gt.addActors(actors_gt);
    vtk_viewer_gt.addActor(axes);

    vtk_viewer_madg_mag.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
    vtk_viewer_madg_mag.addActors(actors_madg_mag);
    vtk_viewer_madg_mag.addActor(axes);

    vtk_viewer_comp_mag.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
    vtk_viewer_comp_mag.addActors(actors_comp_mag);
    vtk_viewer_comp_mag.addActor(axes);

    // ---- OpenGL stuff ----

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
    ImGuiStyle* style = &ImGui::GetStyle();
    style->WindowRounding = 4;
    style->FrameRounding = 4;
    style->GrabRounding = 4;

    // Setup Platform/renderer_madg backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Initial UI state variables set
    static bool loop = false;
    static bool black_background = false;

    static bool vtk_madg_open = false;
    static bool vtk_gt_open = false;
    static bool vtk_comp_open = false;
    static bool vtk_madg_mag_open = false;
    static bool vtk_comp_mag_open = false;

    static bool is_playing = false;
    static bool is_manual_playback = true;

    static bool is_calculated = false;
    static int vector_index = 0;
    static bool min_index = true;
    static bool max_index = true;
    static bool save_to_file = false;

    // static bool show_style_editor = false;
    // static bool show_demo_window = false;
    std::vector<Vec3> rmse;

    // Main window loop
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);

        // if(ImGui::BeginMainMenuBar())
        // {
        //     if(ImGui::BeginMenu("Windows")) 
        //     {
        //         ImGui::MenuItem("Style editor", 0, &show_style_editor);
        //         ImGui::MenuItem("Demo Window", 0, &show_demo_window);
        //         ImGui::EndMenu();
        //     }
        //     ImGui::EndMainMenuBar();
        // }

        // if (show_demo_window) ImGui::ShowDemoWindow();
		// if (show_style_editor) ImGui::ShowStyleEditor();

        {            
            ImGui::Begin("Menu");

            if(ImGui::Button("Select Folder"))
            {
                selectedfolderPath = NULL;
                fPath = "";

                // If the dialog is open and we try to open it again, stop the program until it is closed
                selectedfolderPath = tinyfd_selectFolderDialog("Select Folder with data files in it", "");
                if (!selectedfolderPath)
                {
                    // Return to UI
                }
                else
                {
                    fPath = selectedfolderPath;
                }
            }

            ImGui::Text("%s", fPath.c_str());

            static int start_index = 0;
            ImGui::Checkbox(_labelPrefix("From first:").c_str(), &min_index);
            if(!min_index) 
            {
                ImGui::InputInt(_labelPrefix("  Range from:").c_str(), &start_index); 
            }

            static int end_index = 45000;
            ImGui::Checkbox(_labelPrefix("To last:").c_str(), &max_index);
            if(!max_index)
            { 
                ImGui::InputInt(_labelPrefix("  Range to:").c_str(), &end_index); 
            }            

            static float freq = 286.0;
            ImGui::InputFloat(_labelPrefix("Sample frequency:").c_str(), &freq, 0.01f, 1.0f, "%.3f");

            static float comp_gain = 0.2f;
            ImGui::InputFloat(_labelPrefix("Comp. filter gain:").c_str(), &comp_gain, 0.01f, 1.0f, "%.3f");

            static float madg_beta = 0.2f;
            ImGui::InputFloat(_labelPrefix("Madg. filter beta").c_str(), &madg_beta, 0.01f, 1.0f, "%.3f");

            ImGui::Checkbox(_labelPrefix("Save to file").c_str(), &save_to_file);

            if(ImGui::Button("Calculate"))
            {
                if(fPath.length() != 0)
                {
                    gravity_vectors_madg.clear();
                    gravity_vectors_madg_mag.clear();
                    gravity_vectors_gt.clear();
                    gravity_vectors_comp.clear();
                    gravity_vectors_comp_mag.clear();

                    if(min_index)
                    {
                        start_index = 0;
                    }
                
                    std::string results_suffix = runFilter(freq, comp_gain, madg_beta, start_index, end_index, max_index, gravity_vectors_madg_mag, gravity_vectors_madg, gravity_vectors_gt, gravity_vectors_comp_mag, gravity_vectors_comp);

                    vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(0));
                    vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(0));
                    vtk_viewer_comp.updateActors(actors_comp, gravity_vectors_comp.at(0));
                    vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(0));
                    vtk_viewer_comp_mag.updateActors(actors_comp_mag, gravity_vectors_comp_mag.at(0));

                    rmse = CsvReader::getRMSE(fPath, results_suffix);

                    if(!save_to_file)
                    {
                        std::cout << fPath + results_suffix << std::endl;
                        std::filesystem::remove_all(fPath + results_suffix);
                    }
                    
                    is_calculated = true;
                    vector_index = 0;
                }
                else
                {
                    ImGui::OpenPopup("No data, no problem...?");
                }
            }
            ImGui::Text("");

            // Always center this window when appearing
            ImVec2 center = ImGui::GetMainViewport()->GetCenter();
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            if(ImGui::BeginPopupModal("No data, no problem...?", NULL, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::Text("Please select a folder containing data files!\n\n");
                ImGui::Separator();

                if (ImGui::Button("OK", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }
                ImGui::SetItemDefaultFocus();
                ImGui::EndPopup();
            }

            if(is_calculated)
            {
                auto renderer_madg = vtk_viewer_madg.getRenderer();                
                auto renderer_comp = vtk_viewer_comp.getRenderer();
                auto renderer_gt = vtk_viewer_gt.getRenderer();
                auto renderer_madg_mag = vtk_viewer_madg_mag.getRenderer();                
                auto renderer_comp_mag = vtk_viewer_comp_mag.getRenderer();

                ImGui::Checkbox(_labelPrefix("Black background:").c_str(), &black_background);
                if(black_background)
                {
                    // Set background to black
                    renderer_madg->SetBackground(0, 0, 0);
                    renderer_gt->SetBackground(0, 0, 0);
                    renderer_comp->SetBackground(0, 0, 0);
                    renderer_madg_mag->SetBackground(0, 0, 0);
                    renderer_comp_mag->SetBackground(0, 0, 0);
                }

                if(!black_background)
                {
                    renderer_madg->SetBackground(colors->GetColor3d("BkgColor").GetData());
                    renderer_gt->SetBackground(colors->GetColor3d("BkgColor").GetData());
                    renderer_comp->SetBackground(colors->GetColor3d("BkgColor").GetData());
                    renderer_madg_mag->SetBackground(colors->GetColor3d("BkgColor").GetData());
                    renderer_comp_mag->SetBackground(colors->GetColor3d("BkgColor").GetData());
                }
            
                static float vtk2BkgAlpha = 0.2f;
                ImGui::SliderFloat(_labelPrefix("Background alpha:").c_str(), &vtk2BkgAlpha, 0.0f, 1.0f);
                renderer_madg->SetBackgroundAlpha(vtk2BkgAlpha);
                renderer_gt->SetBackgroundAlpha(vtk2BkgAlpha);
                renderer_comp->SetBackgroundAlpha(vtk2BkgAlpha);
                renderer_madg_mag->SetBackgroundAlpha(vtk2BkgAlpha);
                renderer_comp_mag->SetBackgroundAlpha(vtk2BkgAlpha);

                if(is_manual_playback)
                {
                    if(ImGui::SliderInt(_labelPrefix("Vector index:").c_str(), &vector_index, 0, gravity_vectors_madg.size()-1))
                    {
                        vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(vector_index));
                        vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(vector_index));
                        vtk_viewer_comp.updateActors(actors_comp, gravity_vectors_comp.at(vector_index));
                        vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                        vtk_viewer_comp_mag.updateActors(actors_comp_mag, gravity_vectors_comp_mag.at(vector_index));
                    }

                    ImGui::Text("");
                    float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
                    ImGui::SameLine(ImGui::GetCursorPosX()+ImGui::CalcItemWidth()*0.95f+spacing);
                    ImGui::PushButtonRepeat(true);
                    if(ImGui::ArrowButton("##left", ImGuiDir_Left)) 
                    { 
                        if(vector_index < 1)
                        {
                            vector_index = gravity_vectors_madg.size()-1;
                            vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(vector_index));
                            vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(vector_index));
                            vtk_viewer_comp.updateActors(actors_comp, gravity_vectors_comp.at(vector_index));
                            vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                            vtk_viewer_comp_mag.updateActors(actors_comp_mag, gravity_vectors_comp_mag.at(vector_index));
                        }
                        else
                        {
                            vector_index--;
                            vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(vector_index));
                            vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(vector_index));
                            vtk_viewer_comp.updateActors(actors_comp, gravity_vectors_comp.at(vector_index));
                            vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                            vtk_viewer_comp_mag.updateActors(actors_comp_mag, gravity_vectors_comp_mag.at(vector_index));
                        } 
                    }
                    ImGui::SameLine(0.0f, spacing);
                    if(ImGui::ArrowButton("##right", ImGuiDir_Right))
                    { 
                        if(vector_index >= gravity_vectors_madg.size()-1) 
                        {
                            vector_index = 0;
                            vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(vector_index));
                            vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(vector_index));
                            vtk_viewer_comp.updateActors(actors_comp, gravity_vectors_comp.at(vector_index));
                            vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                            vtk_viewer_comp_mag.updateActors(actors_comp_mag, gravity_vectors_comp_mag.at(vector_index));
                        }
                        else
                        {
                            vector_index++;
                            vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(vector_index));
                            vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(vector_index));
                            vtk_viewer_comp.updateActors(actors_comp, gravity_vectors_comp.at(vector_index));
                            vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                            vtk_viewer_comp_mag.updateActors(actors_comp_mag, gravity_vectors_comp_mag.at(vector_index));
                        }
                    }
                    ImGui::PopButtonRepeat();

                    ImGui::Text("");
                    ImGui::SameLine(ImGui::GetCursorPosX()+ImGui::CalcItemWidth()*0.95f+spacing);
                    if(is_playing)
                    {
                        if(ImGui::Button("Stop")) 
                        {
                            is_playing = false;
                        }
                    }
                    else if(ImGui::Button("Play"))
                    {
                        is_playing = true;
                    }

                    ImGui::Checkbox(_labelPrefix("Loop:").c_str(), &loop);
                }

                if(is_playing)
                {
                    if(vector_index >= gravity_vectors_madg.size()-1) 
                    {
                        if(!loop)
                        {
                            is_playing = false;
                        }

                        vector_index = 0;
                        vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(vector_index));
                        vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(vector_index));
                        vtk_viewer_comp.updateActors(actors_comp, gravity_vectors_comp.at(vector_index));
                        vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                        vtk_viewer_comp_mag.updateActors(actors_comp_mag, gravity_vectors_comp_mag.at(vector_index));
                    }
                    vector_index++;
                    vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(vector_index));
                    vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(vector_index));
                    vtk_viewer_comp.updateActors(actors_comp, gravity_vectors_comp.at(vector_index));
                    vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                    vtk_viewer_comp_mag.updateActors(actors_comp_mag, gravity_vectors_comp_mag.at(vector_index));
                }

                ImGui::Text("");
                ImGui::Text("Show estimated vector visualization windows:");
                ImGui::Checkbox(_labelPrefix("Ground truth:").c_str(), &vtk_gt_open);
                ImGui::Checkbox(_labelPrefix("Madg. filter w/ mag.:").c_str(), &vtk_madg_mag_open);
                ImGui::Checkbox(_labelPrefix("Madg. filter:").c_str(), &vtk_madg_open);
                ImGui::Checkbox(_labelPrefix("Comp. filter w/ mag.:").c_str(), &vtk_comp_mag_open);
                ImGui::Checkbox(_labelPrefix("Comp. filter:").c_str(), &vtk_comp_open);
                ImGui::Text("");

                ImGui::Text("RMSE Values (degrees): ");
                if(ImGui::BeginTable("table1", 5))
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("Madg. w/ mag.");
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("Madg.");
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("Comp. w/ mag.");
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
            }

            ImGui::Text("\nApplication average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

            ImGui::End();
        }

        if(vtk_madg_open)
        {
            ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
            ImGui::Begin("Madgwick Filter estimation", &vtk_madg_open, VtkViewer::NoScrollFlags());
            ImGui::Text("Euler angles (degrees): ");
            if(ImGui::BeginTable("table2", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", gravity_vectors_madg.at(vector_index).at(0)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", gravity_vectors_madg.at(vector_index).at(1)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", gravity_vectors_madg.at(vector_index).at(2)*180/M_PI);
                ImGui::EndTable();
            }
            ImGui::Text("Euler angle errors (degrees): ");
            if(ImGui::BeginTable("table5", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", (gravity_vectors_madg.at(vector_index).at(0)-gravity_vectors_gt.at(vector_index).at(0))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", (gravity_vectors_madg.at(vector_index).at(1)-gravity_vectors_gt.at(vector_index).at(1))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", (gravity_vectors_madg.at(vector_index).at(2)-gravity_vectors_gt.at(vector_index).at(2))*180/M_PI);
                ImGui::EndTable();
            }
            vtk_viewer_madg.render();
            ImGui::End();
        }

        if(vtk_madg_mag_open)
        {
            ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
            ImGui::Begin("Madgwick Filter w/ mag. estimation", &vtk_madg_mag_open, VtkViewer::NoScrollFlags());
            ImGui::Text("Euler angles (degrees): ");
            if(ImGui::BeginTable("table2", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", gravity_vectors_madg_mag.at(vector_index).at(0)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", gravity_vectors_madg_mag.at(vector_index).at(1)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", gravity_vectors_madg_mag.at(vector_index).at(2)*180/M_PI);
                ImGui::EndTable();
            }
            ImGui::Text("Euler angle errors (degrees): ");
            if(ImGui::BeginTable("table5", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", (gravity_vectors_madg_mag.at(vector_index).at(0)-gravity_vectors_gt.at(vector_index).at(0))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", (gravity_vectors_madg_mag.at(vector_index).at(1)-gravity_vectors_gt.at(vector_index).at(1))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", (gravity_vectors_madg_mag.at(vector_index).at(2)-gravity_vectors_gt.at(vector_index).at(2))*180/M_PI);
                ImGui::EndTable();
            }
            vtk_viewer_madg_mag.render();
            ImGui::End();
        }

        if(vtk_gt_open)
        {
            ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
            ImGui::Begin("Ground truth", &vtk_gt_open, VtkViewer::NoScrollFlags());
            ImGui::Text("Euler angles (degrees): ");
            if(ImGui::BeginTable("table3", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", gravity_vectors_gt.at(vector_index).at(0)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", gravity_vectors_gt.at(vector_index).at(1)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", gravity_vectors_gt.at(vector_index).at(2)*180/M_PI);
                ImGui::EndTable();
            }
            vtk_viewer_gt.render();
            ImGui::End();
        }

        if(vtk_comp_open)
        {
            ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
            ImGui::Begin("Complementary filter estimation", &vtk_comp_open, VtkViewer::NoScrollFlags());
            ImGui::Text("Euler angles (degrees): ");
            if(ImGui::BeginTable("table4", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", gravity_vectors_comp.at(vector_index).at(0)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", gravity_vectors_comp.at(vector_index).at(1)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", gravity_vectors_comp.at(vector_index).at(2)*180/M_PI);
                ImGui::EndTable();
            }
            ImGui::Text("Euler angle errors (degrees): ");
            if(ImGui::BeginTable("table6", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", (gravity_vectors_comp.at(vector_index).at(0)-gravity_vectors_gt.at(vector_index).at(0))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", (gravity_vectors_comp.at(vector_index).at(1)-gravity_vectors_gt.at(vector_index).at(1))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", (gravity_vectors_comp.at(vector_index).at(2)-gravity_vectors_gt.at(vector_index).at(2))*180/M_PI);
                ImGui::EndTable();
            }
            vtk_viewer_comp.render();
            ImGui::End();
        }

        if(vtk_comp_mag_open)
        {
            ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
            ImGui::Begin("Complementary filter w/ mag. estimation", &vtk_comp_mag_open, VtkViewer::NoScrollFlags());
            ImGui::Text("Euler angles (degrees): ");
            if(ImGui::BeginTable("table4", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", gravity_vectors_comp_mag.at(vector_index).at(0)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", gravity_vectors_comp_mag.at(vector_index).at(1)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", gravity_vectors_comp_mag.at(vector_index).at(2)*180/M_PI);
                ImGui::EndTable();
            }
            ImGui::Text("Euler angle errors (degrees): ");
            if(ImGui::BeginTable("table6", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", (gravity_vectors_comp_mag.at(vector_index).at(0)-gravity_vectors_gt.at(vector_index).at(0))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", (gravity_vectors_comp_mag.at(vector_index).at(1)-gravity_vectors_gt.at(vector_index).at(1))*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", (gravity_vectors_comp_mag.at(vector_index).at(2)-gravity_vectors_gt.at(vector_index).at(2))*180/M_PI);
                ImGui::EndTable();
            }
            vtk_viewer_comp_mag.render();
            ImGui::End();
        }

        ImGui::Render();

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Update and Render additional Platform Windows
        if(io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
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