#include "gui.hpp"

namespace GUI 
{
    char const* selectedfolderPath;
    std::string fPath;

    static std::thread t;
    static std::thread _t;

    std::atomic<bool> is_calculated{false};
    static bool loading = false;

    // Init vtkViewers
    VtkViewer vtk_viewer_madg_mag;
    VtkViewer vtk_viewer_gt;
    VtkViewer vtk_viewer_naive_mag;
    VtkViewer vtk_viewer_madg;
    VtkViewer vtk_viewer_naive;

    std::vector<std::vector<double>> gravity_vectors_madg_mag;
    std::vector<std::vector<double>> gravity_vectors_madg;
    std::vector<std::vector<double>> gravity_vectors_gt;
    std::vector<std::vector<double>> gravity_vectors_naive_mag;
    std::vector<std::vector<double>> gravity_vectors_naive;

    vtkSmartPointer<vtkActor> arrowActor_madg;
    vtkSmartPointer<vtkActor> planeActor_madg;

    vtkSmartPointer<vtkActor> arrowActor_naive;
    vtkSmartPointer<vtkActor> planeActor_naive;

    vtkSmartPointer<vtkActor> arrowActor_madg_mag;
    vtkSmartPointer<vtkActor> planeActor_madg_mag;

    vtkSmartPointer<vtkActor> arrowActor_naive_mag;
    vtkSmartPointer<vtkActor> planeActor_naive_mag;

    vtkSmartPointer<vtkActor> arrowActor_gt;
    vtkSmartPointer<vtkActor> planeActor_gt;

    vtkSmartPointer<vtkActorCollection> actors_madg;
    vtkSmartPointer<vtkActorCollection> actors_naive;
    vtkSmartPointer<vtkActorCollection> actors_madg_mag;
    vtkSmartPointer<vtkActorCollection> actors_naive_mag;
    vtkSmartPointer<vtkActorCollection> actors_gt;

    vtkNew<vtkNamedColors> colors;

    std::vector<Vec3> rmse;

    // Initial UI state variables set
    static bool loop = false;
    static bool black_background = false;

    static float naive_gain = 0.003f;
    static float madg_beta = 0.18f;
    static float madg_zeta = 0.0001f;
    static float freq = 286.0;
    static int end_index = 45000;
    static int start_index = 0;

    static bool vtk_madg_open = false;
    static bool vtk_gt_open = true;
    static bool vtk_naive_open = false;
    static bool vtk_madg_mag_open = true;
    static bool vtk_naive_mag_open = true;

    static bool is_playing = false;
    static bool is_manual_playback = true;

    static int vector_index = 0;
    static bool min_index = true;
    static bool max_index = true;
    static bool save_to_file = false;
    static bool has_ground_truth = false;
    static bool has_mag = false;

    // static bool show_style_editor = false;
    // static bool show_demo_window = false;

    // tinyfiledialogs
    void* call_from_thread()
    {
        selectedfolderPath = tinyfd_selectFolderDialog("Select Folder with data files in it", "");

        // If the dialog is open and we try to open it again, stop the program until it is closed
        if (!selectedfolderPath)
        {
            // Return to UI
            return NULL;
        }
        else
        {
            fPath = selectedfolderPath;
        }
        return NULL;
    }

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

    void* runFilter()
    {
        float deltat = 1.0f/(float)freq;

        has_ground_truth = std::filesystem::exists(fPath + "opt_quat.csv");
        has_mag = std::filesystem::exists(fPath + "imu_mag.csv");

        // Input
        NaiveFilter naive = NaiveFilter(deltat, naive_gain);
        NaiveFilter naive_mag = NaiveFilter(deltat, naive_gain);
        MadgwickFilter madg = MadgwickFilter(deltat, madg_beta, madg_zeta);
        MadgwickFilter madg_mag = MadgwickFilter(deltat, madg_beta, madg_zeta);

        std::ofstream est_madg_mag;
        std::ofstream est_madg_no_mag;
        std::ofstream est_naive_mag;
        std::ofstream est_naive_no_mag;

        std::ofstream error_madg_mag;
        std::ofstream error_madg_no_mag;
        std::ofstream error_naive_mag;
        std::ofstream error_naive_no_mag;

        std::vector<Vec3> a = CsvReader::getVec3Data(fPath + "imu_acc.csv");
        std::vector<Vec3> w = CsvReader::getVec3Data(fPath + "imu_gyr.csv");
        std::vector<Vec3> m;
        std::vector<int> mov;
        std::vector<Quaternion> gt;

        if(has_mag)
        {
            m = CsvReader::getVec3Data(fPath + "imu_mag.csv");
        }
        else
        {
            vtk_naive_mag_open = false;
            vtk_naive_open = true;
            vtk_madg_mag_open = false;
            vtk_madg_open = true;
            for(int i = 0; i < a.size(); ++i)
            {
                m.push_back(Vec3(0,0,0));
            }
        }

        if(max_index)
        {
            end_index = a.size();
        }

        Quaternion initial_state;
        if(start_index < 1)
        {
            start_index = 0;
            initial_state = has_mag ? Quaternion::getOrientationFromAccMag(a.at(0), m.at(0)) : Quaternion::getOrientationFromAcc(a.at(0));        
        }
        else
        {
            initial_state = has_mag ? Quaternion::getOrientationFromAccMag(a.at(start_index-1), m.at(start_index-1)) : Quaternion::getOrientationFromAcc(a.at(start_index-1));
        }

        std::string results_suffix = "_" +std::to_string(freq) + "_" + std::to_string(naive_gain) + "_" + std::to_string(madg_beta) + "_" + std::to_string(start_index) + "_" + std::to_string(end_index);

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
        est_naive_mag.open(fPath + results_suffix + "/est_naive_mag.csv");
        est_naive_no_mag.open(fPath + results_suffix + "/est_naive_no_mag.csv");

        if(has_ground_truth)
        {
            gt = CsvReader::getQuatData(fPath + "opt_quat.csv");
            mov = CsvReader::get1DData(fPath + "movement.csv");
            error_madg_mag.open(fPath + results_suffix + "/error_madg_mag.csv");
            error_madg_no_mag.open(fPath + results_suffix + "/error_madg_no_mag.csv");
            error_naive_mag.open(fPath + results_suffix + "/error_naive_mag.csv");
            error_naive_no_mag.open(fPath + results_suffix + "/error_naive_no_mag.csv");
        }
        else
        {
            vtk_gt_open = false;

            // fill vectors with zeroes
            for(int i = 0; i < a.size(); ++i)
            {
                gt.push_back(Quaternion(0,0,0,0));
                gravity_vectors_gt.push_back({0,0,0});
            }
        }

        naive_mag.setInitialState(initial_state);
        madg_mag.setInitialState(initial_state);
        naive.setInitialState(initial_state);
        madg.setInitialState(initial_state);

        Quaternion enu_madg_mag;
        Quaternion enu_madg;
        Quaternion enu_naive_mag;
        Quaternion enu_naive;

        for (int i = start_index; i < end_index; ++i)
        {
            naive_mag.updateFilter(w.at(i), a.at(i), m.at(i));
            naive.updateFilter(w.at(i), a.at(i));

            madg_mag.updateMARGFilter(w.at(i), a.at(i), m.at(i));        
            madg.updateIMUFilter(w.at(i), a.at(i));

            // Convert to ENU from NED
            enu_madg_mag = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg_mag.q);
            enu_madg = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), madg.q);
            enu_naive_mag = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), naive_mag.q);
            enu_naive = Metrics::hamiltonProduct(Quaternion(1.0/sqrt(2), 0.0, 0.0, 1.0/sqrt(2)), naive.q);

            // est_madg_mag << madg_mag.q.q_1 << "," << madg_mag.q.q_2 << "," << madg_mag.q.q_3 << "," << madg_mag.q.q_4 << "\n";
            // est_naive_mag << naive_mag.q.q_1 << "," << naive_mag.q.q_2 << "," << naive_mag.q.q_3 << "," << naive_mag.q.q_4 << "\n";

            // est_madg_no_mag << madg.q.q_1 << "," << madg.q.q_2 << "," << madg.q.q_3 << "," << madg.q.q_4 << "\n";
            // est_naive_no_mag << comp.q.q_1 << "," << comp.q.q_2 << "," << comp.q.q_3 << "," << comp.q.q_4 << "\n";

            est_madg_mag << enu_madg_mag.q_1 << "," << enu_madg_mag.q_2 << "," << enu_madg_mag.q_3 << "," << enu_madg_mag.q_4 << "\n";
            est_naive_mag << enu_naive_mag.q_1 << "," << enu_naive_mag.q_2 << "," << enu_naive_mag.q_3 << "," << enu_naive_mag.q_4 << "\n";

            est_madg_no_mag << enu_madg.q_1 << "," << enu_madg.q_2 << "," << enu_madg.q_3 << "," << enu_madg.q_4 << "\n";
            est_naive_no_mag << enu_naive.q_1 << "," << enu_naive.q_2 << "," << enu_naive.q_3 << "," << enu_naive.q_4 << "\n";

            if(has_ground_truth)
            {
                // These are in ENU
                if((!gt.at(i).isNaN()) && (mov.at(i)==1))
                {
                    Quaternion err_quat_madg_mag = Metrics::error_quaternion_earth(gt.at(i), enu_madg_mag);
                    Quaternion err_quat_madg_no_mag = Metrics::error_quaternion_earth(gt.at(i), enu_madg);
                    Quaternion err_quat_naive_mag = Metrics::error_quaternion_earth(gt.at(i), enu_naive_mag);
                    Quaternion err_quat_naive_no_mag = Metrics::error_quaternion_earth(gt.at(i), enu_naive);

                    error_madg_mag << Metrics::total_error(err_quat_madg_mag) << "," << Metrics::inclination_error(err_quat_madg_mag) << "," << Metrics::heading_error(err_quat_madg_mag) << "\n";
                    error_madg_no_mag << Metrics::total_error(err_quat_madg_no_mag) << "," << Metrics::inclination_error(err_quat_madg_no_mag) << "," << Metrics::heading_error(err_quat_madg_no_mag) << "\n";
                    error_naive_mag << Metrics::total_error(err_quat_naive_mag) << "," << Metrics::inclination_error(err_quat_naive_mag) << "," << Metrics::heading_error(err_quat_naive_mag) << "\n";
                    error_naive_no_mag << Metrics::total_error(err_quat_naive_no_mag) << "," << Metrics::inclination_error(err_quat_naive_no_mag) << "," << Metrics::heading_error(err_quat_naive_no_mag) << "\n";
                }
                else
                {
                    error_madg_mag << 9999 << "," << 9999 << "," << 9999 << "\n";
                    error_madg_no_mag << 9999 << "," << 9999 << "," << 9999 << "\n";
                    error_naive_mag << 9999 << "," << 9999 << "," << 9999 << "\n";
                    error_naive_no_mag << 9999 << "," << 9999 << "," << 9999 << "\n";
                }
                gravity_vectors_gt.push_back({gt.at(i).roll(), gt.at(i).pitch(), gt.at(i).yaw()});
            }

            gravity_vectors_madg.push_back({enu_madg.roll(), enu_madg.pitch(), enu_madg.yaw()});
            gravity_vectors_madg_mag.push_back({enu_madg_mag.roll(), enu_madg_mag.pitch(), enu_madg_mag.yaw()});            
            gravity_vectors_naive.push_back({enu_naive.roll(), enu_naive.pitch(), enu_naive.yaw()});
            gravity_vectors_naive_mag.push_back({enu_naive_mag.roll(), enu_naive_mag.pitch(), enu_naive_mag.yaw()});
        }

        est_madg_mag.close();
        est_madg_no_mag.close();
        est_naive_mag.close();
        est_naive_no_mag.close();

        if(has_ground_truth)
        {
            error_madg_mag.close();
            error_madg_no_mag.close();
            error_naive_mag.close();
            error_naive_no_mag.close();

            vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(0));

            rmse = CsvReader::getRMSE(fPath, results_suffix);
        }

        vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(0));        
        vtk_viewer_naive.updateActors(actors_naive, gravity_vectors_naive.at(0));
        vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(0));
        vtk_viewer_naive_mag.updateActors(actors_naive_mag, gravity_vectors_naive_mag.at(0));        

        if(!save_to_file)
        {
            std::filesystem::remove_all(fPath + results_suffix);
        }
        
        is_calculated = true;
        loading = false;
        vector_index = 0;
        return NULL;
    }

    void initUI()
    {
        // Set background color
        std::array<unsigned char, 4> bkg{{26, 51, 77, 255}};
        colors->SetColor("BkgColor", bkg.data());

        // Initialize actors
        arrowActor_madg = getArrowActor({0, 0, 0});
        planeActor_madg = getPlaneActor({0, 0, 0});

        arrowActor_naive = getArrowActor({0, 0, 0});
        planeActor_naive = getPlaneActor({0, 0, 0});

        arrowActor_madg_mag = getArrowActor({0, 0, 0});
        planeActor_madg_mag = getPlaneActor({0, 0, 0});

        arrowActor_naive_mag = getArrowActor({0, 0, 0});
        planeActor_naive_mag = getPlaneActor({0, 0, 0});

        arrowActor_gt = getArrowActor({0, 0, 0});
        planeActor_gt = getPlaneActor({0, 0, 0});

        actors_madg = vtkSmartPointer<vtkActorCollection>::New();
        actors_naive = vtkSmartPointer<vtkActorCollection>::New();
        actors_madg_mag = vtkSmartPointer<vtkActorCollection>::New();
        actors_naive_mag = vtkSmartPointer<vtkActorCollection>::New();
        actors_gt = vtkSmartPointer<vtkActorCollection>::New();

        actors_madg->AddItem(arrowActor_madg);
        actors_madg->AddItem(planeActor_madg);

        actors_gt->AddItem(arrowActor_gt);
        actors_gt->AddItem(planeActor_gt);

        actors_naive->AddItem(arrowActor_naive);
        actors_naive->AddItem(planeActor_naive);

        actors_madg_mag->AddItem(arrowActor_madg_mag);
        actors_madg_mag->AddItem(planeActor_madg_mag);

        actors_naive_mag->AddItem(arrowActor_naive_mag);
        actors_naive_mag->AddItem(planeActor_naive_mag);

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

        vtk_viewer_naive.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
        vtk_viewer_naive.addActors(actors_naive);
        vtk_viewer_naive.addActor(axes);

        vtk_viewer_gt.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
        vtk_viewer_gt.addActors(actors_gt);
        vtk_viewer_gt.addActor(axes);

        vtk_viewer_madg_mag.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
        vtk_viewer_madg_mag.addActors(actors_madg_mag);
        vtk_viewer_madg_mag.addActor(axes);

        vtk_viewer_naive_mag.getRenderer()->SetBackground(colors->GetColor3d("BkgColor").GetData());
        vtk_viewer_naive_mag.addActors(actors_naive_mag);
        vtk_viewer_naive_mag.addActor(axes);
    }

    void generateUI()
    {
        {            
            ImGui::Begin("Menu");

            if(ImGui::Button("Select Folder"))
            {
                selectedfolderPath = NULL;
                fPath = "";

                // If the dialog is open and we try to open it again, stop the program until it is closed
				if(t.joinable())
				{
					t.join();
					t = std::thread(&call_from_thread);
				}
				else
				{
					t = std::thread(&call_from_thread);
				}
            }

            ImGui::Text("%s", fPath.c_str());

            
            ImGui::Checkbox(_labelPrefix("From first:").c_str(), &min_index);
            if(!min_index) 
            {
                ImGui::InputInt(_labelPrefix("  Range from:").c_str(), &start_index); 
            }

            ImGui::Checkbox(_labelPrefix("To last:").c_str(), &max_index);
            if(!max_index)
            { 
                ImGui::InputInt(_labelPrefix("  Range to:").c_str(), &end_index); 
            }            
    
            ImGui::InputFloat(_labelPrefix("Sample frequency:").c_str(), &freq, 0.01f, 1.0f, "%.3f");

            ImGui::InputFloat(_labelPrefix("Naive filter gain:").c_str(), &naive_gain, 0.001f, 0.01f, "%.5f");

            ImGui::InputFloat(_labelPrefix("Madg. filter beta:").c_str(), &madg_beta, 0.01f, 0.1f, "%.5f");

            ImGui::InputFloat(_labelPrefix("Madg. filter zeta:").c_str(), &madg_zeta, 0.0001f, 0.001f, "%.5f");

            ImGui::Checkbox(_labelPrefix("Save results to files: ").c_str(), &save_to_file);

            if(ImGui::Button("Calculate"))
            {     
                if(t.joinable())
				{
					t.join();
				}

                if(fPath.length() != 0)
                {
                    loading = true;
                    is_calculated = false;
                    vector_index = 0;
                    gravity_vectors_madg.clear();
                    gravity_vectors_madg_mag.clear();
                    gravity_vectors_gt.clear();
                    gravity_vectors_naive.clear();
                    gravity_vectors_naive_mag.clear();

                    if(min_index)
                    {
                        start_index = 0;
                    }

                    if(_t.joinable())
                    {
                        _t.join();
                        _t = std::thread(&runFilter);
                    }
                    else
                    {
                        _t = std::thread(&runFilter);
                    }
                }
                else
                {
                    ImGui::OpenPopup("No data, no problem...?");
                }
            }
            if(!is_calculated && loading)
            {
                ImGui::SameLine();
                ImGui::Text("Calculating... %c", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3]);
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
                if(_t.joinable())
                {
                    _t.join();
                }
                if(t.joinable())
                {
                    t.join();
                }

                auto renderer_madg = vtk_viewer_madg.getRenderer();                
                auto renderer_naive = vtk_viewer_naive.getRenderer();
                auto renderer_gt = vtk_viewer_gt.getRenderer();
                auto renderer_madg_mag = vtk_viewer_madg_mag.getRenderer();                
                auto renderer_naive_mag = vtk_viewer_naive_mag.getRenderer();

                ImGui::Checkbox(_labelPrefix("Black background:").c_str(), &black_background);
                if(black_background)
                {
                    // Set background to black
                    renderer_madg->SetBackground(0, 0, 0);
                    renderer_gt->SetBackground(0, 0, 0);
                    renderer_naive->SetBackground(0, 0, 0);
                    renderer_madg_mag->SetBackground(0, 0, 0);
                    renderer_naive_mag->SetBackground(0, 0, 0);
                }

                if(!black_background)
                {
                    renderer_madg->SetBackground(colors->GetColor3d("BkgColor").GetData());
                    renderer_gt->SetBackground(colors->GetColor3d("BkgColor").GetData());
                    renderer_naive->SetBackground(colors->GetColor3d("BkgColor").GetData());
                    renderer_madg_mag->SetBackground(colors->GetColor3d("BkgColor").GetData());
                    renderer_naive_mag->SetBackground(colors->GetColor3d("BkgColor").GetData());
                }
            
                static float vtk2BkgAlpha = 0.2f;
                ImGui::SliderFloat(_labelPrefix("Background alpha:").c_str(), &vtk2BkgAlpha, 0.0f, 1.0f);
                renderer_madg->SetBackgroundAlpha(vtk2BkgAlpha);
                renderer_gt->SetBackgroundAlpha(vtk2BkgAlpha);
                renderer_naive->SetBackgroundAlpha(vtk2BkgAlpha);
                renderer_madg_mag->SetBackgroundAlpha(vtk2BkgAlpha);
                renderer_naive_mag->SetBackgroundAlpha(vtk2BkgAlpha);

                if(is_manual_playback)
                {
                    if(ImGui::SliderInt(_labelPrefix("Vector index:").c_str(), &vector_index, 0, gravity_vectors_madg.size()-1))
                    {
                        vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(vector_index));
                        vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(vector_index));
                        vtk_viewer_naive.updateActors(actors_naive, gravity_vectors_naive.at(vector_index));
                        vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                        vtk_viewer_naive_mag.updateActors(actors_naive_mag, gravity_vectors_naive_mag.at(vector_index));
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
                            vtk_viewer_naive.updateActors(actors_naive, gravity_vectors_naive.at(vector_index));
                            vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                            vtk_viewer_naive_mag.updateActors(actors_naive_mag, gravity_vectors_naive_mag.at(vector_index));
                        }
                        else
                        {
                            vector_index--;
                            vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(vector_index));
                            vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(vector_index));
                            vtk_viewer_naive.updateActors(actors_naive, gravity_vectors_naive.at(vector_index));
                            vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                            vtk_viewer_naive_mag.updateActors(actors_naive_mag, gravity_vectors_naive_mag.at(vector_index));
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
                            vtk_viewer_naive.updateActors(actors_naive, gravity_vectors_naive.at(vector_index));
                            vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                            vtk_viewer_naive_mag.updateActors(actors_naive_mag, gravity_vectors_naive_mag.at(vector_index));
                        }
                        else
                        {
                            vector_index++;
                            vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(vector_index));
                            vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(vector_index));
                            vtk_viewer_naive.updateActors(actors_naive, gravity_vectors_naive.at(vector_index));
                            vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                            vtk_viewer_naive_mag.updateActors(actors_naive_mag, gravity_vectors_naive_mag.at(vector_index));
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
                        vtk_viewer_naive.updateActors(actors_naive, gravity_vectors_naive.at(vector_index));
                        vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                        vtk_viewer_naive_mag.updateActors(actors_naive_mag, gravity_vectors_naive_mag.at(vector_index));
                    }
                    vector_index++;
                    vtk_viewer_madg.updateActors(actors_madg, gravity_vectors_madg.at(vector_index));
                    vtk_viewer_gt.updateActors(actors_gt, gravity_vectors_gt.at(vector_index));
                    vtk_viewer_naive.updateActors(actors_naive, gravity_vectors_naive.at(vector_index));
                    vtk_viewer_madg_mag.updateActors(actors_madg_mag, gravity_vectors_madg_mag.at(vector_index));
                    vtk_viewer_naive_mag.updateActors(actors_naive_mag, gravity_vectors_naive_mag.at(vector_index));
                }

                ImGui::Text("");
                ImGui::Text("Show estimated vector visualization windows:");
                if(has_ground_truth)
                {
                    ImGui::Checkbox(_labelPrefix("Ground truth:").c_str(), &vtk_gt_open);
                }
                if(has_mag)
                {
                    ImGui::Checkbox(_labelPrefix("Madg. filter w/ mag.:").c_str(), &vtk_madg_mag_open);
                }
                ImGui::Checkbox(_labelPrefix("Madg. filter:").c_str(), &vtk_madg_open);
                if(has_mag)
                {
                    ImGui::Checkbox(_labelPrefix("Naive filter w/ mag.:").c_str(), &vtk_naive_mag_open);
                }
                ImGui::Checkbox(_labelPrefix("Naive filter:").c_str(), &vtk_naive_open);
                ImGui::Text("");

                if(has_ground_truth)
                {
                    ImGui::Text("RMSE Values (degrees): ");
                    if(ImGui::BeginTable("table1", 5))
                    {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("Madg. w/ mag.");
                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("Madg.");
                        ImGui::TableSetColumnIndex(3);
                        ImGui::Text("Naive w/ mag.");
                        ImGui::TableSetColumnIndex(4);
                        ImGui::Text("Naive");
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
                        ImGui::Text("Head. error");
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
            }

            ImGui::Text("\nApplication average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

            ImGui::End();
        }

        if(vtk_madg_open && is_calculated)
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
            if(has_ground_truth)
            {
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
            }
            vtk_viewer_madg.render();
            ImGui::End();
        }

        if(vtk_madg_mag_open && is_calculated)
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
            if(has_ground_truth)
            {
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
            }
            vtk_viewer_madg_mag.render();
            ImGui::End();
        }

        if(vtk_gt_open && is_calculated)
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

        if(vtk_naive_open && is_calculated)
        {
            ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
            ImGui::Begin("Naive complementary filter estimation", &vtk_naive_open, VtkViewer::NoScrollFlags());
            ImGui::Text("Euler angles (degrees): ");
            if(ImGui::BeginTable("table4", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", gravity_vectors_naive.at(vector_index).at(0)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", gravity_vectors_naive.at(vector_index).at(1)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", gravity_vectors_naive.at(vector_index).at(2)*180/M_PI);
                ImGui::EndTable();
            }
            if(has_ground_truth)
            {
                ImGui::Text("Euler angle errors (degrees): ");
                if(ImGui::BeginTable("table6", 3))
                {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::Text("Roll: %f", (gravity_vectors_naive.at(vector_index).at(0)-gravity_vectors_gt.at(vector_index).at(0))*180/M_PI);
                    ImGui::TableNextColumn();
                    ImGui::Text("Pitch: %f", (gravity_vectors_naive.at(vector_index).at(1)-gravity_vectors_gt.at(vector_index).at(1))*180/M_PI);
                    ImGui::TableNextColumn();
                    ImGui::Text("Yaw: %f", (gravity_vectors_naive.at(vector_index).at(2)-gravity_vectors_gt.at(vector_index).at(2))*180/M_PI);
                    ImGui::EndTable();
                }
            }
            vtk_viewer_naive.render();
            ImGui::End();
        }

        if(vtk_naive_mag_open && is_calculated)
        {
            ImGui::SetNextWindowSize(ImVec2(720, 480), ImGuiCond_FirstUseEver);
            ImGui::Begin("Naive complementary filter w/ mag. estimation", &vtk_naive_mag_open, VtkViewer::NoScrollFlags());
            ImGui::Text("Euler angles (degrees): ");
            if(ImGui::BeginTable("table4", 3))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Roll: %f", gravity_vectors_naive_mag.at(vector_index).at(0)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Pitch: %f", gravity_vectors_naive_mag.at(vector_index).at(1)*180/M_PI);
                ImGui::TableNextColumn();
                ImGui::Text("Yaw: %f", gravity_vectors_naive_mag.at(vector_index).at(2)*180/M_PI);
                ImGui::EndTable();
            }
            if(has_ground_truth)
            {
                ImGui::Text("Euler angle errors (degrees): ");
                if(ImGui::BeginTable("table6", 3))
                {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::Text("Roll: %f", (gravity_vectors_naive_mag.at(vector_index).at(0)-gravity_vectors_gt.at(vector_index).at(0))*180/M_PI);
                    ImGui::TableNextColumn();
                    ImGui::Text("Pitch: %f", (gravity_vectors_naive_mag.at(vector_index).at(1)-gravity_vectors_gt.at(vector_index).at(1))*180/M_PI);
                    ImGui::TableNextColumn();
                    ImGui::Text("Yaw: %f", (gravity_vectors_naive_mag.at(vector_index).at(2)-gravity_vectors_gt.at(vector_index).at(2))*180/M_PI);
                    ImGui::EndTable();
                }
            }
            vtk_viewer_naive_mag.render();
            ImGui::End();
        }
    }
}