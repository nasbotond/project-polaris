#pragma once

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <array>
#include <thread>

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

namespace GUI 
{
	void setup();
	void begin();
    void render();
    void initUI();
    void generateUI();
    void destroy();
    static void glfw_error_callback(int error, const char* description);
    static std::string _labelPrefix(const char* const label);
    void* runFilter(const int &freq, const float &comp_gain, const float &madg_beta, int &start_index, int &end_index, const bool &max_index, std::vector<std::vector<double>> &gravity_vectors_madg_mag,
    std::vector<std::vector<double>> &gravity_vectors_madg, std::vector<std::vector<double>> &gravity_vectors_gt, std::vector<std::vector<double>> &gravity_vectors_comp_mag, std::vector<std::vector<double>> &gravity_vectors_comp);
	void* call_from_thread();
	GLFWwindow* getCurrentWindow();
}