#pragma once

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "vtk_viewer.hpp"

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAxesActor.h>

#include "madgwick_filter.hpp"
#include "naive_filter.hpp"
#include "csv_reader.hpp"
#include "metrics.hpp"

#include "vtk_actor_generator.hpp"
#include "tinyfiledialogs.h"

namespace GUI
{
    void initUI();
    void generateUI();
    static std::string _labelPrefix(const char* const label);
	void* runFilter();
    void* call_from_thread();
}