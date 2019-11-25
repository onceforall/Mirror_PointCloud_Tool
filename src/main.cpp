/*
 Point Cloud Registration Tool
 
 BSD 2-Clause License
 Copyright (c) 2017, Dario Rethage
 See LICENSE at package root for full license
 */

#ifdef _OPENMP
#include <omp.h>
#endif

// Standard
#include <iostream>
#include <iterator>
#include <algorithm>
#include "areapick.h"
#include "mirror.hpp"

// GFLAGS
#include <gflags/gflags.h>
#ifdef _OPENMP
DEFINE_bool(no_parallel, false, "run single-threaded");
#endif


int main () 
{ 
    Mirror mir;

    AreaPick inputareapick;
    Pointspick inputpointspick;
    
    string inputfilename = "/home/yons/projects/Mirror_PointCloud_Tool/data/wzx_skull.ply";

    inputpointspick.loadInputcloud(inputfilename);
    inputpointspick.simpleViewer(inputfilename);
    
    inputareapick.loadInputcloud(inputfilename);
    inputareapick.simpleViewer(inputfilename);

   
    mir.get_mirrorpointcloud(inputfilename);
    //mir.view_mirror();
    mir.view_comparation();
    return 0;
}
