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


int main () { 
    Mirror mir;

    AreaPick inputareapick;
    Pointspick inputpointspick;
    
    string inputfilename = "/home/yons/File/Mirror_PointCloud_Tool/data/yxz_cut.ply";

    inputpointspick.loadInputcloud(inputfilename);
    inputpointspick.simpleViewer(inputfilename);
    
    inputareapick.loadInputcloud(inputfilename);
    inputareapick.simpleViewer(inputfilename);

    string mirror_inputfilename=inputfilename.substr(0,inputfilename.find_last_of('.')).append("_picked.ply");
    mir.get_mirrorpointcloud(mirror_inputfilename);
    mir.view_mirror();
    return 0;
}
