#pragma once 
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include "areapick.h"
#include <vector>
#include <fstream>
#include <memory.h>




typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct Coord
{
    double x;
    double y;
    double z;
};
struct Coefficient
{
    double a;
    double b;
    double c;
    double d;
};



class Mirror
{
public:
    Mirror();
    ~Mirror(){};
    Coefficient get_panal(struct Coord pp[3]);
    double dis_pt2panel(Coord pt,Coefficient C);   
    Coord get_mirrorpt(Coord pt,Coefficient C);
    void get_mirrorpointcloud(string inputcloudfilename);
    void loadInputcloud(string inputcloudfile,PointCloudT::Ptr cloud);
    void stl_ply(string stl_path,string ply_path);
    int find_nexttolast(const string filepath);
    void view_mirror();
    void view_comparation();
private:
    PointCloudT::Ptr mirror_cloud;
    PointCloudT::Ptr ori_cloud;
    PointCloudT::Ptr AbnormalPart_Cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    string WindowName;
    const int screen_width=2560;
	const int screen_height=1080;
};