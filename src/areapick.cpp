#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/io/ply_io.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include "areapick.h"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>



typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int screen_width=2560;
int screen_height=1080;

/*
void closeviewer(const pcl::visualization::KeyboardEvent& event, void* obj) {
    
    Visualizer* vis = (Visualizer*) obj;
    if ((event.getKeySym () == "q" || event.getKeySym () == "Q") && event.keyDown ())
        vis->toggleShowSource();
    // if (event.getKeySym () == "s" && event.keyDown ())
    //     vis->toggleShowSource();
    
    // if (event.getKeySym () == "t" && event.keyDown ())
    //     vis->toggleShowTarget();
    
    // if (event.getKeySym () == "r" && event.keyDown ())
    //     vis->toggleShowRegistered();
    
    // if (event.getKeySym () == "k" && event.keyDown ())
    //     vis->toggleShowKeypoints();
    
    // if (event.getKeySym () == "Up" && event.keyDown ())
    //     vis->incrementResidualThreshold();
    
    // if (event.getKeySym () == "Down" && event.keyDown ())
    //     vis->decrementResidualThreshold();
    
}*/


AreaPick::AreaPick()
{
	inputcloud=PointCloudT::Ptr(new PointCloudT);
	clicked_points_3d=PointCloudT::Ptr (new PointCloudT);
	cloudName = "AreaPick";	
	num = 0;
	
}
void AreaPick::loadInputcloud(string inputcloudfile)
{
	string filetype = inputcloudfile.substr(inputcloudfile.find_last_of('.'), inputcloudfile.size());
	if (filetype == ".pcd")
	{
		if (pcl::io::loadPCDFile(inputcloudfile, *inputcloud) == -1)
		{
			cout << "Can't load pointcloud from " + inputcloudfile << endl;
			return ;
		}
	}
	else if (filetype == ".ply")
	{
		if (pcl::io::loadPLYFile(inputcloudfile, *inputcloud) == -1)
		{
			cout << "Can't load pointcloud from " + inputcloudfile << endl;
			return ;
		}
	}
	else if(filetype==".stl")
	{
		string ply_path=inputcloudfile.substr(0,inputcloudfile.find_last_of('.')).append(".ply");
		stl_ply(inputcloudfile, ply_path);
		if(pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path,*inputcloud)==-1)
		{
			std::cout<<"couldn't read file "+ply_path<<std::endl;
			return ;
		}
		std::cout<<"target cloud size: "<<inputcloud->size()<<std::endl;
	}
	return ;
}

void AreaPick::stl_ply(string stl_path,string ply_path)
{
	struct Coordinate {
		float x, y, z;
		bool operator<(const Coordinate& rhs) {
			return x<rhs.x || (x == rhs.x&&y<rhs.y) || (x == rhs.x&&y == rhs.y&&z<rhs.z);
		}
		bool operator==(const Coordinate& rhs) {
			return x == rhs.x&&y == rhs.y && z == rhs.z;
		}
	};
	vector<Coordinate> vecSorted, vecOrigin;
	vector<Coordinate>::iterator iter, iterBegin;

	int numberOfFacets;
	int numberOfPoints;
	int index;
	char c1[] = "ply\nformat binary_little_endian 1.0\ncomment By ET \nelement vertex ";
	char c2[] = "\nproperty float x\nproperty float y\nproperty float z\nelement face ";
	char c3[] = "\nproperty list uchar int vertex_indices\nend_header\n";
	clock_t start, finish;
	double totaltime;
	start = clock();

	int length;
	int position = 80;
	fstream fileIn(stl_path, ios::in | ios::binary);
	fileIn.seekg(0, ios::end);
	length = (int)fileIn.tellg();
	fileIn.seekg(0, ios::beg);
	char* buffer = new char[length];
	fileIn.read(buffer, length);
	fileIn.close();

	numberOfFacets = *(int*)&(buffer[position]);
	position += 4;
	cout << "Number of Facets: " << numberOfFacets << endl;
	for (int i = 0; i < numberOfFacets; i++)
	{
		Coordinate tmpC;
		position += 12;
		for (int j = 0; j < 3; j++)
		{
			tmpC.x = *(float*)&(buffer[position]);
			position += 4;
			tmpC.y = *(float*)&(buffer[position]);
			position += 4;
			tmpC.z = *(float*)&(buffer[position]);
			position += 4;
			vecOrigin.push_back(tmpC);
		}
		position += 2;
	}

	free(buffer);

	vecSorted = vecOrigin;
	sort(vecSorted.begin(), vecSorted.end());
	iter = unique(vecSorted.begin(), vecSorted.end());
	vecSorted.erase(iter, vecSorted.end());
	numberOfPoints = vecSorted.size();

	ofstream fileOut(ply_path, ios::binary | ios::out | ios::trunc);

	fileOut.write(c1, sizeof(c1) - 1);
	fileOut << numberOfPoints;
	fileOut.write(c2, sizeof(c2) - 1);
	fileOut << numberOfFacets;
	fileOut.write(c3, sizeof(c3) - 1);


	buffer = new char[numberOfPoints * 3 * 4];
	position = 0;
	for (int i = 0; i < numberOfPoints; i++)
	{
		buffer[position++] = *(char*)(&vecSorted[i].x);
		buffer[position++] = *((char*)(&vecSorted[i].x) + 1);
		buffer[position++] = *((char*)(&vecSorted[i].x) + 2);
		buffer[position++] = *((char*)(&vecSorted[i].x) + 3);
		buffer[position++] = *(char*)(&vecSorted[i].y);
		buffer[position++] = *((char*)(&vecSorted[i].y) + 1);
		buffer[position++] = *((char*)(&vecSorted[i].y) + 2);
		buffer[position++] = *((char*)(&vecSorted[i].y) + 3);
		buffer[position++] = *(char*)(&vecSorted[i].z);
		buffer[position++] = *((char*)(&vecSorted[i].z) + 1);
		buffer[position++] = *((char*)(&vecSorted[i].z) + 2);
		buffer[position++] = *((char*)(&vecSorted[i].z) + 3);
	}


	fileOut.write(buffer, numberOfPoints * 3 * 4);

	free(buffer);

	buffer = new char[numberOfFacets * 13];

	for (int i = 0; i < numberOfFacets; i++)
	{
		buffer[13 * i] = (unsigned char)3;
	}

	iterBegin = vecSorted.begin();
	position = 0;
	for (int i = 0; i < numberOfFacets; i++)
	{
		position++;
		for (int j = 0; j < 3; j++)
		{
			iter = lower_bound(vecSorted.begin(), vecSorted.end(), vecOrigin[3 * i + j]);
			index = iter - iterBegin;
			buffer[position++] = *(char*)(&index);
			buffer[position++] = *((char*)(&index) + 1);
			buffer[position++] = *((char*)(&index) + 2);
			buffer[position++] = *((char*)(&index) + 3);

		}
	}


	fileOut.write(buffer, numberOfFacets * 13);
	free(buffer);
	fileOut.close();


	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC * 1000;
	cout << "All Time: " << totaltime << "ms\n";
	return;
}

void  AreaPick::simpleViewer(string inputcloudfile)
{
	//visualizer
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(cloudName));
	viewer->addPointCloud(inputcloud, cloudName);
	viewer->resetCameraViewpoint(cloudName);
	viewer->registerAreaPickingCallback(&AreaPick::pp_callback, *this);
	viewer->registerKeyboardCallback(&AreaPick::closeviewer,*this);
	//viewer->setFullScreen(true); // Visualiser window size
	//viewer->addCoordinateSystem();
	viewer->setSize(screen_width,screen_height);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	pcl::io::savePLYFileASCII(inputcloudfile.substr(0,inputcloudfile.find_last_of('.')).append("_picked.ply"), *clicked_points_3d);
	return;
}

void AreaPick::closeviewer(const pcl::visualization::KeyboardEvent& event, void* args) 
{
    if ((event.getKeySym () == "q" || event.getKeySym () == "Q") && event.keyDown ())
        viewer->~PCLVisualizer();

}
void AreaPick::pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	std::vector< int > indices;
	if (event.getPointsIndices(indices) == -1)
		return;

	for (int i = 0; i < indices.size(); ++i)
	{
		clicked_points_3d->points.push_back(inputcloud->points.at(indices[i]));
	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);

	std::stringstream ss;
	std::string cloudName;
	ss << num++;
	ss >> cloudName;
	cloudName += "_cloudName";

	viewer->addPointCloud(clicked_points_3d, red, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
	return;
}


Pointspick::Pointspick()
{
	inputcloud = PointCloudT::Ptr(new PointCloudT);
	clicked_points_3d= PointCloudT::Ptr(new PointCloudT);
	num = 0;
	cloudName = "PointPick";	
}

void Pointspick::loadInputcloud(string inputcloudfile)
{
	string filetype = inputcloudfile.substr(inputcloudfile.find_last_of('.'), inputcloudfile.size());
	if (filetype == ".pcd")
	{
		if (pcl::io::loadPCDFile(inputcloudfile, *inputcloud) == -1)
		{
			cout << "Can't load pointcloud from " + inputcloudfile << endl;
			return;
		}
	}
	else if (filetype == ".ply")
	{
		if (pcl::io::loadPLYFile(inputcloudfile, *inputcloud) == -1)
		{
			cout << "Can't load pointcloud from " + inputcloudfile << endl;
			return;
		}
	}
	else if(filetype==".stl")
	{
		string ply_path=inputcloudfile.substr(0,inputcloudfile.find_last_of('.')).append(".ply");
		stl_ply(inputcloudfile, ply_path);
		if(pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path,*inputcloud)==-1)
		{
			std::cout<<"couldn't read file "+ply_path<<std::endl;
			return ;
		}
		std::cout<<"target cloud size: "<<inputcloud->size()<<std::endl;
	}
	return;
}

void Pointspick::stl_ply(string stl_path,string ply_path)
{
	struct Coordinate {
		float x, y, z;
		bool operator<(const Coordinate& rhs) {
			return x<rhs.x || (x == rhs.x&&y<rhs.y) || (x == rhs.x&&y == rhs.y&&z<rhs.z);
		}
		bool operator==(const Coordinate& rhs) {
			return x == rhs.x&&y == rhs.y && z == rhs.z;
		}
	};
	vector<Coordinate> vecSorted, vecOrigin;
	vector<Coordinate>::iterator iter, iterBegin;

	int numberOfFacets;
	int numberOfPoints;
	int index;
	char c1[] = "ply\nformat binary_little_endian 1.0\ncomment By ET \nelement vertex ";
	char c2[] = "\nproperty float x\nproperty float y\nproperty float z\nelement face ";
	char c3[] = "\nproperty list uchar int vertex_indices\nend_header\n";
	clock_t start, finish;
	double totaltime;
	start = clock();

	int length;
	int position = 80;
	fstream fileIn(stl_path, ios::in | ios::binary);
	fileIn.seekg(0, ios::end);
	length = (int)fileIn.tellg();
	fileIn.seekg(0, ios::beg);
	char* buffer = new char[length];
	fileIn.read(buffer, length);
	fileIn.close();

	numberOfFacets = *(int*)&(buffer[position]);
	position += 4;
	cout << "Number of Facets: " << numberOfFacets << endl;
	for (int i = 0; i < numberOfFacets; i++)
	{
		Coordinate tmpC;
		position += 12;
		for (int j = 0; j < 3; j++)
		{
			tmpC.x = *(float*)&(buffer[position]);
			position += 4;
			tmpC.y = *(float*)&(buffer[position]);
			position += 4;
			tmpC.z = *(float*)&(buffer[position]);
			position += 4;
			vecOrigin.push_back(tmpC);
		}
		position += 2;
	}

	free(buffer);

	vecSorted = vecOrigin;
	sort(vecSorted.begin(), vecSorted.end());
	iter = unique(vecSorted.begin(), vecSorted.end());
	vecSorted.erase(iter, vecSorted.end());
	numberOfPoints = vecSorted.size();

	ofstream fileOut(ply_path, ios::binary | ios::out | ios::trunc);

	fileOut.write(c1, sizeof(c1) - 1);
	fileOut << numberOfPoints;
	fileOut.write(c2, sizeof(c2) - 1);
	fileOut << numberOfFacets;
	fileOut.write(c3, sizeof(c3) - 1);


	buffer = new char[numberOfPoints * 3 * 4];
	position = 0;
	for (int i = 0; i < numberOfPoints; i++)
	{
		buffer[position++] = *(char*)(&vecSorted[i].x);
		buffer[position++] = *((char*)(&vecSorted[i].x) + 1);
		buffer[position++] = *((char*)(&vecSorted[i].x) + 2);
		buffer[position++] = *((char*)(&vecSorted[i].x) + 3);
		buffer[position++] = *(char*)(&vecSorted[i].y);
		buffer[position++] = *((char*)(&vecSorted[i].y) + 1);
		buffer[position++] = *((char*)(&vecSorted[i].y) + 2);
		buffer[position++] = *((char*)(&vecSorted[i].y) + 3);
		buffer[position++] = *(char*)(&vecSorted[i].z);
		buffer[position++] = *((char*)(&vecSorted[i].z) + 1);
		buffer[position++] = *((char*)(&vecSorted[i].z) + 2);
		buffer[position++] = *((char*)(&vecSorted[i].z) + 3);
	}


	fileOut.write(buffer, numberOfPoints * 3 * 4);

	free(buffer);

	buffer = new char[numberOfFacets * 13];

	for (int i = 0; i < numberOfFacets; i++)
	{
		buffer[13 * i] = (unsigned char)3;
	}

	iterBegin = vecSorted.begin();
	position = 0;
	for (int i = 0; i < numberOfFacets; i++)
	{
		position++;
		for (int j = 0; j < 3; j++)
		{
			iter = lower_bound(vecSorted.begin(), vecSorted.end(), vecOrigin[3 * i + j]);
			index = iter - iterBegin;
			buffer[position++] = *(char*)(&index);
			buffer[position++] = *((char*)(&index) + 1);
			buffer[position++] = *((char*)(&index) + 2);
			buffer[position++] = *((char*)(&index) + 3);

		}
	}

	fileOut.write(buffer, numberOfFacets * 13);

	free(buffer);
	fileOut.close();

	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC * 1000;
	cout << "All Time: " << totaltime << "ms\n";
	return;
}


void Pointspick::simpleViewer(const string inputcloudfile)
{
	//visualizer
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(cloudName));
	viewer->addPointCloud(inputcloud, cloudName);
	viewer->resetCameraViewpoint(cloudName);
	//viewer->addCoordinateSystem();
	viewer->registerPointPickingCallback(&Pointspick::pp_callback, *this);
	//viewer->setFullScreen(true); // Visualiser window size
	viewer->setSize(screen_width,screen_height);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
    //打开文件
	std::ofstream outFile;
	int nexttolast=find_nexttolast(inputcloudfile);
	if(nexttolast==-1) 
	{
		cout<<"wrong save path"<<endl;
		return;
	}
	
	if(inputcloudfile.find("registered")!=string::npos)
		outFile.open(inputcloudfile.substr(0,nexttolast).append("/res/line.txt"));
    else
		outFile.open(inputcloudfile.substr(0,nexttolast).append("/res/coords.txt"));
   
    for(auto p:clicked_points_3d->points)
	{
		outFile<<p.x<<' '<<p.y<<' '<<p.z<<endl;
	}
		
    //关闭文件
    outFile.close();
	//pcl::io::savePLYFileASCII(inputcloudfile.substr(0,inputcloudfile.find_last_of('.')).append("_pickedpoints.ply"), *clicked_points_3d);
	return;
}

void Pointspick::pp_callback(const pcl::visualization::PointPickingEvent& event, void*)
{
	
	if (event.getPointIndex() == -1)
		return;
	float x, y, z;
	event.getPoint(x, y, z);
	std::cout << "Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
	clicked_points_3d->points.push_back(pcl::PointXYZ(x, y, z));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
	
	num++;
	cloudName =to_string(num)+"_cloudName";

	viewer->addPointCloud(clicked_points_3d, red, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
	return;
}

int Pointspick::find_nexttolast(const string filepath)
{
	int index=-1;
	int cnt=0;
	int i=filepath.size();
	while(i--)
	{
		if(filepath[i]=='/')
			cnt++;
		if(cnt==2)
		{
			index=i;
			break;
		}
	}
	return index;
}