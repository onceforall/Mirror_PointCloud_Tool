#include "mirror.hpp"
#include <algorithm>
#include <cmath>

Mirror::Mirror()
{
    mirror_cloud=PointCloudT::Ptr(new PointCloudT);
    ori_cloud=PointCloudT::Ptr(new PointCloudT);
	AbnormalPart_Cloud=PointCloudT::Ptr(new PointCloudT);
	WindowName="Mirror Point Cloud";
}


Coefficient Mirror::get_panal(struct Coord pp[3])
{

    Coefficient C;
	
    C.a=((pp[1].y-pp[0].y)*(pp[2].z-pp[0].z)-(pp[1].z-pp[0].z)*(pp[2].y-pp[0].y));
    C.b=((pp[1].z-pp[0].z)*(pp[2].x-pp[0].x)-(pp[1].x-pp[0].x)*(pp[2].z-pp[0].z));
    C.c=((pp[1].x-pp[0].x)*(pp[2].y-pp[0].y)-(pp[1].y-pp[0].y)*(pp[2].x-pp[0].x));
    C.d=(0-(C.a*pp[0].x+C.b*pp[0].y+C.c*pp[0].z));
    return C;
}

double Mirror::dis_pt2panel(Coord pt,Coefficient C)
{
    return abs(C.a*pt.x+C.b*pt.y+C.c*pt.z+C.d)/sqrt(C.a*C.a+C.b*C.b+C.c*C.c);
}

Coord Mirror::get_mirrorpt(Coord pt,Coefficient C)
{
    Coord mirror_pt;
    
    if(abs(C.a)==0 && abs(C.b)==0 && abs(C.c)==0)
        exit(0);
    else
    {
        double t=-(C.a*pt.x+C.b*pt.y+C.c*pt.z+C.d)/(pow(C.a,2)+pow(C.b,2)+pow(C.c,2));
        mirror_pt.x=pt.x+2*t*C.a;
        mirror_pt.y=pt.y+2*t*C.b;
        mirror_pt.z=pt.z+2*t*C.c;  //it's so importmant .watch out your little mistake
    }
    return mirror_pt;
}

int Mirror::find_nexttolast(const string filepath)
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

void Mirror::get_mirrorpointcloud(string inputfilename)
{
	string mirror_inputfilename=inputfilename.substr(0,inputfilename.find_last_of('.')).append("_picked.ply");
    string abnormal_inputfilename=inputfilename.substr(0,inputfilename.find_last_of('.')).append("_notpicked.ply");
    loadInputcloud(mirror_inputfilename,ori_cloud);
	loadInputcloud(abnormal_inputfilename,AbnormalPart_Cloud);
    struct Coord pp[3];
	int nexttolast=find_nexttolast(inputfilename);
	if(nexttolast==-1) 
	{
		cout<<"wrong save path"<<endl;
		return;
	}
	string coord_filename=inputfilename.substr(0,nexttolast).append("/res/coords.txt");
	string mirror_filename=inputfilename.substr(0,nexttolast).append("/res/mirror.ply");
    ifstream fin(coord_filename);
    if(!fin) exit(0);
 
    for(int i=0;i<1;i++)
    {
        fin>>pp[i].x>>pp[i].y>>pp[i].z;
    }
	pp[1].x=pp[0].x;
	pp[1].y=pp[0].y+1;
	pp[1].z=pp[0].z;
	pp[2].x=pp[0].x;
	pp[2].y=pp[0].y+1;
	pp[2].z=pp[0].z+1;

    Coefficient pc=get_panal(pp);
	cout<<"The plane coeff is "<<pc.a<<' '<<pc.b<<' '<<pc.c<<' '<<pc.d<<endl;
    Coord mirror_pt;
    for(auto point:ori_cloud->points)
    {
        struct Coord pt={point.x,point.y,point.z};
       
        mirror_pt=get_mirrorpt(pt,pc);
        //mirror_cloud->points.push_back(pcl::PointXYZ(point.x,point.y,point.z));
        mirror_cloud->points.push_back(pcl::PointXYZ(mirror_pt.x, mirror_pt.y, mirror_pt.z));    
    }
	fin.close();	
    pcl::io::savePLYFileASCII(mirror_filename, *mirror_cloud);
}

void Mirror::loadInputcloud(string inputcloudfile,PointCloudT::Ptr cloud)
{
	string filetype = inputcloudfile.substr(inputcloudfile.find_last_of('.'), inputcloudfile.size());
	if (filetype == ".pcd")
	{
		if (pcl::io::loadPCDFile(inputcloudfile, *cloud) == -1)
		{
			cout << "Can't load pointcloud from " + inputcloudfile << endl;
			return;
		}
	}
	else if (filetype == ".ply")
	{
		if (pcl::io::loadPLYFile(inputcloudfile, *cloud) == -1)
		{
			cout << "Can't load pointcloud from " + inputcloudfile << endl;
			return;
		}
		else
		{
			cout<<"load pointcloud from "+inputcloudfile<<" width "+to_string(cloud->points.size())<<" points."<<endl;
		}
		
	}
	else if(filetype==".stl")
	{
		string ply_path=inputcloudfile.substr(0,inputcloudfile.find_last_of('.')).append(".ply");
		stl_ply(inputcloudfile, ply_path);
		if(pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path,*cloud)==-1)
		{
			std::cout<<"couldn't read file "+ply_path<<std::endl;
			return ;
		}
		std::cout<<"target cloud size: "<<cloud->size()<<std::endl;
	}
	return;
}

void Mirror::stl_ply(string stl_path,string ply_path)
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

void Mirror::view_mirror()
{

	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(WindowName));
	viewer->addPointCloud(mirror_cloud, WindowName);
	viewer->resetCameraViewpoint(WindowName);
	viewer->addCoordinateSystem(10);
	//viewer->setFullScreen(true); // Visualiser window size
	viewer->setSize(screen_width,screen_height);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


void Mirror::view_comparation()
{
	
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(WindowName)); //定义窗口共享指针
	int v1 ; //定义两个窗口v1，v2，窗口v1用来显示初始位置，v2用以显示配准过程
	int v2 ;
	viewer->setSize(screen_width,screen_height);
	viewer->resetCameraViewpoint(WindowName);
	viewer->addCoordinateSystem(10);

	viewer->createViewPort(0.0,0.0,0.5,1.0,v1);  //四个窗口参数分别对应x_min,y_min,x_max.y_max.
	viewer->createViewPort(0.5,0.0,1.0,1.0,v2);

	viewer->setBackgroundColor(0.0,0.05,0.05,v1); //设着两个窗口的背景色
	viewer->setBackgroundColor(0.05,0.05,0.05,v2);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> sources_cloud_color(AbnormalPart_Cloud,250,0,0); //设置源点云的颜色为红色
	viewer->addPointCloud(AbnormalPart_Cloud,sources_cloud_color,"AbnormalPart_Cloud",v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"AbnormalPart_Cloud");  //设置显示点的大小
	

	pcl::visualization::PointCloudColorHandlerCustom<PointT>  res_cloud(mirror_cloud,0,255,0);  //设置配准结果为白色
	viewer->addPointCloud(mirror_cloud,res_cloud,"Mirror_cloud",v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"Mirror_cloud");

	while(!viewer->wasStopped())
	{
		viewer->spinOnce();  //运行视图
	}
}