#include "mirror.hpp"
#include <algorithm>
#include <cmath>

Mirror::Mirror()
{
    mirror_cloud=PointCloudT::Ptr(new PointCloudT);
    ori_cloud=PointCloudT::Ptr(new PointCloudT);
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
    // if(C.a!=0 && C.b!=0 && C.c!=0)
    // {
    //     mirror_pt.x=-1*((pow(C.a,2)-pow(C.b,2)-pow(C.c,2))/C.a+2*C.b*pt.y+2*C.c*pt.z+2*C.d)/(pow(C.a,2)+pow(C.b,2)+pow(C.c,2));
    //     mirror_pt.y=pt.y-C.b/C.a*(pt.x-mirror_pt.x);
    //     mirror_pt.z=pt.z-C.c/C.a*(pt.x-mirror_pt.x);
    // }
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

void Mirror::get_mirrorpointcloud(string inputcloudfilename)
{
    loadInputcloud(inputcloudfilename);
    struct Coord pp[3];
	int nexttolast=find_nexttolast(inputcloudfilename);
	string coord_filename=inputcloudfilename.substr(0,nexttolast).append("/res/coords.txt");
	string mirror_filename=inputcloudfilename.substr(0,nexttolast).append("/res/mirror.ply");
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
	cout<<"the coeff is "<<pc.a<<' '<<pc.b<<' '<<pc.c<<' '<<pc.d<<endl;
    Coord mirror_pt;
    for(auto point:ori_cloud->points)
    {
        struct Coord pt={point.x,point.y,point.z};
       
        mirror_pt=get_mirrorpt(pt,pc);
        mirror_cloud->points.push_back(pcl::PointXYZ(point.x,point.y,point.z));
        mirror_cloud->points.push_back(pcl::PointXYZ(mirror_pt.x, mirror_pt.y, mirror_pt.z));    
    }
	fin.close();	
    pcl::io::savePLYFileASCII(mirror_filename, *mirror_cloud);
}

void Mirror::loadInputcloud(string inputcloudfile)
{
	string filetype = inputcloudfile.substr(inputcloudfile.find_last_of('.'), inputcloudfile.size());
	if (filetype == ".pcd")
	{
		if (pcl::io::loadPCDFile(inputcloudfile, *ori_cloud) == -1)
		{
			cout << "Can't load pointcloud from " + inputcloudfile << endl;
			return;
		}
	}
	else if (filetype == ".ply")
	{
		if (pcl::io::loadPLYFile(inputcloudfile, *ori_cloud) == -1)
		{
			cout << "Can't load pointcloud from " + inputcloudfile << endl;
			return;
		}
		else
		{
			cout<<"load pointcloud from "+inputcloudfile<<" width "+to_string(ori_cloud->points.size())<<" points."<<endl;
		}
		
	}
	else if(filetype==".stl")
	{
		string ply_path=inputcloudfile.substr(0,inputcloudfile.find_last_of('.')).append(".ply");
		stl_ply(inputcloudfile, ply_path);
		if(pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path,*ori_cloud)==-1)
		{
			std::cout<<"couldn't read file "+ply_path<<std::endl;
			return ;
		}
		std::cout<<"target cloud size: "<<ori_cloud->size()<<std::endl;
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