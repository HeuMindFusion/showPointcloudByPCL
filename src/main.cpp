#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>

int main(int argc, char** argv)
{
    
    char tmpStr[100];
    strcpy(tmpStr, argv[1]);
    char* pext = strrchr(tmpStr, '.');
    std::string extply("ply");
    std::string extpcd("pcd");
    if (pext) {
        *pext = '\0';
        pext++;
    }
    std::string ext(pext);
    
    if (!((ext == extply) || (ext == extpcd))) {
        std::cout << "file suffix error( pcd and ply)!!!" << std::endl;
        
        return(-1);
    }

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //�������ƶ���ָ�룬���ڴ洢����
    if (ext == extply) {
        if (pcl::io::loadPLYFile(argv[1], *cloud) == -1) {
            PCL_ERROR("Could not read ply file!\n");
            return -1;
        }
    }
    else {
        if (pcl::io::loadPCDFile(argv[1], *cloud) == -1) {
            PCL_ERROR("Could not read pcd file!\n");
            return -1;
        }
    }
    pcl::io::savePLYFile("cloud.ply", *cloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    while (!viewer->wasStopped())
    {
        viewer->removeAllPointClouds();  // 移除当前所有点云
        viewer->addPointCloud(cloud, "3D Viewer");  
        viewer->updatePointCloud(cloud, "3D Viewer"); 
        viewer->spinOnce(0.0000000000001);
       
    }
    viewer->close();
    return 0;
}