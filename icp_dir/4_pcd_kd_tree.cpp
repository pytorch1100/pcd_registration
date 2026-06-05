#include <iostream>
#include <memory>
#include <vector>
#include <chrono>       // compute compute time

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/registration/icp.h>

using pcdptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using icpI = pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;

int main(int argc, char* argv[])
{
    if (argc < 2)
        std::cerr << "argc = " << argc << std::endl;

    std::string src_pcd_file_name = argv[1];
    std::string dst_pcd_file_name = argv[2];
    std::cout << src_pcd_file_name << std::endl;
    std::cout << dst_pcd_file_name << std::endl;

    pcdptr source_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    pcdptr dst_pcd = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    int ret = pcl::io::loadPCDFile(src_pcd_file_name, *source_pcd);
    ret = pcl::io::loadPCDFile(dst_pcd_file_name, *dst_pcd);
    if (ret == 0)
    {
        std::cout << "load pcd success, ret = " << ret << std::endl;
    }


    // construct kd-tree
    pcl::KdTreeFLANN<pcl::PointXYZ>* kdtree = new  pcl::KdTreeFLANN<pcl::PointXYZ>();
    kdtree->setInputCloud(dst_pcd);

    int K = 1;
    std::vector<float>  dis;
    std::vector<int> index;


    // search  k nearest point
    auto start =  std::chrono::steady_clock::now();
    int match_num = 0;

    #pragma omp parallel for
    for (auto &p : *source_pcd)
    {
        if (kdtree->nearestKSearch(p, K, index, dis) > 0)
        {
            match_num++;
            #if 0
            std::cout << "nearest point num: " << index.size() << endl;
            std::cout << "nearest point dis: " << dis[0] << endl;
            std::cout << "src point: " << p << endl;
            std::cout << "dst point: " << dst_pcd->points[index[0]] << endl;
            #endif
        }
    }
    std::cout << "point src sum num: " << source_pcd->points.size() << std::endl;
    std::cout << "point kdtree sum num: " << dst_pcd->points.size() << std::endl;
    std::cout << "match sum num: " << match_num << std::endl;
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "diff time: " << 
        std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() << "ms"
        << std::endl; 

#if 0
    pcl::visualization::PCLVisualizer viewer("pointcloud viewer befor registration");
    viewer.addPointCloud<pcl::PointXYZ>(source_pcd, "cloud1");

    viewer.addPointCloud<pcl::PointXYZ>(dst_pcd, "cloud2");
    viewer.spin();
#endif

    return 0;
}