#include <iostream>
#include <memory>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>

#include <pcl/registration/icp.h>

using pcdptr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
using icpI = pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>;

int main(int argc, char* argv[])
{
    if (argc < 2)
        std::cerr << "argc = " << argc << std::endl;

    std::string src_pcd_file_name = argv[1];
    std::string dst_pcd_file_name = argv[2];
    std::cout << src_pcd_file_name << std::endl;
    std::cout << dst_pcd_file_name << std::endl;

    pcdptr source_pcd(new pcl::PointCloud<pcl::PointXYZI>);
    pcdptr dst_pcd = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    int ret = pcl::io::loadPCDFile(src_pcd_file_name, *source_pcd);
    ret = pcl::io::loadPCDFile(dst_pcd_file_name, *dst_pcd);
    if (ret == 0)
    {
        std::cout << "load pcd success, ret = " << ret << std::endl;
    }



    // 1. use pcl icp
    icpI icp_obj;

    icp_obj.setMaximumIterations(10);
    icp_obj.setTransformationEpsilon(1e-10);      // 变换矩阵变化阈值
    icp_obj.setEuclideanFitnessEpsilon(1e-6);     // 均方误差变化阈值
    icp_obj.setMaxCorrespondenceDistance(0.5);   // 最大对应点距离

    pcdptr aligned(new pcl::PointCloud<pcl::PointXYZI>);
    icp_obj.setInputSource(source_pcd);
    icp_obj.setInputTarget(dst_pcd);
    icp_obj.align(*aligned);

        // 检查是否收敛
    if (icp_obj.hasConverged()) {
        std::cout << "ICP converged!" << std::endl;
        std::cout << "Fitness score: " << icp_obj.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:\n" << icp_obj.getFinalTransformation() << std::endl;
    } else {
        std::cout << "ICP did not converge!" << std::endl;
    }

    Eigen::Matrix3f R(icp_obj.getFinalTransformation().block<3, 3>(0, 0));
    Eigen::Quaternionf q(R);
     std::cout << "q: " << q.w() << q.x() << q.y() << q.z() << std::endl;

    pcl::visualization::PCLVisualizer viewer("pointcloud viewer befor registration");
    // viewer.addPointCloud<pcl::PointXYZI>(source_pcd, "cloud1");

    viewer.addPointCloud<pcl::PointXYZI>(dst_pcd, "cloud2");

    viewer.addPointCloud<pcl::PointXYZI>(aligned, "aligned");
    viewer.spin();

    return 0;
}