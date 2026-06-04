#include <iostream>
#include <memory>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>

using pcdptr = pcl::PointCloud<pcl::PointXYZINormal>::Ptr;

int main(int argc, char* argv[])
{
    if (argc < 2)
        std::cerr << "argc = " << argc << std::endl;

    std::string src_pcd_file_name = argv[1];
    std::string dst_pcd_file_name = argv[2];
    std::cout << src_pcd_file_name << std::endl;
    std::cout << dst_pcd_file_name << std::endl;

    pcdptr source_pcd(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcdptr dst_pcd = boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
    int ret = pcl::io::loadPCDFile(src_pcd_file_name, *source_pcd);
    ret = pcl::io::loadPCDFile(dst_pcd_file_name, *dst_pcd);
    std::cout << "ret = " << ret << std::endl;

    if (ret >= 0)
    {
        std::cout << "pcd size: " << source_pcd->size() << std::endl;
        std::cout << "pcd width: " << source_pcd->width << std::endl;
        std::cout << "pcd height: " << source_pcd->height << std::endl;
    }

    pcl::visualization::PCLVisualizer viewer("pointcloud viewer");
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> handle(source_pcd, "z");
    viewer.addPointCloud<pcl::PointXYZINormal>(source_pcd, "cloud1");

    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> handle_dst(dst_pcd, "z");
    viewer.addPointCloud<pcl::PointXYZINormal>(dst_pcd, "cloud2");
    viewer.spin();

    return 0;
}

#if 0
#include <iostream>
#include <memory>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>

using pcdptr = pcl::PointCloud<pcl::PointXYZINormal>::Ptr;

int main(int argc, char* argv[])
{
    if (argc < 2)
        std::cerr << "argc = " << argc << std::endl;

    std::string src_pcd_file_name = argv[1];
    std::cout << src_pcd_file_name << std::endl;

    pcl::PointCloud<pcl::PointXYZINormal> source_pcd;
    int ret = pcl::io::loadPCDFile(src_pcd_file_name, source_pcd);
    std::cout << "ret = " << ret << std::endl;

    if (ret >= 0)
    {
        std::cout << "pcd size: " << source_pcd.size() << std::endl;
        std::cout << "pcd width: " << source_pcd.width << std::endl;
        std::cout << "pcd height: " << source_pcd.height << std::endl;
    }

    pcl::visualization::PCLVisualizer viewer("pointcloud viewer");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> handle(static_cast<pcdptr>(&source_pcd), "z");
    viewer.addPointCloud<pcl::PointXYZINormal>(static_cast<pcdptr>(&source_pcd), handle);
    viewer.spin();

    return 0;
}
#endif