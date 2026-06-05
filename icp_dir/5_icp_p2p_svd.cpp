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

#include <Eigen/Dense>

using pointtype = pcl::PointXYZ;
using pcdptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using icpI = pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;


void compute_mean(pcdptr &pcd, pointtype &pcd_mean)
{
    int sum = pcd->points.size();
    for (auto& p : *pcd)
    {
        pcd_mean.x += p.x;
        pcd_mean.y += p.y;
        pcd_mean.z += p.z;
    }
    pcd_mean.x /= sum;
    pcd_mean.y /= sum;
    pcd_mean.z /= sum;
}

void test_compute_mean()
{
    // 测试代码示例
    pcdptr test_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    test_cloud->points.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    test_cloud->points.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
    test_cloud->points.push_back(pcl::PointXYZ(7.0, 8.0, 9.0));

    pointtype mean;
    compute_mean(test_cloud, mean); // 期望输出 (4.0, 5.0, 6.0)
    std::cout << "Mean: " << mean << std::endl;

    Eigen::Matrix<double, 3, Eigen::Dynamic> mat(3, 3);
    Eigen::Vector3d vec_mean(mean.x, mean.y, mean.z);
    Eigen::Vector3d vec_p = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; ++i)
    {
        auto& p = test_cloud->points[i];
        vec_p << p.x, p.y, p.z;
        std::cout << "vec_p: " << vec_p << std::endl;
        mat.block<3, 1>(0, i) = vec_p - vec_mean;
    }
    std::cout << "test_mat: " << mat << std::endl;
}

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr matched_points(new pcl::PointCloud<pcl::PointXYZ>());   // matched points in target_points


    // search  k nearest point
    auto start =  std::chrono::steady_clock::now();
    int match_num = 0;

    for (auto &p : *source_pcd)
    {
        if (kdtree->nearestKSearch(p, K, index, dis) > 0)
        {
            
            matched_points->points.emplace_back(dst_pcd->points[index[0]]);

            // std::cout << "matched point: " << matched_points->points[match_num] << endl;
            // std::cout << "dst point: " << dst_pcd->points[index[0]] << endl;
            match_num++;
        }
    }

    // use svd to get R and t
    pointtype mean_src_pcd, mean_matched_pcd;
    compute_mean(source_pcd,mean_src_pcd);
    compute_mean(matched_points, mean_matched_pcd);
    std::cout << "mean_src_pcd: " << mean_src_pcd << std::endl;
    std::cout << "mean_matched_pcd: " << mean_matched_pcd << std::endl;

    // construct Eigen Matrix srcMat and dstMat, use Eigen to do mat computation
    Eigen::Matrix<double, 3, Eigen::Dynamic> srcMat(3, match_num);  
    Eigen::Matrix<double, 3, Eigen::Dynamic> dstMat(3, match_num);

    Eigen::Vector3d mean_src_vec(mean_src_pcd.x, mean_src_pcd.y, mean_src_pcd.z);
    Eigen::Vector3d mean_matched_vec(mean_matched_pcd.x, mean_matched_pcd.y, mean_matched_pcd.z);

    for (int i = 0; i < match_num; ++i)
    {
        auto &pcl_p = source_pcd->points[i];
        Eigen::Vector3d src_p(pcl_p.x, pcl_p.y, pcl_p.z);
        srcMat.block<3, 1>(0, i) = src_p - mean_src_vec;

        auto &pcl_matched_p = matched_points->points[i];
        Eigen::Vector3d matched_p(pcl_matched_p.x, pcl_matched_p.y, pcl_matched_p.z);
        dstMat.block<3, 1>(0, i) = matched_p - mean_matched_vec;
    }

    // Get R
    Eigen::Matrix3d W = srcMat * dstMat.transpose();    // construct Matrix W
    std::cout << "W: " << W << std::endl;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R  = svd.matrixU() * (svd.matrixV().transpose());
    std::cout << "R: " << R << std::endl;

    Eigen::Quaterniond q(R);
    std::cout << "q: " << q.w() << q.x() << q.y() << q.z() << std::endl;

    
    //test_compute_mean();
    


#if 0
    pcl::visualization::PCLVisualizer viewer("pointcloud viewer befor registration");
    viewer.addPointCloud<pcl::PointXYZ>(source_pcd, "cloud1");

    viewer.addPointCloud<pcl::PointXYZ>(dst_pcd, "cloud2");
    viewer.spin();
#endif

    return 0;
}