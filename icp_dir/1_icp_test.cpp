#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


int main(int argc, char* argv[])
{
    pcl::PointXYZINormal p1;
    p1.x = 1;
    p1.y = 2;
    p1.z = 3;

    p1.intensity = 1.3;

    p1.normal_x = 2.3;
    p1.normal_y = 2.3;
    p1.normal_z = 2.3;

    std::cout << "p1:" << p1 << std::endl;
    std::cout << "size of p1:" << sizeof(p1) << std::endl;  // aligned 16bytes

    return 0;
}