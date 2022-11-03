
#if 0

//相关的头文件声明



#include <iostream> //标准C++库中输入输出类相关头文件
#include <pcl/io/pcd_io.h> //pcd读写类相关头文件
#include "pcl/point_cloud.h"
#include <pcl/point_types.h> //PCL中支持的点类型头文件
#include<pcl/common/common.h>
#include <pcl/filters/passthrough.h>

#include<pcl/visualization/cloud_viewer.h>

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
    viewer.setBackgroundColor(0.0, 0.0, 0.0);   //设置背景颜色
}


int main()
{
    //创建一个PointCloud<PointXYZ> boost 共享指针并进行实例化
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //滤波后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //pcd文件路径
    std::string pcd_in = "table_scene_lms400.pcd";
    std::string pcd_out = "rops_cloud_passthroughed.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_in, *cloud) == -1)
        //打开点云文件
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass; //设置滤波器对象
    pass.setInputCloud(cloud);//设置输入点云
    pass.setFilterFieldName("z"); //设置过滤时所需要的点云类型的z字段
    pass.setFilterLimits(minPt.z + 0.2, maxPt.z - 0.2); //设置在过滤字段上的范围
    pass.setFilterLimitsNegative(false); //设置保留范围内的还是过滤掉范围内的
    pass.filter(*cloud_filtered); //执行滤波

    pcl::visualization::CloudViewer viewer("Cloud Viewer");     //创建viewer对象

    viewer.showCloud(cloud_filtered);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    system("pause");
    //将数据存储到磁盘
    pcl::io::savePCDFileASCII(pcd_out, *cloud_filtered);

    return (0);
}



#endif // 0


