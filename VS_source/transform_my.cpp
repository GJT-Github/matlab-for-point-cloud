#if  1


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>


#define method 1


int main() {

    std::string pcd_input = "rabbit.pcd";
    std::string pcd_output = "rabbit_z_45.pcd";


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);



    std::string format_in = pcd_input.substr(pcd_input.length() - 4, 4);
    if (format_in == ".ply")
    {
        pcl::io::loadPLYFile(pcd_input, *cloud);
        /*pcl::io::loadPLYFile(output_filename, *target);*/
    }
    else if (format_in == ".pcd")
    {
        pcl::io::loadPCDFile(pcd_input, *cloud);
        /* pcl::io::loadPCDFile(output_filename, *target);*/
    }


    //全局变化
    //构造变化矩阵 
    if (method == 1)
    {
        //方法一：自定义矩阵
        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        float theta = M_PI / 4;   //旋转的度数，这里是45度
        transform_1(0, 0) = cos(theta);  //这里是绕的Z轴旋转
        transform_1(0, 1) = -sin(theta);
        transform_1(1, 0) = sin(theta);
        transform_1(1, 1) = cos(theta);
        //   transform_1 (0,2) = 0.3;   //这样会产生缩放效果
        //   transform_1 (1,2) = 0.6;
        //    transform_1 (2,2) = 1;
        transform_1(0, 3) = 0; //这里沿X轴平移
        transform_1(1, 3) = 0;
        transform_1(2, 3) = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *transform_cloud1, transform_1);  //不言而喻

    }
    else{
        /// 方式2：Affine3f
        // 创建矩阵对象transform_2.matrix()，初始化为4×4单位阵
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        // 定义在x轴上的平移，2.5m
        transform_2.translation() << 2.5, 0.0, 0.0; // 三个数分别对应X轴、Y轴、Z轴方向上的平移
        // 定义旋转矩阵，绕z轴
        transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ())); //同理，UnitX(),绕X轴；UnitY(),绕Y轴.
        // 打印平移、旋转矩阵
        std::cout << "\n方式2: 使用Affine3f\n";
        std::cout << transform_2.matrix() << std::endl; //注意：不是transform_2

        /// 执行转换
        // transform_1 或者 transform_2 都可以实现相同的转换
        pcl::transformPointCloud(*cloud, *transform_cloud1, transform_2);  //注意：不是transform_2.matrix()
    }

    

    //局部变换
    //pcl::transformPointCloud(*cloud, pcl::PointIndices indices, *transform_cloud1, matrix); //第一个参数为输入，第二个参数为输入点云中部分点集索引，第三个为存储对象，第四个是变换矩阵。


    //将数据存储到磁盘
    pcl::io::savePCDFileASCII(pcd_output, *transform_cloud1);

    std::cout << "transform the" << "\"" << pcd_output << "\"" << " complated!" << std::endl;


    //可视化


    clock_t start, end, time;
    start = clock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

    std::string input_filename = pcd_input;
    std::string output_filename = pcd_output;

    /*string input_filename = "table_scene_lms400.pcd";
    string output_filename = "table_scene_lms400_downsampled.pcd";*/

    std::string format = input_filename.substr(input_filename.length() - 4, 4);
    std::string format_out = output_filename.substr(output_filename.length() - 4, 4);
    //std::cout<<"pointcloud format:"<<format<<std::endl;
    if (format == ".ply")
    {
        pcl::io::loadPLYFile(input_filename, *source);
        /*pcl::io::loadPLYFile(output_filename, *target);*/
    }
    else if (format == ".pcd")
    {
        pcl::io::loadPCDFile(input_filename, *source);
        /* pcl::io::loadPCDFile(output_filename, *target);*/
    }
    if (format_out == ".ply")
    {
        /*pcl::io::loadPLYFile(input_filename, *source);*/
        pcl::io::loadPLYFile(output_filename, *target);
    }
    else if (format_out == ".pcd")
    {
        /* pcl::io::loadPCDFile(input_filename, *source);*/
        pcl::io::loadPCDFile(output_filename, *target);
    }



    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(
        new pcl::visualization::PCLVisualizer("visual"));
    //view->addCoordinateSystem(1.0);
    //view->initCameraParameters ();
    int v1(0);
    int v2(1);

    view->createViewPort(0, 0.0, 0.5, 1.0, v1);
    view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    view->setBackgroundColor(255, 255, 255, v1);
    view->setBackgroundColor(255, 255, 255, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source, 250, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target, 0, 250, 0);

    // v1
    view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
    //设置点的大小为2
    //view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");

    //v2
    view->addPointCloud(target, target_cloud_color, "target_cloud_v2", v2);
    //view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

    view->addCoordinateSystem(10.0);

    view->spin();


}

#endif