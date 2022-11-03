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


    //ȫ�ֱ仯
    //����仯���� 
    if (method == 1)
    {
        //����һ���Զ������
        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        float theta = M_PI / 4;   //��ת�Ķ�����������45��
        transform_1(0, 0) = cos(theta);  //�������Ƶ�Z����ת
        transform_1(0, 1) = -sin(theta);
        transform_1(1, 0) = sin(theta);
        transform_1(1, 1) = cos(theta);
        //   transform_1 (0,2) = 0.3;   //�������������Ч��
        //   transform_1 (1,2) = 0.6;
        //    transform_1 (2,2) = 1;
        transform_1(0, 3) = 0; //������X��ƽ��
        transform_1(1, 3) = 0;
        transform_1(2, 3) = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *transform_cloud1, transform_1);  //���Զ���

    }
    else{
        /// ��ʽ2��Affine3f
        // �����������transform_2.matrix()����ʼ��Ϊ4��4��λ��
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        // ������x���ϵ�ƽ�ƣ�2.5m
        transform_2.translation() << 2.5, 0.0, 0.0; // �������ֱ��ӦX�ᡢY�ᡢZ�᷽���ϵ�ƽ��
        // ������ת������z��
        transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ())); //ͬ��UnitX(),��X�᣻UnitY(),��Y��.
        // ��ӡƽ�ơ���ת����
        std::cout << "\n��ʽ2: ʹ��Affine3f\n";
        std::cout << transform_2.matrix() << std::endl; //ע�⣺����transform_2

        /// ִ��ת��
        // transform_1 ���� transform_2 ������ʵ����ͬ��ת��
        pcl::transformPointCloud(*cloud, *transform_cloud1, transform_2);  //ע�⣺����transform_2.matrix()
    }

    

    //�ֲ��任
    //pcl::transformPointCloud(*cloud, pcl::PointIndices indices, *transform_cloud1, matrix); //��һ������Ϊ���룬�ڶ�������Ϊ��������в��ֵ㼯������������Ϊ�洢���󣬵��ĸ��Ǳ任����


    //�����ݴ洢������
    pcl::io::savePCDFileASCII(pcd_output, *transform_cloud1);

    std::cout << "transform the" << "\"" << pcd_output << "\"" << " complated!" << std::endl;


    //���ӻ�


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
    //���õ�Ĵ�СΪ2
    //view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");

    //v2
    view->addPointCloud(target, target_cloud_color, "target_cloud_v2", v2);
    //view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

    view->addCoordinateSystem(10.0);

    view->spin();


}

#endif