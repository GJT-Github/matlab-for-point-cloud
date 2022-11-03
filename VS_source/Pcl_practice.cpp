#define Filters_passthrough false                       //�˲���
#define Filters_VoxelGrid   false                       //��VoxelGrid��������PointCloud�����²���(); \
                                                            ���ػ����񷽷��Ե������ݼ����н������������ٵ�����
#define Filters_StatisticsOutlierRemoval false          //StatisticsOutlierRemoval������ɾ����Ⱥֵ
#define project_inliers false                           //����ͶӰ������ģ�ͣ����磬ƽ�棬����ȣ��ϡ�\
                                                            ����ģ��ͨ��һ��ϵ������ - ��ƽ������£�\
                                                            ͨ���䷽��ʽ��ax + by + cz + d = 0��
#define Filters_ExtractIndices false                    //ʹ�ã�ExclIndices <pcl :: ExtractIndices>\
                                                            ���������ڷֶ��㷨����������ӵ�������ȡ����Ӽ���
#define planar_segmentation false                       //��һ�����м򵥵�ƽ��ָ���ڵ������ҵ�֧��ƽ��ģ�͵����е㡣
#define bianhuan false                                  //�Ե��ƽ���ȫ�ֻ�ֲ��任


#if 0

#elif bianhuan                                         //�Ե��ƽ���ȫ�ֻ�ֲ��任

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>


int main() {
    
    std::string pcd_input = "monkey.pcd";
    std::string pcd_output = "monkey_downsampled.pcd";
    

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

    //�ֲ�
    //pcl::transformPointCloud(*cloud, pcl::PointIndices indices, *transform_cloud1, matrix); //��һ������Ϊ���룬�ڶ�������Ϊ��������в��ֵ㼯������������Ϊ�洢���󣬵��ĸ��Ǳ任����


    //�����ݴ洢������
    pcl::io::savePCDFileASCII(pcd_output, *transform_cloud1);

    std::cout << "transform the"<<"\"" << pcd_output <<"\""<< " complated!" << std::endl;


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







#elif planar_segmentation                               //��һ�����м򵥵�ƽ��ָ���ڵ������ҵ�֧��ƽ��ģ�͵����е㡣

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

int
main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width = 15;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // Generate the data
    for (auto& point : *cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1.0;
    }

    // Set a few outliers
    (*cloud)[0].z = 2.0;
    (*cloud)[3].z = -2.0;
    (*cloud)[6].z = 4.0;

    std::cerr << "Point cloud data: " << cloud->size() << " points" << std::endl;
    for (const auto& point : *cloud)
        std::cerr << "    " << point.x << " "
        << point.y << " "
        << point.z << std::endl;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
        << coefficients->values[1] << " "
        << coefficients->values[2] << " "
        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
   // for (std::size_t i = 0; i < inliers->indices.size(); ++i)
        for (const auto& idx : inliers->indices)
            std::cerr << idx << "    " << cloud->points[idx].x << " "
            << cloud->points[idx].y << " "
            << cloud->points[idx].z << std::endl;

    return (0);
}





#elif Filters_ExtractIndices                            //���ڷֶ��㷨����������ӵ�������ȡ����Ӽ���

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int
main(int argc, char** argv)
{
    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read("table_scene_lms400.pcd", *cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // Write the downsampled version to disk
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int)cloud_filtered->size();
    // While 30% of the original cloud is still there
    while (cloud_filtered->size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        std::stringstream ss;
        ss << "table_scene_lms400_plane_" << i << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
        i++;
    }

    return (0);
}


#elif project_inliers                               //����ͶӰ������ģ��


#include <iostream>
#include <pcl/point_cloud.h> // for PointCloud
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (auto& point : *cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before projection: " << std::endl;
    for (const auto& point : *cloud)
        std::cerr << "    " << point.x << " "
        << point.y << " "
        << point.z << std::endl;

    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    std::cerr << "Cloud after projection: " << std::endl;
    for (const auto& point : *cloud_projected)
        std::cerr << "    " << point.x << " "
        << point.y << " "
        << point.z << std::endl;

    return (0);
}





#elif Filters_StatisticsOutlierRemoval              //Filters_StatisticsOutlierRemoval

#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/statistical_outlier_removal.h>

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //Fill in the cloud data
    pcl::PCDReader reader;
    //Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    //Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("table_scene_lms400_inliners.pcd", *cloud_filtered, false);

    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ>("table_scene_lms400_outliners.pcd", *cloud_filtered, false);

    return 0;
}











#elif Filters_VoxelGrid                             //��VoxelGrid��������PointCloud�����²���();\
                                                      ���ػ����񷽷��Ե������ݼ����н������������ٵ�����
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//
//int main(int argc, char** argv)
//{
//    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
//    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
//
//    // Fill in the cloud data
//    pcl::PCDReader reader;
//    // Replace the path below with the path where you saved your file
//    reader.read("rabbit.pcd", *cloud); // Remember to download the file first!
//
//    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//        << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
//
//    // Create the filtering object
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//    sor.setInputCloud(cloud);
//    sor.setLeafSize(0.01f, 0.01f, 0.01f);
//    sor.filter(*cloud_filtered);
//
//    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//        << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
//
//    pcl::PCDWriter writer;
//    writer.write("rabbit_downsampled.pcd", *cloud_filtered,
//        Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
//
//
//    std::cout<<"���������²����ٷֱȣ�" << (1- (float)(cloud_filtered->width * cloud_filtered->height)/(cloud->width * cloud->height))*100<<"%"<< std::endl;
//    return (0);
//}









#elif Filters_passthrough                     //passthrough�˲���


#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (auto& point : *cloud)             //����point��������*clude����ͨ����point�ĸ�ֵ������*clude���ݵ����
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    for (const auto& point : *cloud)
        std::cerr << "    " << point.x << " "
        << point.y << " "
        << point.z << std::endl;

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");               //�������ֶ���������Ϊz����
    pass.setFilterLimits(100, 500);             //�ɽ��ܵļ��ֵ����Ϊ��100.0; 500.0��
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    for (const auto& point : *cloud_filtered)
        std::cerr << "    " << point.x << " "
        << point.y << " "
        << point.z << std::endl;

    return (0);
}



#endif // 0

