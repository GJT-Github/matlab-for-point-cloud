/*
���ӻ����� pcl_visualizer

����̨����

(������ʾ)
p,P :�л������ڵ�ı�ʾ
w,W :�л��������ߵı�ʾ
s,S :�л���������ı�ʾ

(��Ĵ�С)
-/+ :��С��Ĵ�С/���ӵ�Ĵ�С
-/+ [+Alt]  :Զ��/��������

���˳���
e,E :�˳�������
q,Q :ֹͣ������VTK'TerminateApp

(���)
g,G :�򿪡��رձ��

l,L :Ϊ��ǰ��ͼ�г����п��õļ��κ���ɫ�������

o,O :��͸�ӡ�ƽ��ͶӰ֮���л���Ĭ����͸�ӣ�

x,X :�л���Ƥ��ģʽѡ��ڵ�Ϊ������������ѡ���

j,J :��ȡһ��.png����

c,C :��ʾ��ǰ����ʹ��ڲ���

f,F :�����ģʽ



Alt +  r,R  :������׶���ӵ�����
Ctrl + s,S  :�����������
Ctrl + r,R  :��ԭ�������
Alt + s,S   :����������ģʽ
Alt + f,F   :��󴰿ڴ�С��ԭʼ���ڴ�С�л�
Alt + 0..9 +Ctrl:�ڲ�ͬ�ļ��δ��������л�
0..9 + Ctrl :�ڲ�ͬ����ɫ���������л�
Shift + Left click:



*/


#define creat_pcd false             //����pcd�ļ�
#define read_pcd false              //��ȡ��������
#define link_two_point_cloud false   //����������ͬ����
#define Kd_tree false                //k_d�������������ڶ�ά�ռ�ؼ����ݵ���������Χ�����������������
#define Two_window_show_point_cloud false//������������ʾ����


#if 0

#elif Two_window_show_point_cloud       //������������ʾ����

#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //����fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //�����Ĵ����Ӧ��ϵȥ��
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //�������һ����ȥ��
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>


using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;


#define load_point(format,input,point)  if (format == ".ply")       \
                             pcl::io::loadPLYFile(input, *point);   \
                            else if (format == ".pcd")pcl::io::loadPCDFile(input, *point);


int main(int argc, char** argv)
{
    clock_t start, end, time;
    start = clock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

    if (argc != 3)
    {
        cerr << "���������������!" << endl;
        exit(1);
    }

    string input_filename = argv[1];
    string output_filename = argv[2];

    /*string input_filename = "table_scene_lms400.pcd";
    string output_filename = "table_scene_lms400_downsampled.pcd";*/

    std::string format = input_filename.substr(input_filename.length() - 4, 4);
    ////std::cout<<"pointcloud format:"<<format<<std::endl;
    //if (format == ".ply")
    //{
    //    pcl::io::loadPLYFile(input_filename, *source);
    //    pcl::io::loadPLYFile(output_filename, *target);
    //}
    //else if (format == ".pcd")
    //{
    //    pcl::io::loadPCDFile(input_filename, *source);
    //    pcl::io::loadPCDFile(output_filename, *target);
    //}
   
    load_point(format, input_filename, source)
    format = output_filename.substr(output_filename.length() - 4, 4);
    load_point(format, output_filename, target)



    //���ӻ�
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

    //while (!view->wasStopped())
    //{
         view->spin();
    //    view->spinOnce(100);
    //    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    //}
    /*system("pause");*/
    return 0;
}




#elif Kd_tree

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

#include<pcl/visualization/cloud_viewer.h>


//void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
//    viewer.setBackgroundColor(0.0, 0.0, 0.0);   //���ñ�����ɫ
//}


int
main(int argc, char** argv)
{
    srand(time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // ���ɵ�������
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }


    //pcl::visualization::CloudViewer viewer("Cloud Viewer");     //����viewer����

    //viewer.showCloud(cloud);
    //viewer.runOnVisualizationThreadOnce(viewerOneOff);
    //system("pause");




    //����һ��kd_tree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud(cloud);

    pcl::PointXYZ searchPoint; // Ҫ�����ĵ�

    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);


 


    // ���������
    int K = 10;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    std::cout << "K nearest neighbor search at (" << searchPoint.x
        << " " << searchPoint.y
        << " " << searchPoint.z
        << ") with K=" << K << std::endl;

    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
            << " " << cloud->points[pointIdxNKNSearch[i]].y
            << " " << cloud->points[pointIdxNKNSearch[i]].z
            << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    // ���շ�Χ���������ո����İ뾶��������

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << searchPoint.x
        << " " << searchPoint.y
        << " " << searchPoint.z
        << ") with radius=" << radius << std::endl;


    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
            << " " << cloud->points[pointIdxRadiusSearch[i]].y
            << " " << cloud->points[pointIdxRadiusSearch[i]].z
            << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }



    return 0;
}


#elif link_two_point_cloud          //����������ͬ����


#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

int main(int argc,char** argv) {
    if (argc != 2) {
        std::cerr << "use '-f' or '-p' " << std::endl;  //-p���������ӣ�-f����ά��ƴ��
        exit(0);
    }
    pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
    pcl::PointCloud<pcl::Normal> n_cloud_b;
    pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

    //����������
    cloud_a.width = 5;
    cloud_a.height = cloud_b.height = n_cloud_b.height = 1;
    cloud_a.points.resize(cloud_a.width * cloud_a.height);
    if (strcmp(argv[1], "-p") == 0) {
        cloud_b.width = 3;
        cloud_b.points.resize(cloud_b.width * cloud_b.height);
    }
    else {
        n_cloud_b.width = 5;
        n_cloud_b.points.resize(n_cloud_b.width * n_cloud_b.height);    
    }
    for (size_t i = 0; i < cloud_a.points.size(); ++i) {
        cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    if(strcmp(argv[1],"-p")==0)
        for (size_t i = 0; i < cloud_b.points.size(); ++i) {
            cloud_b.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud_b.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud_b.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        }
    else
        for (size_t i = 0; i < n_cloud_b.points.size(); ++i) {
            n_cloud_b.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
            n_cloud_b.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
            n_cloud_b.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
        }
    std::cerr << "Cloud A: " << std::endl;
    for (size_t i = 0; i < cloud_a.points.size(); ++i) {
        std::cerr << "  " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;
    }
    std::cerr << "Cloud B: " << std::endl;
    if (strcmp(argv[1], "-p") == 0)
        for (size_t i = 0; i < cloud_b.points.size(); ++i)
            std::cerr << "  " << cloud_b.points[i].x << " " << cloud_b.points[i].y << " " << cloud_b.points[i].z << std::endl;
    else
        for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
            std::cerr << "  " << n_cloud_b.points[i].normal[0] << " " << n_cloud_b.points[i].normal[1] << " " << n_cloud_b.points[i].normal[2] << std::endl;
   //Copy the point cloud data
    if (strcmp(argv[1], "-p") == 0) {
        //���ӵ�
        cloud_c = cloud_a;
        cloud_c += cloud_b;
        std::cerr << "Cloud C: " << std::endl;
        for (size_t i = 0; i < cloud_c.points.size(); ++i)
            std::cerr << "   " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z <<"  "<< std::endl;
    }
    else {
        //�����ֶ�
        pcl::concatenateFields(cloud_a, n_cloud_b, p_n_cloud_c);
        std::cerr << "Cloud C: " << std::endl;
        for (size_t i = 0; i < p_n_cloud_c.points.size(); ++i)
            std::cerr << "   " << p_n_cloud_c.points[i].x << " " << p_n_cloud_c.points[i].y << " " << p_n_cloud_c.points[i].z << " " << \
            p_n_cloud_c.points[i].normal[0] << " " << p_n_cloud_c.points[i].normal[1] << " " << p_n_cloud_c.points[i].normal[2] << std::endl;

    }
            
    return 0;
}



#elif read_pcd                      //��ȡ��������
#include <iostream>
#include <pcl/io/pcd_io.h>          //������Ҫ������I/O��������
#include <pcl/point_types.h>        //������Ҫ�� һЩ������������

int main(int argc, char** argv)
{
    // ����һ������ָ�벢��ʼ��
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud) == -1) // �����Լ���·��
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from test_pcd.pcd with the following fields: "
        << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cout << "    " << cloud->points[i].x
        << " " << cloud->points[i].y
        << " " << cloud->points[i].z << std::endl;

    return (0);
}



#elif creat_pcd                 //����pcd�ļ�

#include <iostream>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

    for (size_t i = 0; i < cloud.points.size(); ++i)
        std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    return (0);
}







#endif // 0


