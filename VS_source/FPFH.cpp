
#define FPFH false                   //���ٵ���ֱ��ͼ��fpfh��

#if FPFH


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
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
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
    //������
    pointnormal::Ptr point_normal(new pointnormal);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
    est_normal.setInputCloud(input_cloud);
    est_normal.setSearchMethod(tree);
    est_normal.setKSearch(10);
    est_normal.compute(*point_normal);
    //fpfh ����
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    //pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
    est_fpfh.setNumberOfThreads(4); //ָ��4�˼���
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
    est_fpfh.setInputCloud(input_cloud);
    est_fpfh.setInputNormals(point_normal);
    est_fpfh.setSearchMethod(tree);
    est_fpfh.setKSearch(10);
    est_fpfh.compute(*fpfh);

    return fpfh;

}

int main(int argc, char** argv)
{


   
    if (argc < 3)
    {
        cout << "please input two pointcloud" << endl;
        return -1;
    }
    clock_t start, end, time;
    start = clock();
    pointcloud::Ptr source(new pointcloud);
    pointcloud::Ptr target(new pointcloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());


    string input_filename = argv[1];
    string output_filename = argv[2];


    std::string format = input_filename.substr(input_filename.length() - 4, 4);
    //std::cout<<"pointcloud format:"<<format<<std::endl;
    if (format == ".ply")
    {
        pcl::io::loadPLYFile(input_filename, *source);
        pcl::io::loadPLYFile(output_filename, *target);
    }
    else if (format == ".pcd")
    {
        pcl::io::loadPCDFile(input_filename, *source);
        pcl::io::loadPCDFile(output_filename, *target);
    }


    fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);
    fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);

    //����(ռ���˴󲿷�����ʱ��)
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target);
    sac_ia.setTargetFeatures(target_fpfh);
    pointcloud::Ptr align(new pointcloud);
    //  sac_ia.setNumberOfSamples(20);  //����ÿ�ε���������ʹ�õ�������������ʡ��,�ɽ�ʡʱ��
    //sac_ia.setCorrespondenceRandomness(6); //���ü���Э����ʱѡ����ٽ��ڵ㣬��ֵԽ��Э����Խ��ȷ�����Ǽ���Ч��Խ��.(��ʡ)
    sac_ia.align(*align);
    end = clock();
    cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;

    //���ӻ�
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh test"));
    int v1;
    int v2;

    view->createViewPort(0, 0.0, 0.5, 1.0, v1);
    view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    view->setBackgroundColor(0, 0, 0, v1);
    view->setBackgroundColor(0.05, 0, 0, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source, 250, 0, 0);
    view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target, 0, 250, 0);
    view->addPointCloud(target, target_cloud_color, "target_cloud_v1", v1);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(align, 255, 0, 0);
    view->addPointCloud(align, aligend_cloud_color, "aligend_cloud_v2", v2);
    view->addPointCloud(target, target_cloud_color, "target_cloud_v2", v2);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "aligend_cloud_v2");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

    //   view->addCorrespondences<pcl::PointXYZ>(source,target,*cru_correspondences,"correspondence",v1);//�����ʾ��Ӧ���
    while (!view->wasStopped())
    {
        // view->spin();
        view->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));


    }
    pcl::io::savePCDFile("crou_output.pcd", *align);
    //  pcl::io::savePCDFile ("final_align.pcd", *final);

    return 0;
}

#if 0
//pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_grid;
pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_grid;
approximate_voxel_grid.setLeafSize(0.5, 0.5, 0.5); //����߳�.�������ֵԽ���򾫼��Խ������ʣ�µ������٣�
pointcloud::Ptr source(new pointcloud);
pointcloud::Ptr sample_sources(new pointcloud);
approximate_voxel_grid.setInputCloud(source);
approximate_voxel_grid.filter(*sample_source);
cout << "source voxel grid  Filte cloud size is " << sample_source->size() << endl;
// pcl::io::savePCDFile("voxelgrid.pcd",*out);


pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;

boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
crude_cor_est.setInputSource(source_fpfh);
crude_cor_est.setInputTarget(target_fpfh);
//  crude_cor_est.determineCorrespondences(cru_correspondences);
crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
cout << "crude size is:" << cru_correspondences->size() << endl;




#endif // 0

#endif // FPFH