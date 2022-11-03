
#if 0

//��ص�ͷ�ļ�����



#include <iostream> //��׼C++����������������ͷ�ļ�
#include <pcl/io/pcd_io.h> //pcd��д�����ͷ�ļ�
#include "pcl/point_cloud.h"
#include <pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ�
#include<pcl/common/common.h>
#include <pcl/filters/passthrough.h>

#include<pcl/visualization/cloud_viewer.h>

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
    viewer.setBackgroundColor(0.0, 0.0, 0.0);   //���ñ�����ɫ
}


int main()
{
    //����һ��PointCloud<PointXYZ> boost ����ָ�벢����ʵ����
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //�˲���ĵ���
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //pcd�ļ�·��
    std::string pcd_in = "table_scene_lms400.pcd";
    std::string pcd_out = "rops_cloud_passthroughed.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_in, *cloud) == -1)
        //�򿪵����ļ�
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    // �����˲�������
    pcl::PassThrough<pcl::PointXYZ> pass; //�����˲�������
    pass.setInputCloud(cloud);//�����������
    pass.setFilterFieldName("z"); //���ù���ʱ����Ҫ�ĵ������͵�z�ֶ�
    pass.setFilterLimits(minPt.z + 0.2, maxPt.z - 0.2); //�����ڹ����ֶ��ϵķ�Χ
    pass.setFilterLimitsNegative(false); //���ñ�����Χ�ڵĻ��ǹ��˵���Χ�ڵ�
    pass.filter(*cloud_filtered); //ִ���˲�

    pcl::visualization::CloudViewer viewer("Cloud Viewer");     //����viewer����

    viewer.showCloud(cloud_filtered);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    system("pause");
    //�����ݴ洢������
    pcl::io::savePCDFileASCII(pcd_out, *cloud_filtered);

    return (0);
}



#endif // 0


