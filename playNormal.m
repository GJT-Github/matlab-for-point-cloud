%come from :https://www.cnblogs.com/tiandsp/p/10662783.html


clear all;
close all;
clc;
warning off;

addpath(genpath('./source/readDatas'))

% pc = pcread('rabbit.pcd');
pc = loadpcd('rabbit.pcd');
pc = pc';
pc = pointCloud(pc);
% pc=pcdownsample(pc,'random',0.3);       %0.3��������
pcshow(pc);

pc_point = pc.Location';                %�õ���������
kdtree = vl_kdtreebuild(pc_point);      %ʹ��vlfeat����kdtree

normE=[];
% tic
for i=1:length(pc_point)
    
    p_cur = pc_point(:,i);
    [index, distance] = vl_kdtreequery(kdtree, pc_point, p_cur, 'NumNeighbors', 10);    %Ѱ�ҵ�ǰ�������10����
    p_neighbour = pc_point(:,index)';
    p_cent = mean(p_neighbour);     % �õ��ֲ�����ƽ��ֵ�����ڼ��㷨�������Ⱥͷ���
    
    %��С���˹���ƽ��
    X=p_neighbour(:,1);
    Y=p_neighbour(:,2);
    Z=p_neighbour(:,3);
    XX=[X Y ones(length(index),1)];
    YY=Z;
    %�õ�ƽ�淨����
    C=(XX'*XX)\XX'*YY;
    
    %�ֲ�ƽ��ָ��ֲ����ĵ�����
    dir1 = p_cent-p_cur';
    %�ֲ�ƽ�淨����
    dir2=[C(1) C(2) -1];
    
    %�������������ļн�
    ang = sum(dir1.*dir2) / (sqrt(dir1(1)^2 +dir2(1)^2) + sqrt(dir1(2)^2 +dir2(2)^2)+sqrt(dir1(3)^2 +dir2(3)^2) );
    
    %���ݼн��жϷ�������ȷ��ָ��
    flag = acos(ang);
    dis = norm(dir1);
    if flag<0
        dis = -dis;
    end
    
    %������ǰ��ı��淨����
    t=(0:dis/100:dis)';
    x = p_cur(1) + C(1)*t;
    y = p_cur(2) + C(2)*t;
    z = p_cur(3) + (-1)*t;
    
    normE =[normE;x y z];
%     i
end
% toc
pcshowpair(pc,pointCloud(normE));