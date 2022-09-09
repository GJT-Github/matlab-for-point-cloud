clc
clear
close all


addpath(genpath('../source/'))   %addpath 是添加SGDLibrary-master目录   genpath 是读取SGDLibrary-master目录所有子目录
%% 读取文件

file1='../Datas/bun045.asc';
file2='../Datas/bun000.asc';

data1 = ascread(file1);      %读取asc类型点云，值为2行1列的cell矩阵（俩行可以不同类型）{1}为点数40097points，{2}为3行40097列坐标矩阵
data2 = ascread(file2);      %读取asc类型点云，值为2行1列的cell矩阵（俩行可以不同类型）{1}为点数40097points，{2}为3行40256列坐标矩阵
 
P = data1{2};                %P为3行40097列矩阵，等于data1矩阵（cell）第二行
Q = data2{2};


% 绘制读到的点云
figure;                                  %画读到的点云图
axe(1)=subplot(221);
plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot绘图函数，分别取P中第1.2.3行所有点作为坐标轴，r表示颜色
hold on
plot3(Q(1,:),Q(2,:),Q(3,:),'b.');
title('模板点云与场景点云初始位置')
view(3)

%% 通过8邻域PCA建立 法向量估计 [pn、qn]
k=8;                                    %邻域选定：8邻域

pn = lsqnormest(P, k);                  %调用求法向量子函数lsqnormest求P阵所有法向量
qn = lsqnormest(Q, k);


%% 特征点提取   demo_1
[p0,q0,fep,feq,feq0,n1,d1,n2,d2] = featurePoint(P,Q,pn,qn,k);

%% PFH特征计算/描述  demo_2
[vep,veq] = PFHCaculate(P,Q,p0,q0,fep,feq,pn,qn,n1,d1,n2,d2);

%% demo_3
%%误匹配剔除
[p0,q0,feq,nv]=removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq);

save main.mat

%% 均方根评价
load main.mat
RMSE(p0,q0)


%%特征匹配/配准
registration(P,Q,fep,feq,nv);


