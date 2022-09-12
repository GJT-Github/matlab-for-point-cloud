

function main   %https://blog.csdn.net/weixin_37610397/article/details/80441523

clc
clear
close all

%global axe;

addpath(genpath('../source/'))   %addpath 是添加SGDLibrary-master目录   genpath 是读取SGDLibrary-master目录所有子目录
%% 读取文件

file1='../Datas/bun045.asc';
file2='../Datas/bun000.asc';

% file1='../Datas/rabbit.pcd';
% file2='../Datas/rabbit.pcd';

% file1='../Datas/rabbit.ply';
% file2='../Datas/Scene3.ply';

tic         %计时开始

[P,Q]= readPointCloudDatas(file1,file2);

toc         %计时结束，自动打印运行的时间
% t=toc   

%绘制读到的点云
% displayInitPointCloud(P,Q);
displayer = displayFunction;
displayer.displayInitPointCloud(P,Q);


%% 通过8邻域PCA建立 法向量估计 [pn、qn]
% PCA法向量估计：某点邻域协方差矩阵最小特征值对应特征向量即该点法向量
k=8;                                    %邻域选定：8邻域

pn = lsqnormest(P, k);                  %调用求法向量子函数lsqnormest求P阵所有法向量
qn = lsqnormest(Q, k);

%绘制目标点云的法向量
displayer.displayNormalOnSourcePointCloud(P,pn);


%% 特征点提取   demo_1
[p0,q0,fep,feq,feq0,n1,d1,n2,d2] = featurePoint(P,Q,pn,qn,k);

%% PFH特征计算/描述  demo_2
[vep,veq] = PFHCaculate(P,Q,p0,q0,fep,feq,pn,qn,n1,d1,n2,d2);
save main.mat
%% demo_3
%% 误匹配剔除
% load main.mat
[p0,q0,feq,nv] = removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq);

%save main.mat

%% 均方根评价
%load main.mat
RMSE(p0,q0)


%% 特征匹配/配准
coarseRegistration(P,Q,fep,feq,nv);


%% 精配准
fineRegistration();


end








%% 绘图封装
%读点云绘制
% function [] = displayInitPointCloud(P,Q)
%     global axe;
%     global posionFigureX;
%     global posionFigureY;
%     global posionFigureZ;
%     global posionFigureN;
%     posionFigureX = 10;
%     posionFigureY = 350;
%     posionFigureZ = 500;
% 	posionFigureN = 400;

% 	figure(1);                               %画读到的点云图
% 	% set(gcf,'position',[10 350 500 400]);
% 	set(gcf,'position',[posionFigureX,posionFigureY,posionFigureZ,posionFigureN]);
% 	axe(1)=subplot(221);
% 	plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot绘图函数，分别取P中第1.2.3行所有点作为坐标轴，r表示颜色
% 	hold on
% 	plot3(Q(1,:),Q(2,:),Q(3,:),'b.');
% 	title('模板点云与场景点云初始位置')
% 	view(3)
% end

% %法向量绘制
% function [] = displayNormalOnSourcePointCloud(P,normal)
%     global axe;
%     global posionFigureX;
%     global posionFigureY;
%     global posionFigureZ;
%     global posionFigureN;
% 	figure(2);
%     set(gcf,'position',[posionFigureX + 510,posionFigureY,posionFigureZ,posionFigureN]);
% 	plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot绘图函数，分别取P中第1.2.3行所有点作为坐标轴，r表示颜色
% 	hold on
% 	quiver3( P(1,:) , P(2,:) , P(3,:)  ,  normal(1,:) , normal(2,:) , normal(3,:) ,'g');
% 	xlabel('x');ylabel('y');zlabel('z');
% 	title('源点云法向量显示');
% end
