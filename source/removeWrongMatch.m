function [p0,q0,feq,nv]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq)      %
% [p0,q0,feq]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq) ：剔除误匹配点
%      
%  输入参数  
%    P       ：模板点云
%    Q       ：场景点云
%    p0      ：模板点云的特征点集合
%    q0      ：场景点云的特征点集合
%    fep     ：模板点云特征点的索引
%    feq     ：场景点云特征点的索引
%    feq0    ：场景点云特征点的索引备份
%    vep     ：场景点云特征点的PFH描述
%    veq     ：场景点云特征点的PFH描述
%  
%
%  输出参数
%    p0      ：剔除误匹配后剩余正确匹配对应点,模板点云特征点 剔除后 
%    q0      ：剔除误匹配后剩余正确匹配对应点,场景点云特征点 剔除后
%    feq     ：剔除误匹配后剩余正确匹配对应点,场景点云特征点 剔除后
%    nv      ：PFH描述中的最近邻点索引 剔除后
%
%  Author：GJT 
%  E-mail：gjt0114@outlook.com


% load PFH2.mat


%% 根据直方图 建立特征点匹配关系 并建 立刚性不变约束
[nv,d]=knnsearch(vep',veq');           %vep',veq'取对应矩阵共轭转置，knnsearch(X, Y) 在向量集合X中找到分别与向量集合Y 每个行向量 最近的 邻居索引nv，距离d
nv=nv';                                %再转置
d=d';

%% 显示模型与场景特征点及其 最近点 连线
displayer = displayFunction;
displayer.displayPointCloudAndLine(P,Q,p0,q0,fep,feq,nv);


%% 1、剔除 距离过远的对应点
[p0,q0,feq,nv] = DeleteDisdence(P,Q,fep,feq,feq0,nv);

%绘制 删除距离大于0.05后的图
displayer.displayDeleteDisdencePointCloudAndLine(P,Q,p0,q0,fep,feq,nv);


%% 2、刚性不变约束
[p0,q0,feq,nv] = RigidInvariantConstraints(P,Q,fep,feq,feq0,nv);

%出图，更新feq与nv后的关键点及其连线
displayer.displayRigidInvariantConstraintsPointCloudAndLine(P,Q,p0,q0,fep,feq,nv);


%% 3、使用随机抽样一致性算法RANSAC确定匹配关系
aa = 500;    %抽样次数
[p0,q0,feq,nv] = RANSAC(P,Q,fep,feq,feq0,aa,nv);

%RANSAC剔除误匹配后的结果绘制
displayer.displayRANSACPointCloudAndLine(P,Q,p0,q0,fep,feq,nv);


