function [p0,q0,feq,nv]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq,e_Delet_Distance,e_RANSAC_Distance)      %
% [p0,q0,feq]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq) ：剔除误匹配点
%      
%  输入参数  
%    P       ：模板点云 3 * n
%    Q       ：场景点云 3 * n
%    p0      ：模板点云的特征点集合      3 * n 
%    q0      ：场景点云的特征点集合      3 * n
%    fep     ：模板点云特征点的索引      1 * n
%    feq     ：场景点云特征点的索引      1 * n
%    feq0    ：场景点云特征点的索引备份  1 * n
%    vep     ：场景点云特征点的PFH描述   64 * n
%    veq     ：场景点云特征点的PFH描述   64 * n
%    e_Delet_Distance  : 剔除最远点的距离阈值
%    e_RANSAC_Distance : 随机采样一致性距离阈值
%
%  输出参数
%    p0      ：剔除误匹配后剩余正确匹配对应点,模板点云特征点 剔除后 3 * n
%    q0      ：剔除误匹配后剩余正确匹配对应点,场景点云特征点 剔除后 3 * n
%    feq     ：剔除误匹配后剩余正确匹配对应点,场景点云特征点 剔除后 1 * n
%    nv      ：PFH描述中的最近邻点索引 剔除后  1 * n
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
% e_Delet_Distance = 0.05;     %距离阈值
[feq,nv,e_Delet_Distance] = DeleteDisdence(P,Q,fep,feq,feq0,nv);

% %绘制 删除距离大于0.05后的图
% displayer.displayDeleteDisdencePointCloudAndLine(P,Q,p0,q0,fep,feq,nv);
displayer.displayDeleteDisdencePointCloudAndLine(P,Q,p0,q0,fep,feq,nv,e_Delet_Distance);

%% 2、刚性不变约束
[feq,nv] = RigidInvariantConstraints(P,Q,fep,feq,feq0,nv);

%出图，更新feq与nv后的关键点及其连线
displayer.displayRigidInvariantConstraintsPointCloudAndLine(P,Q,p0,q0,fep,feq,nv);


%% 3、使用随机抽样一致性算法RANSAC确定匹配关系
aa = 500;                       %抽样次数
% e_RANSAC_Distance = 0.005;    %随机采样一致性距离阈值
[feq,nv] = RANSAC(P,Q,fep,feq,feq0,aa,nv,e_RANSAC_Distance);

%RANSAC剔除误匹配后的结果绘制
displayer.displayRANSACPointCloudAndLine(P,Q,p0,q0,fep,feq,nv);


