function [feq,nv,e_Delet_Distance] = DeleteDisdence(P,Q,fep,feq,feq0,nv)
%  [p0,q0,feq,nv] = DeleteDisdence(P,Q,fep,feq,feq0,nv)  : 剔除 距离过远的对应点
%
%  输入参数  
%    P       ：模板点云
%    Q       ：场景点云
%    fep     ：模板点云特征点的索引
%    feq     ：场景点云特征点的索引
%    feq0    ：场景点云特征点的索引备份
%    nv      : PFH描述中的最近邻点索引
%
%  输出参数
%    p0      ：模板点云的特征点集合 剔除后
%    q0      ：场景点云的特征点集合 剔除后
%    feq     ：场景点云特征点的索引 剔除后 
%    nv      : PFH描述中的最近邻点索引 剔除后
%   
%  Author：GJT 
%  E-mail：gjt0114@outlook.com



    dist = zeros(size(nv));                 %创建0矩阵，大小等于矩阵nv
    for i = 1:length(nv)
        dist(1,i) = norm( Q( :,feq(i) ) - P( :,fep( nv(1,i) ) ) );%二范数Q(:,feq(i))表示Q点云中的第i个关键点，P(:,fep(nv(1,i))表示P点云中距离Q中第i关键点最近点
    end

    e_Delet_Distance = 1.0*mean(dist);

    feq(dist>e_Delet_Distance) = [];                    %从feq中删除距离大于0.05的最近点（关键点）的指标
    nv(dist>e_Delet_Distance)  = [];                    %从nv中删除距离大于0.05的特征描述子的指标

%     p0 = P(:,fep);
%     q0 = Q(:,feq0);

%     %绘制 删除距离大于0.05后的图
%     displayer = displayFunction;
%     displayer.displayDeleteDisdencePointCloudAndLine(P,Q,p0,q0,fep,feq,nv,e_Delet_Distance);

end