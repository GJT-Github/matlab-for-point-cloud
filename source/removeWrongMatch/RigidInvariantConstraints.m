function [feq,nv] = RigidInvariantConstraints(P,Q,fep,feq,feq0,nv)
%% [p0,q0,feq,nv] = RigidInvariantConstraints(P,Q,fep,feq,feq0,nv)
%                   :   刚性不变约束
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



num = zeros(size(feq));                 %创建同feq大小的零矩阵，用来计数

for i = 1:length(nv)
    a =0;
    for j = 1:length(nv)
        if i == j
            continue                  
        end
        dq = norm(Q(:,feq(i))-Q(:,feq(j)));         %norm求Q中 第i个关键点与第j个关键点的 二范数  
        dp = norm(P(:,fep(nv(i)))-P(:,fep(nv(j)))); %norm求P中距离Q中第i个关键点最近点与距离Q中第j个关键点最近点的二范数
        if abs(dp-dq)/(dp+dq)<0.02        %abs取绝对值函数，判断(dp-dq)/(dp+dq)<0.02
            a = a + 1;                    %在第i个关键点下，满足条件的计数一次
        end
    end
    num(1,i) = a;
end

num1 = sort(num,'descend');              %sort排序 descend：按照降序排列

% feq(num<num1(10)) = [];                  %根据num<num1(10)条件删除满足条件的关键点指标
% nv(num<num1(10))  = [];                  %根据num<num1(10)条件删除满足条件的描述子指标

feq(num<num1(10)) = [];                  %根据num<num1(10)条件删除满足条件的关键点指标
nv(num<num1(10))  = [];                  %根据num<num1(10)条件删除满足条件的描述子指标

% p0 = P(:,fep);
% q0 = Q(:,feq0);

end
