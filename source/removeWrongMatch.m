function [p0,q0,feq,nv]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq)      %
% [p0,q0,feq]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq) ：剔除误匹配点
%      
%  输入参数  
%      ：
%
%
%  输出参数
%      ：
%   


% load PFH2.mat

%根据直方图 建立特征点匹配关系 并建 立刚性不变约束
[nv,d]=knnsearch(vep',veq');           %vep',veq'取对应矩阵共轭转置，knnsearch(X, Y) 在向量集合X中找到分别与向量集合Y 每个行向量 最近的 邻居索引nv，距离d
nv=nv';                                %再转置
d=d';

%% 显示模型与场景特征点及其 最近点 连线
%p0=P(:,fep);
%q0=Q(:,feq);

% 绘制 目标点云 和 源点云
%figure;
axe(2)=subplot(232);
plot3(p0(1,:),p0(2,:),p0(3,:),'r.');
hold on
plot3(q0(1,:),q0(2,:),q0(3,:),'b.');

%绘制最近邻点间连线
for i=1:length(nv)                    
    hold on
    x=[Q(1,feq(i)),P(1,fep(nv(1,i)))];%直线的起点x坐标与终点x坐标，注意起点为场景中第一点，终点为对应其在模型中最近点
    y=[Q(2,feq(i)),P(2,fep(nv(1,i)))];
    z=[Q(3,feq(i)),P(3,fep(nv(1,i)))];
    plot3(x,y,z,'g-');
    %pause(0.02)
end
xlabel('x');ylabel('y');zlabel('z');
title('源点云和目标点云最近点连线');
view(3)



%% 剔除 距离过远的对应点
dist=zeros(size(nv));                 %创建0矩阵，大小等于矩阵nv
for i=1:length(nv)
    dist(1,i)=norm(Q(:,feq(i))-P(:,fep(nv(1,i))));%二范数Q(:,feq(i))表示Q点云中的第i个关键点，P(:,fep(nv(1,i))表示P点云中距离Q中第i关键点最近点
end

feq(dist>0.05)=[];                    %从feq中删除距离大于0.05的最近点（关键点）的指标
nv(dist>0.05)=[];                     %从nv中删除距离大于0.05的特征描述子的指标


%绘制 删除距离大于0.05后的图
p0=P(:,fep);
q0=Q(:,feq0);

%figure;
axe(3)=subplot(233);
plot3(p0(1,:),p0(2,:),p0(3,:),'r.');  %原P中的关键点
hold on
plot3(q0(1,:),q0(2,:),q0(3,:),'b.');  %原Q中的关键点
for i=1:length(nv)
    hold on
    x=[Q(1,feq(i)),P(1,fep(nv(1,i)))];%更新关键点指标与特征描述子指标的连线
    y=[Q(2,feq(i)),P(2,fep(nv(1,i)))];
    z=[Q(3,feq(i)),P(3,fep(nv(1,i)))];
    plot3(x,y,z,'g-');
end
xlabel('x');ylabel('y');zlabel('z');
title('去除距离大于0.05的最近点连线');
view(3)



%% 刚性不变约束
num=zeros(size(feq));                 %创建同feq大小的零矩阵，用来计数
for i=1:length(nv)
    a=0;
    for j=1:length(nv)
        if i==j
            continue                  %跳过语句，直接开始下一次循环
        end
        dq=norm(Q(:,feq(i))-Q(:,feq(j)));         %norm求Q中 第i个关键点与第j个关键点的 二范数  
        dp=norm(P(:,fep(nv(i)))-P(:,fep(nv(j)))); %norm求P中距离Q中第i个关键点最近点与距离Q中第j个关键点最近点的二范数
        if abs(dp-dq)/(dp+dq)<0.02    %abs取绝对值函数，判断(dp-dq)/(dp+dq)<0.02
            a=a+1;                    %在第i个关键点下，满足条件的计数一次
        end
    end
    num(1,i)=a;
end
num1=sort(num,'descend');             %sort排序 descend：按照降序排列
feq(num<num1(10))=[];                 %根据num<num1(10)条件删除满足条件的关键点指标
nv(num<num1(10))=[];                  %根据num<num1(10)条件删除满足条件的描述子指标

%出图，更新feq与nv后的关键点及其连线
p0=P(:,fep);
q0=Q(:,feq0);

%figure;
axe(4)=subplot(234);
plot3(p0(1,:),p0(2,:),p0(3,:),'r.');  %原P中的关键点
hold on
plot3(q0(1,:),q0(2,:),q0(3,:),'b.');  %原Q中的关键点
for i=1:length(nv)
    hold on
    x=[Q(1,feq(i)),P(1,fep(nv(1,i)))];%更新关键点指标与特征描述子指标的连线
    y=[Q(2,feq(i)),P(2,fep(nv(1,i)))];
    z=[Q(3,feq(i)),P(3,fep(nv(1,i)))];
    plot3(x,y,z,'g-');
end
xlabel('x');ylabel('y');zlabel('z');
title('满足刚性不变约束条件的点连线');
view(3)

%使用随机抽样一致性算法RANSAC确定匹配关系
aa=500;                               %抽样次数
b0=0;
c0=zeros(size(feq));
while aa
    n=length(nv);
    a=randperm(n);
    feq1=feq(a(1:3));
    nv1=nv(a(1:3));
    
    [R,T] = Quater_Registration(Q(:,feq1)',P(:,fep(nv1))');
    Q0 = R * Q(:,feq) + repmat(T,1,n);
    
    dist=Q0-P(:,fep(nv));
    b=0;
    c=zeros(1,n);
    for i=1:n
%         if norm(dist(:,i))<0.0012
        if norm(dist(:,i))<0.005
            b=b+1;
            c(i)=1;
        end
    end
    if b>b0
        b0=b;
        c0=c;
    end
    aa=aa-1;
end
feq(c0<1)=[];
nv(c0<1)=[];

p0=P(:,fep);
q0=Q(:,feq0);

%figure;
axe(5)=subplot(235);
plot3(p0(1,:),p0(2,:),p0(3,:),'r.');
hold on
plot3(q0(1,:),q0(2,:),q0(3,:),'b.');
for i=1:length(nv)
    hold on
    x=[Q(1,feq(i)),P(1,fep(nv(1,i)))];
    y=[Q(2,feq(i)),P(2,fep(nv(1,i)))];
    z=[Q(3,feq(i)),P(3,fep(nv(1,i)))];
    plot3(x,y,z,'g-');
end
xlabel('x');ylabel('y');zlabel('z');
title('RANSAC剔除误匹配连线');
view(3)
