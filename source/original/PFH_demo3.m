clc
clear
close all
load PFH2.mat
%% 识别
figure;
axe(1)=subplot(231);
bar(vep(:,144));                      %bar和bar3分别用来绘制二维和三维竖直方图，绘制vep中第144列描述子
axis([0 64 0 1200])                   %axis([xmin xmax ymin ymax])设置当前坐标轴 x轴和y轴的范围

title('第144个关键点的PFH特征描述子')

%根据直方图建立特征点匹配关系并建立刚性不变约束
[nv,d]=knnsearch(vep',veq');           %vep',veq'取对应矩阵共轭转置，knnsearch(X, Y) 在向量集合X中找到分别与向量集合Y 每个行向量 最近的 邻居索引nv，距离d
nv=nv';                                %再转置
d=d';
%显示模型与场景特征点及其最近点连线
p0=P(:,fep);
q0=Q(:,feq);

%figure;
axe(2)=subplot(232);
plot3(p0(1,:),p0(2,:),p0(3,:),'r.');
hold on
plot3(q0(1,:),q0(2,:),q0(3,:),'b.');


for i=1:length(nv)                    %根据最近邻点连接直线
    hold on
    x=[Q(1,feq(i)),P(1,fep(nv(1,i)))];%直线的起点x坐标与终点x坐标，注意起点为场景中第一点，终点为对应其在模型中最近点
    y=[Q(2,feq(i)),P(2,fep(nv(1,i)))];
    z=[Q(3,feq(i)),P(3,fep(nv(1,i)))];
    plot3(x,y,z,'g-');
    %pause(0.02)
end
xlabel('x');ylabel('y');zlabel('z');
title('源点云和目标点云最近点连线');
view(2)

dist=zeros(size(nv));                 %创建0矩阵，大小等于矩阵nv
for i=1:length(nv)
    dist(1,i)=norm(Q(:,feq(i))-P(:,fep(nv(1,i))));%二范数Q(:,feq(i))表示Q点云中的第i个关键点，P(:,fep(nv(1,i))表示P点云中距离Q中第i关键点最近点
end
%删除距离过远的对应点
feq(dist>0.05)=[];                    %从feq中删除距离大于0.05的最近点（关键点）的指标
nv(dist>0.05)=[];                     %从nv中删除距离大于0.05的特征描述子的指标

%出图，删除距离大于0.05后的图
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
view(2)

%刚性不变约束
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
view(2)

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
view(2)

%四元数法配准
[R,T] = Quater_Registration(Q(:,feq)',P(:,fep(nv))');%调用子函数Quater_Registration配准算法，输入为Q矩阵P矩阵匹配的初值，输出为旋转矩阵R和平移向量T
Q1    = R * Q + repmat(T,1,size(Q,2));

%figure;
axe(6)=subplot(236);
plot3(P(1,:),P(2,:),P(3,:),'r.');

hold on
plot3(Q1(1,:),Q1(2,:),Q1(3,:),'b.');
% hold on
% plot3(Q(1,:),Q(2,:),Q(3,:),'g.');
disp('正确匹配点对的数量为：')
disp(length(nv));
xlabel('x');ylabel('y');zlabel('z');
title('四元数配准');
view(2)
linkaxes(axe(2:6),'xy')
clear axe a aa b b0 c c0 i j n num R T x y z dp dq
