function  displayer = displayFunction      %绘图封装
% display = displayFunction ：
%                             通过匿名函数调用多个函数
%
%  https://blog.csdn.net/weixin_39032619/article/details/109294078
%      
%  输入参数  
%      ：
%
%
%  输出参数
%      ：
%   
%  Author：GJT 
%  E-mail：gjt0114@outlook.com

displayer.displayInitPointCloud = @displayInitPointCloud;                         %读点云绘制
displayer.displayNormalOnSourcePointCloud = @displayNormalOnSourcePointCloud;     %法向量绘制

displayer.displayFirstPickKeyPoint = @displayFirstPickKeyPoint;                   %绘制粗提取的特征点
displayer.displayFinalPickKeyPoint = @displayFinalPickKeyPoint;                   %绘制最终提取的特征点

displayer.displayPFHOfKeyPoint = @displayPFHOfKeyPoint;                           % 绘制粗提取的特征点

displayer.displayPointCloudAndLine = @displayPointCloudAndLine;                   % 绘制匹配后的对应点
displayer.displayDeleteDisdencePointCloudAndLine = @displayDeleteDisdencePointCloudAndLine;  % 绘制剔除距离阈值后的对应点
displayer.displayRigidInvariantConstraintsPointCloudAndLine = @displayRigidInvariantConstraintsPointCloudAndLine;   % 绘制刚性不变约束剔除错误匹配后的对应点                        % 绘制粗提取的特征点
displayer.displayRANSACPointCloudAndLine = @displayRANSACPointCloudAndLine;       % 绘制RANSAC剔除错误匹配后的对应点

displayer.displayRigistration = @displayRigistration;                             % 绘制粗匹配结果


end

%% -----------------------------------------main----------------------------------------------------

%读点云绘制
function [] = displayInitPointCloud(P,Q)
    global axe;
    global posionFigureX;
    global posionFigureY;
    global posionFigureZ;
    global posionFigureN;
    posionFigureX = 10;
    posionFigureY = 350;
    posionFigureZ = 500;
	posionFigureN = 400;

	figure(1);                               %画读到的点云图
	% set(gcf,'position',[10 350 500 400]);
	set(gcf,'position',[posionFigureX,posionFigureY,posionFigureZ,posionFigureN]);
	axe(1)=subplot(221);
	plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot绘图函数，分别取P中第1.2.3行所有点作为坐标轴，r表示颜色
	hold on
	plot3(Q(1,:),Q(2,:),Q(3,:),'b.');
	title('模板点云与场景点云初始位置')
	view(3)
end


%法向量绘制
function [] = displayNormalOnSourcePointCloud(P,normal)
    global axe;
    global posionFigureX;
    global posionFigureY;
    global posionFigureZ;
    global posionFigureN;
	figure(2);
    set(gcf,'position',[posionFigureX + 510,posionFigureY,posionFigureZ,posionFigureN]);
	plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot绘图函数，分别取P中第1.2.3行所有点作为坐标轴，r表示颜色
	hold on
	quiver3( P(1,:) , P(2,:) , P(3,:)  ,  normal(1,:) , normal(2,:) , normal(3,:) ,'g');
	xlabel('x');ylabel('y');zlabel('z');
	title('源点云法向量显示');
end


%--------------------------------------------main_end-------------------------------------------------
%% --------------------------------------------featurePoint---------------------------------------------
%绘制粗提取的特征点
function [] = displayFirstPickKeyPoint(p0,q0)
    global axe;
    figure(1);                                 %显示p0 q0特征点，特征为该点法向量点成最近8个点的法向量绝对值累加的平均值小于所有平均值的平均
    axe(2)=subplot(222);
    plot3(p0(1,:),p0(2,:),p0(3,:),'r.');
    hold on
    plot3(q0(1,:),q0(2,:),q0(3,:),'b.');
    title('模板点云与目标点云的特征点粗提取')
    view(3)
    % daspect([1 1 1]);                      %三维角度出图
end



%绘制最终提取的特征点
function [] = displayFinalPickKeyPoint(p0,q0)
    global axe;
    figure(1);
    axe(3)=subplot(223);
    plot3(p0(1,:),p0(2,:),p0(3,:),'r.');
    title('目标点云关键点精提取')
    view(3)
    figure(1);
    axe(4)=subplot(224);
    plot3(q0(1,:),q0(2,:),q0(3,:),'b.');
    title('模板点云关键点精提取')
    view(3)
    linkaxes(axe,'xy')
end

%--------------------------------------------featurePoint_end------------------------------------------
%% --------------------------------------------PFHCaculate-----------------------------------------------



%绘制粗提取的特征点
function [] = displayPFHOfKeyPoint(vep)
    global axe;
    global posionFigureX;
    global posionFigureY;
    global posionFigureZ;
    global posionFigureN;
    figure(3);
    set(gcf,'position',[posionFigureX + 510*2,posionFigureY,posionFigureZ,posionFigureN]);
    axe(1)=subplot(231);
    bar(vep(:,144));                      %bar和bar3分别用来绘制二维和三维竖直方图，绘制vep中第144列描述子
    axis([0 64 0 1200])                   %axis([xmin xmax ymin ymax])设置当前坐标轴 x轴和y轴的范围
    title('第144个关键点的PFH特征描述子')
end

%--------------------------------------------PFHCaculate_end---------------------------------------------
%% --------------------------------------------removeWrongMatch------------------------------------------


%显示模型与场景特征点及其 最近点 连线
function [] = displayPointCloudAndLine(P,Q,p0,q0,fep,feq,nv)

    global axe;
    %p0=P(:,fep);
    %q0=Q(:,feq);

    % 绘制 目标点云 和 源点云
    figure(3);
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
    view(2)
end


%绘制 删除距离大于0.05后的图
function [] = displayDeleteDisdencePointCloudAndLine(P,Q,p0,q0,fep,feq,nv)

    global axe;

    figure(3);
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

end



%绘制刚性不变约束后的点连线
function [] = displayRigidInvariantConstraintsPointCloudAndLine(P,Q,p0,q0,fep,feq,nv)
    
    global axe;

    figure(3);
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
end


% 绘制RANSAC剔除错误匹配后的对应点
function displayRANSACPointCloudAndLine(P,Q,p0,q0,fep,feq,nv)

    global axe;

    figure(3);
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

end

%--------------------------------------------removeWrongMatch_end------------------------------------------

%--------------------------------------------displayRigistration------------------------------------------

%% 绘制配准结果
function []=displayRigistration(P,Q1)
    global axe;

    figure(3);
    axe(6)=subplot(236);
    plot3(P(1,:),P(2,:),P(3,:),'r.');

    hold on
    plot3(Q1(1,:),Q1(2,:),Q1(3,:),'b.');
    % hold on
    % plot3(Q(1,:),Q(2,:),Q(3,:),'g.');
    xlabel('x');ylabel('y');zlabel('z');
    title('四元数配准');
    view(2)
    linkaxes(axe(2:6),'xy')


end

%% --------------------------------------------displayRigistration_end------------------------------------------
