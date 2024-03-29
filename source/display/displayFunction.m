function  displayer = displayFunction      %绘图封装
% display = displayFunction ：
%                             通过匿名函数调用多个函数
%
%  https://blog.csdn.net/weixin_39032619/article/details/109294078
%      
%  输入参数  
%    P     : 目标点云XYZ点数据, 3*n 的数据矩阵
%    Q     : 场景点云XYZ点数据，3*n 的数据矩阵
%
%
%  输出参数
%      ：
%   
%  Author：GJT 
%  E-mail：gjt0114@outlook.com

% warning off % 开启警告/显示警告

%% 点云读取相关显示
displayer.displayInitPointCloud = @displayInitPointCloud;                         %读点云绘制
displayer.displayNormalOnSourcePointCloud = @displayNormalOnSourcePointCloud;     %法向量绘制

%% 特征点提取相关显示
displayer.displayFirstPickKeyPoint = @displayFirstPickKeyPoint;                   %绘制粗提取的特征点
displayer.displayFinalPickKeyPoint = @displayFinalPickKeyPoint;                   %绘制最终提取的特征点

%% 特征点特征描述相关显示
displayer.displayPFHOfKeyPoint = @displayPFHOfKeyPoint;                           % 绘制特征点的PFH描述
displayer.displayFPFHOfKeyPoint = @displayFPFHOfKeyPoint;                         % 绘制特征点的FPFH描述

%% 特征匹配相关显示
displayer.displayPointCloudAndLine = @displayPointCloudAndLine;                   % 绘制匹配后的对应点
displayer.displayDeleteDisdencePointCloudAndLine = @displayDeleteDisdencePointCloudAndLine;  % 绘制剔除距离阈值后的对应点
displayer.displayRigidInvariantConstraintsPointCloudAndLine = @displayRigidInvariantConstraintsPointCloudAndLine;   % 绘制刚性不变约束剔除错误匹配后的对应点                        % 绘制粗提取的特征点
displayer.displayRANSACPointCloudAndLine = @displayRANSACPointCloudAndLine;       % 绘制RANSAC剔除错误匹配后的对应点

%% 粗配准相关显示
displayer.displayRigistration = @displayRigistration;                             % 绘制粗匹配结果

%% 精配准相关显示
displayer.displayProcessOfICP = @displayProcessOfICP;                             % 绘制ICP配准过程

%% 最终得到的旋转平移变换矩阵一次变换相关显示
displayer.displayFinalQ2P = @displayFinalQ2P;                                     % 绘制最终得到的旋转平移矩阵一次变换结果

global posionFigureX;
global posionFigureY;
global posionFigureZ;
global posionFigureN;
posionFigureX = 10;
posionFigureY = 350;
posionFigureZ = 500;
posionFigureN = 400;

global size_Point;
size_Point = 6;
global num_Figure;
num_Figure=1;



end


%% -----------------------------------------main----------------------------------------------------

%读点云绘制
function [] = displayInitPointCloud(P,Q)
    global axe;
    global posionFigureX;
    global posionFigureY;
    global posionFigureZ;
    global posionFigureN;
    global size_Point;
    global num_Figure;

	figure(num_Figure);                               %画读到的点云图
	% set(gcf,'position',[10 350 500 400]);
	set(gcf,'position',[posionFigureX,posionFigureY,posionFigureZ,posionFigureN]);
	axe(1)=subplot(221);
	plot3(P(1,:),P(2,:),P(3,:),'r.','markersize',size_Point);        %plot绘图函数，分别取P中第1.2.3行所有点作为坐标轴，r表示颜色
	hold on
	plot3(Q(1,:),Q(2,:),Q(3,:),'b.','markersize',size_Point);
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
    set(gcf,'position',[posionFigureX + (posionFigureZ+10)*1,posionFigureY,posionFigureZ,posionFigureN]);
	plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot绘图函数，分别取P中第1.2.3行所有点作为坐标轴，r表示颜色
	hold on
	quiver3( P(1,:) , P(2,:) , P(3,:)  ,  normal(1,:) , normal(2,:) , normal(3,:) ,'g');
    plot3(0,0,0,'b*');
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
    view(2)
    % daspect([1 1 1]);                      %三维角度出图
end



%绘制最终提取的特征点
function [] = displayFinalPickKeyPoint(p0,q0)
    global axe;
    global size_Point;

    figure(1);
    % colordef black; % 2D/3D图背景黑色
    axe(3)=subplot(223);
    plot3(p0(1,:),p0(2,:),p0(3,:),'r.','markersize',size_Point);
    title('目标点云关键点精提取')

    aplha=0:pi/40:2*pi;
    global r_P_k r_Q_k;

    % % 绘制集合球
    % for i=1:3:20 

    %     % r=2;        
    %     x=r_P_k *cos(aplha) + p0(1,i);
    %     y=r_P_k *sin(aplha) + p0(2,i);
    %     hold on
    %     % plot3(x,y,repmat(int32(p0(3,1)),size(x),1),'b-');
    %     plot3(x,y,repelem(p0(3,i),size(x,2)),'b-')
    %     plot3(p0(1,i),p0(2,i),p0(3,i),'b*');

    %     x=r_P_k *cos(aplha) + p0(1,i);
    %     z=r_P_k *sin(aplha) + p0(3,i);
    %     hold on
    %     % plot3(x,y,repmat(int32(p0(3,1)),size(x),1),'b-');
    %     plot3(x,repelem(p0(2,i),size(x,2)),z,'b-')

    %     y=r_P_k *cos(aplha) + p0(2,i);
    %     z=r_P_k *sin(aplha) + p0(3,i);
    %     hold on
    %     % plot3(x,y,repmat(int32(p0(3,1)),size(x),1),'b-');
    %     plot3(repelem(p0(1,i),size(x,2)),y,z,'b-')
    %     hold off
        
    % end

    %     axis equal
        view(2)

        % figure(1);
        axe(4)=subplot(224);
        plot3(q0(1,:),q0(2,:),q0(3,:),'b.','markersize',size_Point);
        title('模板点云关键点精提取')
        view(2)
        warning off % 开启警告/显示警告
        linkaxes(axe,'xy')
        warning on % 开启警告/显示警告
    
end

%--------------------------------------------featurePoint_end------------------------------------------
%% --------------------------------------------PFHCaculate-----------------------------------------------



%绘制特征点的PFH描述
function [] = displayPFHOfKeyPoint(vep)
    global axe;
    global posionFigureX;
    global posionFigureY;
    global posionFigureZ;
    global posionFigureN;
    figure(3);
    set(gcf,'position',[posionFigureX + (posionFigureZ+10)*2,posionFigureY,posionFigureZ,posionFigureN]);
    axe(1)=subplot(231);
    bar(vep(:,144));                      %bar和bar3分别用来绘制二维和三维竖直方图，绘制vep中第144列描述子
    axis([0 64 0 1200])                   %axis([xmin xmax ymin ymax])设置当前坐标轴 x轴和y轴的范围
    title('第144个关键点的PFH特征描述子')
end


%绘制特征点的FPFH描述
function [] = displayFPFHOfKeyPoint(vep)
    global axe;
    global posionFigureX;
    global posionFigureY;
    global posionFigureZ;
    global posionFigureN;
    figure(3);
%     set(gcf,'position',[posionFigureX + (posionFigureZ+10)*2,posionFigureY,posionFigureZ,posionFigureN]);
    axe(1)=subplot(231);
    bar(vep(:,144));                      %bar和bar3分别用来绘制二维和三维竖直方图，绘制vep中第144列描述子
    axis([0 64 0 1200])                   %axis([xmin xmax ymin ymax])设置当前坐标轴 x轴和y轴的范围
    title('第144个关键点的FPFH特征描述子')
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
function [] = displayDeleteDisdencePointCloudAndLine(P,Q,p0,q0,fep,feq,nv,e_dis)

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
    title(['去除距离大于',num2str(e_dis),'的最近点连线']);
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
    view(3)
    warning off % 开启警告/显示警告
    linkaxes(axe(2:6),'xy')
    warning on % 开启警告/显示警告


end

%% --------------------------------------------displayRigistration_end------------------------------------------
%% --------------------------------------------ICP--------------------------------------------------------------
function [] = displayProcessOfICP(data_source,data_target,iteration,num_Figure)
    %
    % data_source  ：3 * n
    % data_target  ：3 * n
    %
        if iteration == 1
            % h = figure( 'position' , [ left + width + 10 * 1 , bottom , width , hight ] );
            figure(num_Figure);
        end
        scatter3( data_source( 1 , : ) , data_source( 2 , : ) , data_source( 3 , : ) , 'b.' );
        hold on;
        scatter3( data_target( 1 , : ) , data_target( 2 , : ) , data_target( 3 , : ) , 'r.' );
        hold off;
        title( '配准过程展示' )
        xlabel( 'x' );
        ylabel( 'y' );
        zlabel( 'z' );
        grid on;
        legend( 'source-file_1(模板)' , 'target-file_2' )
        daspect( [1 1 1] );
        pause( 0.1 );
        drawnow

end




%% --------------------------------------------ICP_end----------------------------------------------------------
%% --------------------------------------------Final_end--------------------------------------------------------
function [] = displayFinalQ2P(data_source,data_target)
    %
    % data_source  ：3 * n
    % data_target  ：3 * n
    %

        figure;
        scatter3( data_source( 1 , : ) , data_source( 2 , : ) , data_source( 3 , : ) , 'b.' );
        hold on;
        scatter3( data_target( 1 , : ) , data_target( 2 , : ) , data_target( 3 , : ) , 'r.' );
        hold off;
        title( '最终得到的旋转平移矩阵一次变换展示' )
        xlabel( 'x' );
        ylabel( 'y' );
        zlabel( 'z' );
        grid on;
        legend( 'source-file_1(模板)' , 'target-file_2' )
        daspect( [1 1 1] );
        drawnow

end


%% --------------------------------------------Final_end--------------------------------------------------------

