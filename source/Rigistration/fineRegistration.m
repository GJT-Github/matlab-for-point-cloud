function [R_Final,T_Final]= fineRegistration(data_target,data_source)      %
% []= fineRegistration(data_target,data_source) ：ICP实现
%      
%  输入参数  
%   data_target   ：粗配准后的目标点云 3 * n
%   data_source   ：粗配准后的源点云   3 * n
%
%
%  输出参数
%   R_Final            ：粗配准后旋转矩阵   3 * 3
%   T_Final            ：粗配准后平移矩阵   3 * 1
%   
%  Author：GJT 
%  E-mail：gjt0114@outlook.com



% *****************ICP实现*****************
%https://blog.csdn.net/Darlingqiang/article/details/108441503
%

%% 参数配置
    max_iteration    = 100;                       % 迭代最大次数
    Tolerance        = 5.0e-30;                   % 误差
    step_Tolerance   = 1.0e-14;                   % 迭代容差步长阈值


    inlier_ratio     = 1;                     % 0.999 内点判定比例（欧式距离为判定条件）
%     B_baoHeDu        = 2.5;                       % M估计中饱和度B
  
    kd               = 1;                         % 对应点搜索选择,0：欧式距离寻找对应点 1：KD-tree寻找对应点
  
    inlier_flag      = 1;                         % 剔除外点控制，1：剔除 0：不剔除
    show_flag        = 1;                         % 配准过程显示控制，1：显示；0：不显示

    iteration        = 0;                         %初始化迭代次数

    num_figure = 4;

    % 初始旋转平移矩阵参数设定
    T_final   =  eye( 4 , 4 );                %旋转矩阵粗配准后旋转平移矩阵 eye:单位阵  
    Rf        =  T_final( 1:3 , 1:3 );        %提取粗配准最终旋转矩阵       Rf初值：单位阵
    Tf        =  T_final( 1:3 ,4 );           %提取粗配准最终平移矩阵       Tf初值：零向量

% data_source     = Rf * data_source + Tf * ones( 1 , size( data_source , 2) );      %初次更新点集（代表粗配准结果）,加入粗配准后注释掉

    data_source_old = data_source;               %中间变换存放
    t_a = 0;
    displayer = displayFunction;                 %$显示迭代过程

%迭代前显示 debug
% displayer = displayFunction;
% displayer.displayRigistration(data_target,data_source);


%% 迭代优化
% tic;
while(1)
    
  iteration = iteration + 1;

    % 找出对应点集
    if kd == 1
        %利用Kd-tree找出对应点集
        kd_tree = KDTreeSearcher( data_target' , 'BucketSize' , 10 );
        [index, dist] = knnsearch( kd_tree , data_source' );
        tic
    else
        %利用欧式距离找出对应点集
        k=size(data_source,2);
        for i = 1:k
            data_q1( 1 , : ) = data_target( 1 , : ) - data_source( 1 , i );    % 两个点集中的点x坐标之差
            data_q1( 2 , : ) = data_target( 2 , : ) - data_source( 2 , i );    % 两个点集中的点y坐标之差
            data_q1( 3 , : ) = data_target( 3 , : ) - data_source( 3 , i );    % 两个点集中的点z坐标之差
            distance = sqrt( data_q1( 1 , : ).^2 + data_q1( 2 , : ).^2 + data_q1( 3 , : ).^2 );  % 欧氏距离
            [dist(i), index(i)] = min( distance );   % 找到距离最小的那个点
        end
        clear k

    end   


%     %迭代误差记录
%     disp( [ '误差err=' , num2str( mean( dist ) ) ] );
%     disp( ['迭代次数iteration=' , num2str( iteration ) ] );
%     err_rec( iteration ) = mean( dist );
%     err = min( err_rec );
%     rmse= sqrt(err);


    %剔除外点
    if inlier_flag == 1
      % 按距离排序，只取前面占比为inlierratio内的点以应对外点
        [~, idx]   = sort( dist ); 
        inlier_num = round( size( data_source , 2 ) * inlier_ratio );
        idx        = idx( 1 : inlier_num );
        data_source_temp = data_source( : , idx );
        dist       = dist( idx );
        index      = index( idx );
        data_mid   = data_target( : , index ); 
    %不剔除外点
    else      
%         [~, idx]   = sort( dist );  
%         index      = index( idx );
        inlier_num = size( data_source , 2 ) ;
        
        data_source_temp = data_source;
        data_mid         = data_target(:,index);  
    end

    %迭代误差记录
    dist=mink(dist,inlier_num);
    err_rec( iteration ) = mean( dist );
    err = min( err_rec );
    % rmse= sqrt(err);

    % disp( [ '误差MSE=' , num2str( mean( dist ) ) ] );
    % disp( [ '误差RMSE=' , num2str( sqrt( mean( dist.^2 ) ) ) ] );
    % disp( ['迭代次数iteration=' , num2str( iteration ) ] );


%计算旋转平移矩阵前显示 debug
% close all
% displayer.displayRigistration(data_target,data_source);


    % 单元四元素求解 旋转矩阵R 与 平移向量T    data_source -> data_mid
    % [R_new,t_new] = Quater_Registration(data_source', data_target(:,index)');
%     [R_new,t_new] = Quater_Registration(data_source_temp', data_mid');

    % 去中心化后SVD分解求解旋转矩阵与平移向量   data_source -> data_mid
    [R_new, t_new] = rigidTransform3D( data_source_temp' ,  data_mid' );



    % 计算累计的旋转矩阵与平移向量
    Rf = R_new * Rf;
    Tf = R_new * Tf + t_new;
    
    % 更新点集
    data_source = Rf * data_source_old + Tf * ones( 1 , size( data_source_old , 2 ) );

%应用旋转变换矩阵后显示 debug
% close all
% displayer.displayRigistration(data_target,data_source);

    
    %%———————————————————————————论文中 M-估计引入目标函数————————————————————————————————————————————————————
      
    % % k=size(data_source,2);
    % % for i = 1:k                    
    %     data_q1( 1 , : ) = data_target( 1 , index ) - data_source( 1 , idx );    % 两个对应点集中的点x坐标之差
    %     data_q1( 2 , : ) = data_target( 2 , index ) - data_source( 2 , idx );    % 两个对应点集中的点y坐标之差
    %     data_q1( 3 , : ) = data_target( 3 , index ) - data_source( 3 , idx );    % 两个对应点集中的点z坐标之差

    %     f(1,:) = F_M_estition( data_q1( 1 , : ) , B_baoHeDu );                  
    %     f(2,:) = F_M_estition( data_q1( 2 , : ) , B_baoHeDu );
    %     f(3,:) = F_M_estition( data_q1( 3 , : ) , B_baoHeDu );
    % % end
    % f_value_m_estition = mean( sum( f ));
    % % err_rec( iteration ) = f_value_m_estition;
    % % err = min( err_rec );
    % err = f_value_m_estition;
    % % clear f

   %%————————————————————————————————————————————————————————————————————————————————————————————————————————— 

     


   
    % 收敛判别 目标函数误差收敛于阈值，则终止迭代
        %误差达到阈值
        if err < Tolerance 
            disp( '――――――――――――――――――――――――――――' );
            disp( '情况1：优化结果已经达到目标，结束优化' );
            break
        end

        %局部最优
        if iteration > 1 &&  abs(err_rec(iteration-1) - err_rec(iteration))< step_Tolerance 
            disp( '――――――――――――――――――――――――――――' );
            disp( '情况2：前后迭代误差减少量小于阈值，局部最优，结束优化' );
            disp('上一次迭代误差：');
            disp(num2str(err_rec(iteration-1)));
            disp('本次迭代误差：');
            disp(num2str(err_rec(iteration)));
            disp('两次次迭代误差差值绝对值：');
            disp(num2str(abs(err_rec(iteration-1) - err_rec(iteration))));
            break
        end

        %迭代次数达阈值
        if iteration >= max_iteration
            disp( '――――――――――――――――――――――――――――' );
            disp( '情况3：迭代次数达到阈值，结束优化' );
            break
        end

      time_record(iteration) = toc;
      t_a(iteration+1) = t_a(iteration) + time_record(iteration);

    % 显示中间结果（配准过程）
    if show_flag == 1
        % if iteration == 1
        %     % h = figure( 'position' , [ left + width + 10 * 1 , bottom , width , hight ] );
        %     figure(num_figure);
        % end
        % scatter3( data_source( 1 , : ) , data_source( 2 , : ) , data_source( 3 , : ) , 'b.' );
        % hold on;
        % scatter3( data_target( 1 , : ) , data_target( 2 , : ) , data_target( 3 , : ) , 'r.' );
        % hold off;
        % title( '配准过程展示' )
        % xlabel( 'x' );
        % ylabel( 'y' );
        % zlabel( 'z' );
        % grid on;
        % legend( 'source-file_1(模板)' , 'target-file_2' )
        % daspect( [1 1 1] );
        % pause( 0.1 );
        % drawnow


        displayer.displayProcessOfICP(data_source,data_target,iteration,num_figure);    %$

        disp( [ '误差 MSE  = '          , num2str( mean( dist.^2 ) ) ] );
        disp( [ '误差 RMSE = '          , num2str( sqrt( mean( dist.^2 ) ) ) ] );
        disp( [ '迭代次数 iteration = ' , num2str( iteration ) ] );
        disp( [ '排序耗时：'            , num2str(time_record(iteration)) ,' s '] );

    end
      
end

R_Final = Rf;
T_Final = Tf;

% figure('position' , [ left  , bottom - hight , width , hight ]);
figure(num_figure + 1);
plot(time_record,'b-')
hold on
line([0,iteration],[mean(time_record),mean(time_record)],'color','r')
% line([0,iteration],[t_a(1),t_a(iteration)],'color','g')
% legend('time of one time iteration','average of all time iteration','up time')
legend('time of one time iteration','average of all time iteration')

% time_end = toc;



% 计算最后结果的误差
if kd == 1
    %利用Kd-tree找出对应点集
    kd_tree = KDTreeSearcher( data_target' , 'BucketSize' , 10 );
    [ index , dist ] = knnsearch(kd_tree, data_source');
%     [n1,d1] = knnsearch(transpose(P), transpose(P), 'k', 400);           % demo1中k近邻查找
else
    %利用欧式距离找出对应点集
    k = size( data_source , 2 );
    for i = 1 : k
        data_q1( 1 , : ) = data_target( 1 , : ) - data_source( 1 , i );    % 两个点集中的点x坐标之差
        data_q1( 2 , : ) = data_target( 2 , : ) - data_source( 2 , i );    % 两个点集中的点y坐标之差
        data_q1( 3 , : ) = data_target( 3 , : ) - data_source( 3 , i );    % 两个点集中的点z坐标之差
        distance = sqrt( data_q1( 1 , : ).^2 + data_q1( 2 , : ).^2 + data_q1( 3 , : ).^2 );  % 欧氏距离
        [ dist( i ) , index( i ) ] = min( distance );   % 找到距离最小的那个点
    end
    clear k
end

dist = mink(dist,inlier_num);
err_rec( iteration + 1 ) = mean( dist );

%迭代优化过程中误差变化曲线
% figure( 'Name' , '迭代误差曲线' , 'NumberTitle' , 'off' , 'position' , [ left + width * 2 + 10 * 2 , bottom , width , hight ] );
figure(num_figure + 2 );
plot(0:iteration,err_rec);
grid on
title( '迭代过程误差变化曲线' )
xlabel( '迭代次数' );
ylabel( '迭代误差' );
grid on;

% 最后点云匹配的结果
% figure( 'Name' ,  '精配准结果' , 'NumberTitle' , 'off' , 'position' , [ left + width * 1 + 10 * 1 , bottom - hight , width , hight ] );
figure(num_figure + 3 );
scatter3( data_source( 1 , : ) , data_source( 2 , : ) , data_source( 3 , : ) , 'b.' );
hold on;
scatter3( data_target( 1 , : ) , data_target( 2 , : ) , data_target( 3 , : ) , 'r.' );
hold off;
daspect( [1 1 1] );
title( '精配准结果' )
xlabel( 'x' ); ylabel( 'y' ); zlabel( 'z' );
grid on;

disp( '旋转平移矩阵的真值：' );
% disp( T0 );  %旋转矩阵真值
disp( '计算出的旋转平移矩阵：' );
T_final = [ Rf , Tf ];
T_final = [ T_final ; 0 , 0 , 0 , 1 ];
disp( T_final );

if inlier_flag == 1
    disp( [ '*********************剔除外点*********************' ] )
else
    disp( [ '********************不剔除外点********************' ] )
end
disp( [ ' 点云数据集规模：', num2str( size( data_source , 2 ) ) ] );
disp( [ ' 最终误差mse='    , num2str( mean( dist.^2 ) ) ] );
disp( [ ' 最终误差rmse='   , num2str( sqrt( mean( dist.^2 ) ) ) ] );
disp( [ ' ICP配准时间：'   , num2str( sum( time_record ) ) , 's' ] );    % time=18.1993    err=8.1929e-17    iteration=41
                                                                         % time=18.2324    err=8.1929e-17   iteration=41
disp( [ ' 迭代次数：' , num2str( iteration ) , '次' ] );
 

