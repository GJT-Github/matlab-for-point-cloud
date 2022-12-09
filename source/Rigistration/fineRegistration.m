function [R_Final,T_Final]= fineRegistration(data_target,data_source)      %
% []= fineRegistration(data_target,data_source) ��ICPʵ��
%      
%  �������  
%   data_target   ������׼���Ŀ����� 3 * n
%   data_source   ������׼���Դ����   3 * n
%
%
%  �������
%   R_Final            ������׼����ת����   3 * 3
%   T_Final            ������׼��ƽ�ƾ���   3 * 1
%   
%  Author��GJT 
%  E-mail��gjt0114@outlook.com



% *****************ICPʵ��*****************
%https://blog.csdn.net/Darlingqiang/article/details/108441503
%

%% ��������
    max_iteration    = 100;                       % ����������
    Tolerance        = 5.0e-30;                   % ���
    step_Tolerance   = 1.0e-14;                   % �����ݲ����ֵ


    inlier_ratio     = 1;                     % 0.999 �ڵ��ж�������ŷʽ����Ϊ�ж�������
%     B_baoHeDu        = 2.5;                       % M�����б��Ͷ�B
  
    kd               = 1;                         % ��Ӧ������ѡ��,0��ŷʽ����Ѱ�Ҷ�Ӧ�� 1��KD-treeѰ�Ҷ�Ӧ��
  
    inlier_flag      = 1;                         % �޳������ƣ�1���޳� 0�����޳�
    show_flag        = 1;                         % ��׼������ʾ���ƣ�1����ʾ��0������ʾ

    iteration        = 0;                         %��ʼ����������

    num_figure = 4;

    % ��ʼ��תƽ�ƾ�������趨
    T_final   =  eye( 4 , 4 );                %��ת�������׼����תƽ�ƾ��� eye:��λ��  
    Rf        =  T_final( 1:3 , 1:3 );        %��ȡ����׼������ת����       Rf��ֵ����λ��
    Tf        =  T_final( 1:3 ,4 );           %��ȡ����׼����ƽ�ƾ���       Tf��ֵ��������

% data_source     = Rf * data_source + Tf * ones( 1 , size( data_source , 2) );      %���θ��µ㼯���������׼�����,�������׼��ע�͵�

    data_source_old = data_source;               %�м�任���
    t_a = 0;
    displayer = displayFunction;                 %$��ʾ��������

%����ǰ��ʾ debug
% displayer = displayFunction;
% displayer.displayRigistration(data_target,data_source);


%% �����Ż�
% tic;
while(1)
    
  iteration = iteration + 1;

    % �ҳ���Ӧ�㼯
    if kd == 1
        %����Kd-tree�ҳ���Ӧ�㼯
        kd_tree = KDTreeSearcher( data_target' , 'BucketSize' , 10 );
        [index, dist] = knnsearch( kd_tree , data_source' );
        tic
    else
        %����ŷʽ�����ҳ���Ӧ�㼯
        k=size(data_source,2);
        for i = 1:k
            data_q1( 1 , : ) = data_target( 1 , : ) - data_source( 1 , i );    % �����㼯�еĵ�x����֮��
            data_q1( 2 , : ) = data_target( 2 , : ) - data_source( 2 , i );    % �����㼯�еĵ�y����֮��
            data_q1( 3 , : ) = data_target( 3 , : ) - data_source( 3 , i );    % �����㼯�еĵ�z����֮��
            distance = sqrt( data_q1( 1 , : ).^2 + data_q1( 2 , : ).^2 + data_q1( 3 , : ).^2 );  % ŷ�Ͼ���
            [dist(i), index(i)] = min( distance );   % �ҵ�������С���Ǹ���
        end
        clear k

    end   


%     %��������¼
%     disp( [ '���err=' , num2str( mean( dist ) ) ] );
%     disp( ['��������iteration=' , num2str( iteration ) ] );
%     err_rec( iteration ) = mean( dist );
%     err = min( err_rec );
%     rmse= sqrt(err);


    %�޳����
    if inlier_flag == 1
      % ����������ֻȡǰ��ռ��Ϊinlierratio�ڵĵ���Ӧ�����
        [~, idx]   = sort( dist ); 
        inlier_num = round( size( data_source , 2 ) * inlier_ratio );
        idx        = idx( 1 : inlier_num );
        data_source_temp = data_source( : , idx );
        dist       = dist( idx );
        index      = index( idx );
        data_mid   = data_target( : , index ); 
    %���޳����
    else      
%         [~, idx]   = sort( dist );  
%         index      = index( idx );
        inlier_num = size( data_source , 2 ) ;
        
        data_source_temp = data_source;
        data_mid         = data_target(:,index);  
    end

    %��������¼
    dist=mink(dist,inlier_num);
    err_rec( iteration ) = mean( dist );
    err = min( err_rec );
    % rmse= sqrt(err);

    % disp( [ '���MSE=' , num2str( mean( dist ) ) ] );
    % disp( [ '���RMSE=' , num2str( sqrt( mean( dist.^2 ) ) ) ] );
    % disp( ['��������iteration=' , num2str( iteration ) ] );


%������תƽ�ƾ���ǰ��ʾ debug
% close all
% displayer.displayRigistration(data_target,data_source);


    % ��Ԫ��Ԫ����� ��ת����R �� ƽ������T    data_source -> data_mid
    % [R_new,t_new] = Quater_Registration(data_source', data_target(:,index)');
%     [R_new,t_new] = Quater_Registration(data_source_temp', data_mid');

    % ȥ���Ļ���SVD�ֽ������ת������ƽ������   data_source -> data_mid
    [R_new, t_new] = rigidTransform3D( data_source_temp' ,  data_mid' );



    % �����ۼƵ���ת������ƽ������
    Rf = R_new * Rf;
    Tf = R_new * Tf + t_new;
    
    % ���µ㼯
    data_source = Rf * data_source_old + Tf * ones( 1 , size( data_source_old , 2 ) );

%Ӧ����ת�任�������ʾ debug
% close all
% displayer.displayRigistration(data_target,data_source);

    
    %%������������������������������������������������������������ M-��������Ŀ�꺯����������������������������������������������������������������������������������������������������������
      
    % % k=size(data_source,2);
    % % for i = 1:k                    
    %     data_q1( 1 , : ) = data_target( 1 , index ) - data_source( 1 , idx );    % ������Ӧ�㼯�еĵ�x����֮��
    %     data_q1( 2 , : ) = data_target( 2 , index ) - data_source( 2 , idx );    % ������Ӧ�㼯�еĵ�y����֮��
    %     data_q1( 3 , : ) = data_target( 3 , index ) - data_source( 3 , idx );    % ������Ӧ�㼯�еĵ�z����֮��

    %     f(1,:) = F_M_estition( data_q1( 1 , : ) , B_baoHeDu );                  
    %     f(2,:) = F_M_estition( data_q1( 2 , : ) , B_baoHeDu );
    %     f(3,:) = F_M_estition( data_q1( 3 , : ) , B_baoHeDu );
    % % end
    % f_value_m_estition = mean( sum( f ));
    % % err_rec( iteration ) = f_value_m_estition;
    % % err = min( err_rec );
    % err = f_value_m_estition;
    % % clear f

   %%������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������ 

     


   
    % �����б� Ŀ�꺯�������������ֵ������ֹ����
        %���ﵽ��ֵ
        if err < Tolerance 
            disp( '�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D' );
            disp( '���1���Ż�����Ѿ��ﵽĿ�꣬�����Ż�' );
            break
        end

        %�ֲ�����
        if iteration > 1 &&  abs(err_rec(iteration-1) - err_rec(iteration))< step_Tolerance 
            disp( '�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D' );
            disp( '���2��ǰ�������������С����ֵ���ֲ����ţ������Ż�' );
            disp('��һ�ε�����');
            disp(num2str(err_rec(iteration-1)));
            disp('���ε�����');
            disp(num2str(err_rec(iteration)));
            disp('���δε�������ֵ����ֵ��');
            disp(num2str(abs(err_rec(iteration-1) - err_rec(iteration))));
            break
        end

        %������������ֵ
        if iteration >= max_iteration
            disp( '�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D�D' );
            disp( '���3�����������ﵽ��ֵ�������Ż�' );
            break
        end

      time_record(iteration) = toc;
      t_a(iteration+1) = t_a(iteration) + time_record(iteration);

    % ��ʾ�м�������׼���̣�
    if show_flag == 1
        % if iteration == 1
        %     % h = figure( 'position' , [ left + width + 10 * 1 , bottom , width , hight ] );
        %     figure(num_figure);
        % end
        % scatter3( data_source( 1 , : ) , data_source( 2 , : ) , data_source( 3 , : ) , 'b.' );
        % hold on;
        % scatter3( data_target( 1 , : ) , data_target( 2 , : ) , data_target( 3 , : ) , 'r.' );
        % hold off;
        % title( '��׼����չʾ' )
        % xlabel( 'x' );
        % ylabel( 'y' );
        % zlabel( 'z' );
        % grid on;
        % legend( 'source-file_1(ģ��)' , 'target-file_2' )
        % daspect( [1 1 1] );
        % pause( 0.1 );
        % drawnow


        displayer.displayProcessOfICP(data_source,data_target,iteration,num_figure);    %$

        disp( [ '��� MSE  = '          , num2str( mean( dist.^2 ) ) ] );
        disp( [ '��� RMSE = '          , num2str( sqrt( mean( dist.^2 ) ) ) ] );
        disp( [ '�������� iteration = ' , num2str( iteration ) ] );
        disp( [ '�����ʱ��'            , num2str(time_record(iteration)) ,' s '] );

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



% ��������������
if kd == 1
    %����Kd-tree�ҳ���Ӧ�㼯
    kd_tree = KDTreeSearcher( data_target' , 'BucketSize' , 10 );
    [ index , dist ] = knnsearch(kd_tree, data_source');
%     [n1,d1] = knnsearch(transpose(P), transpose(P), 'k', 400);           % demo1��k���ڲ���
else
    %����ŷʽ�����ҳ���Ӧ�㼯
    k = size( data_source , 2 );
    for i = 1 : k
        data_q1( 1 , : ) = data_target( 1 , : ) - data_source( 1 , i );    % �����㼯�еĵ�x����֮��
        data_q1( 2 , : ) = data_target( 2 , : ) - data_source( 2 , i );    % �����㼯�еĵ�y����֮��
        data_q1( 3 , : ) = data_target( 3 , : ) - data_source( 3 , i );    % �����㼯�еĵ�z����֮��
        distance = sqrt( data_q1( 1 , : ).^2 + data_q1( 2 , : ).^2 + data_q1( 3 , : ).^2 );  % ŷ�Ͼ���
        [ dist( i ) , index( i ) ] = min( distance );   % �ҵ�������С���Ǹ���
    end
    clear k
end

dist = mink(dist,inlier_num);
err_rec( iteration + 1 ) = mean( dist );

%�����Ż����������仯����
% figure( 'Name' , '�����������' , 'NumberTitle' , 'off' , 'position' , [ left + width * 2 + 10 * 2 , bottom , width , hight ] );
figure(num_figure + 2 );
plot(0:iteration,err_rec);
grid on
title( '�����������仯����' )
xlabel( '��������' );
ylabel( '�������' );
grid on;

% ������ƥ��Ľ��
% figure( 'Name' ,  '����׼���' , 'NumberTitle' , 'off' , 'position' , [ left + width * 1 + 10 * 1 , bottom - hight , width , hight ] );
figure(num_figure + 3 );
scatter3( data_source( 1 , : ) , data_source( 2 , : ) , data_source( 3 , : ) , 'b.' );
hold on;
scatter3( data_target( 1 , : ) , data_target( 2 , : ) , data_target( 3 , : ) , 'r.' );
hold off;
daspect( [1 1 1] );
title( '����׼���' )
xlabel( 'x' ); ylabel( 'y' ); zlabel( 'z' );
grid on;

disp( '��תƽ�ƾ������ֵ��' );
% disp( T0 );  %��ת������ֵ
disp( '���������תƽ�ƾ���' );
T_final = [ Rf , Tf ];
T_final = [ T_final ; 0 , 0 , 0 , 1 ];
disp( T_final );

if inlier_flag == 1
    disp( [ '*********************�޳����*********************' ] )
else
    disp( [ '********************���޳����********************' ] )
end
disp( [ ' �������ݼ���ģ��', num2str( size( data_source , 2 ) ) ] );
disp( [ ' �������mse='    , num2str( mean( dist.^2 ) ) ] );
disp( [ ' �������rmse='   , num2str( sqrt( mean( dist.^2 ) ) ) ] );
disp( [ ' ICP��׼ʱ�䣺'   , num2str( sum( time_record ) ) , 's' ] );    % time=18.1993    err=8.1929e-17    iteration=41
                                                                         % time=18.2324    err=8.1929e-17   iteration=41
disp( [ ' ����������' , num2str( iteration ) , '��' ] );
 

