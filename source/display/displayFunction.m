function  displayer = displayFunction      %��ͼ��װ
% display = displayFunction ��
%                             ͨ�������������ö������
%
%  https://blog.csdn.net/weixin_39032619/article/details/109294078
%      
%  �������  
%    P     : Ŀ�����XYZ������, 3*n �����ݾ���
%    Q     : ��������XYZ�����ݣ�3*n �����ݾ���
%
%
%  �������
%      ��
%   
%  Author��GJT 
%  E-mail��gjt0114@outlook.com

% warning off % ��������/��ʾ����

%% ���ƶ�ȡ�����ʾ
displayer.displayInitPointCloud = @displayInitPointCloud;                         %�����ƻ���
displayer.displayNormalOnSourcePointCloud = @displayNormalOnSourcePointCloud;     %����������

%% ��������ȡ�����ʾ
displayer.displayFirstPickKeyPoint = @displayFirstPickKeyPoint;                   %���ƴ���ȡ��������
displayer.displayFinalPickKeyPoint = @displayFinalPickKeyPoint;                   %����������ȡ��������

%% �������������������ʾ
displayer.displayPFHOfKeyPoint = @displayPFHOfKeyPoint;                           % �����������PFH����
displayer.displayFPFHOfKeyPoint = @displayFPFHOfKeyPoint;                         % �����������FPFH����

%% ����ƥ�������ʾ
displayer.displayPointCloudAndLine = @displayPointCloudAndLine;                   % ����ƥ���Ķ�Ӧ��
displayer.displayDeleteDisdencePointCloudAndLine = @displayDeleteDisdencePointCloudAndLine;  % �����޳�������ֵ��Ķ�Ӧ��
displayer.displayRigidInvariantConstraintsPointCloudAndLine = @displayRigidInvariantConstraintsPointCloudAndLine;   % ���Ƹ��Բ���Լ���޳�����ƥ���Ķ�Ӧ��                        % ���ƴ���ȡ��������
displayer.displayRANSACPointCloudAndLine = @displayRANSACPointCloudAndLine;       % ����RANSAC�޳�����ƥ���Ķ�Ӧ��

%% ����׼�����ʾ
displayer.displayRigistration = @displayRigistration;                             % ���ƴ�ƥ����

%% ����׼�����ʾ
displayer.displayProcessOfICP = @displayProcessOfICP;                             % ����ICP��׼����

%% ���յõ�����תƽ�Ʊ任����һ�α任�����ʾ
displayer.displayFinalQ2P = @displayFinalQ2P;                                     % �������յõ�����תƽ�ƾ���һ�α任���

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

%�����ƻ���
function [] = displayInitPointCloud(P,Q)
    global axe;
    global posionFigureX;
    global posionFigureY;
    global posionFigureZ;
    global posionFigureN;
    global size_Point;
    global num_Figure;

	figure(num_Figure);                               %�������ĵ���ͼ
	% set(gcf,'position',[10 350 500 400]);
	set(gcf,'position',[posionFigureX,posionFigureY,posionFigureZ,posionFigureN]);
	axe(1)=subplot(221);
	plot3(P(1,:),P(2,:),P(3,:),'r.','markersize',size_Point);        %plot��ͼ�������ֱ�ȡP�е�1.2.3�����е���Ϊ�����ᣬr��ʾ��ɫ
	hold on
	plot3(Q(1,:),Q(2,:),Q(3,:),'b.','markersize',size_Point);
	title('ģ������볡�����Ƴ�ʼλ��')
	view(3)
end


%����������
function [] = displayNormalOnSourcePointCloud(P,normal)
    global axe;
    global posionFigureX;
    global posionFigureY;
    global posionFigureZ;
    global posionFigureN;
	figure(2);
    set(gcf,'position',[posionFigureX + (posionFigureZ+10)*1,posionFigureY,posionFigureZ,posionFigureN]);
	plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot��ͼ�������ֱ�ȡP�е�1.2.3�����е���Ϊ�����ᣬr��ʾ��ɫ
	hold on
	quiver3( P(1,:) , P(2,:) , P(3,:)  ,  normal(1,:) , normal(2,:) , normal(3,:) ,'g');
    plot3(0,0,0,'b*');
	xlabel('x');ylabel('y');zlabel('z');
	title('Դ���Ʒ�������ʾ');
end


%--------------------------------------------main_end-------------------------------------------------
%% --------------------------------------------featurePoint---------------------------------------------
%���ƴ���ȡ��������
function [] = displayFirstPickKeyPoint(p0,q0)
    global axe;
    figure(1);                                 %��ʾp0 q0�����㣬����Ϊ�õ㷨����������8����ķ���������ֵ�ۼӵ�ƽ��ֵС������ƽ��ֵ��ƽ��
    axe(2)=subplot(222);
    plot3(p0(1,:),p0(2,:),p0(3,:),'r.');
    hold on
    plot3(q0(1,:),q0(2,:),q0(3,:),'b.');
    title('ģ�������Ŀ����Ƶ����������ȡ')
    view(2)
    % daspect([1 1 1]);                      %��ά�Ƕȳ�ͼ
end



%����������ȡ��������
function [] = displayFinalPickKeyPoint(p0,q0)
    global axe;
    global size_Point;

    figure(1);
    % colordef black; % 2D/3Dͼ������ɫ
    axe(3)=subplot(223);
    plot3(p0(1,:),p0(2,:),p0(3,:),'r.','markersize',size_Point);
    title('Ŀ����ƹؼ��㾫��ȡ')

    aplha=0:pi/40:2*pi;
    global r_P_k r_Q_k;

    % % ���Ƽ�����
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
        title('ģ����ƹؼ��㾫��ȡ')
        view(2)
        warning off % ��������/��ʾ����
        linkaxes(axe,'xy')
        warning on % ��������/��ʾ����
    
end

%--------------------------------------------featurePoint_end------------------------------------------
%% --------------------------------------------PFHCaculate-----------------------------------------------



%�����������PFH����
function [] = displayPFHOfKeyPoint(vep)
    global axe;
    global posionFigureX;
    global posionFigureY;
    global posionFigureZ;
    global posionFigureN;
    figure(3);
    set(gcf,'position',[posionFigureX + (posionFigureZ+10)*2,posionFigureY,posionFigureZ,posionFigureN]);
    axe(1)=subplot(231);
    bar(vep(:,144));                      %bar��bar3�ֱ��������ƶ�ά����ά��ֱ��ͼ������vep�е�144��������
    axis([0 64 0 1200])                   %axis([xmin xmax ymin ymax])���õ�ǰ������ x���y��ķ�Χ
    title('��144���ؼ����PFH����������')
end


%�����������FPFH����
function [] = displayFPFHOfKeyPoint(vep)
    global axe;
    global posionFigureX;
    global posionFigureY;
    global posionFigureZ;
    global posionFigureN;
    figure(3);
%     set(gcf,'position',[posionFigureX + (posionFigureZ+10)*2,posionFigureY,posionFigureZ,posionFigureN]);
    axe(1)=subplot(231);
    bar(vep(:,144));                      %bar��bar3�ֱ��������ƶ�ά����ά��ֱ��ͼ������vep�е�144��������
    axis([0 64 0 1200])                   %axis([xmin xmax ymin ymax])���õ�ǰ������ x���y��ķ�Χ
    title('��144���ؼ����FPFH����������')
end

%--------------------------------------------PFHCaculate_end---------------------------------------------
%% --------------------------------------------removeWrongMatch------------------------------------------


%��ʾģ���볡�������㼰�� ����� ����
function [] = displayPointCloudAndLine(P,Q,p0,q0,fep,feq,nv)

    global axe;
    %p0=P(:,fep);
    %q0=Q(:,feq);

    % ���� Ŀ����� �� Դ����
    figure(3);
    axe(2)=subplot(232);
    plot3(p0(1,:),p0(2,:),p0(3,:),'r.');
    hold on
    plot3(q0(1,:),q0(2,:),q0(3,:),'b.');

    %��������ڵ������
    for i=1:length(nv)                    
        hold on
        x=[Q(1,feq(i)),P(1,fep(nv(1,i)))];%ֱ�ߵ����x�������յ�x���꣬ע�����Ϊ�����е�һ�㣬�յ�Ϊ��Ӧ����ģ���������
        y=[Q(2,feq(i)),P(2,fep(nv(1,i)))];
        z=[Q(3,feq(i)),P(3,fep(nv(1,i)))];
        plot3(x,y,z,'g-');
        %pause(0.02)
    end
    xlabel('x');ylabel('y');zlabel('z');
    title('Դ���ƺ�Ŀ��������������');
    view(2)
end


%���� ɾ���������0.05���ͼ
function [] = displayDeleteDisdencePointCloudAndLine(P,Q,p0,q0,fep,feq,nv,e_dis)

    global axe;

    figure(3);
    axe(3)=subplot(233);
    plot3(p0(1,:),p0(2,:),p0(3,:),'r.');  %ԭP�еĹؼ���
    hold on
    plot3(q0(1,:),q0(2,:),q0(3,:),'b.');  %ԭQ�еĹؼ���
    for i=1:length(nv)
        hold on
        x=[Q(1,feq(i)),P(1,fep(nv(1,i)))];%���¹ؼ���ָ��������������ָ�������
        y=[Q(2,feq(i)),P(2,fep(nv(1,i)))];
        z=[Q(3,feq(i)),P(3,fep(nv(1,i)))];
        plot3(x,y,z,'g-');
    end
    xlabel('x');ylabel('y');zlabel('z');
    title(['ȥ���������',num2str(e_dis),'�����������']);
    view(2)

end



%���Ƹ��Բ���Լ����ĵ�����
function [] = displayRigidInvariantConstraintsPointCloudAndLine(P,Q,p0,q0,fep,feq,nv)
    
    global axe;

    figure(3);
    axe(4)=subplot(234);
    plot3(p0(1,:),p0(2,:),p0(3,:),'r.');  %ԭP�еĹؼ���
    hold on
    plot3(q0(1,:),q0(2,:),q0(3,:),'b.');  %ԭQ�еĹؼ���
    for i=1:length(nv)
        hold on
        x=[Q(1,feq(i)),P(1,fep(nv(1,i)))];%���¹ؼ���ָ��������������ָ�������
        y=[Q(2,feq(i)),P(2,fep(nv(1,i)))];
        z=[Q(3,feq(i)),P(3,fep(nv(1,i)))];
        plot3(x,y,z,'g-');
    end
    xlabel('x');ylabel('y');zlabel('z');
    title('������Բ���Լ�������ĵ�����');
    view(2)
end


% ����RANSAC�޳�����ƥ���Ķ�Ӧ��
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
    title('RANSAC�޳���ƥ������');
    view(2)

end

%--------------------------------------------removeWrongMatch_end------------------------------------------

%--------------------------------------------displayRigistration------------------------------------------

%% ������׼���
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
    title('��Ԫ����׼');
    view(3)
    warning off % ��������/��ʾ����
    linkaxes(axe(2:6),'xy')
    warning on % ��������/��ʾ����


end

%% --------------------------------------------displayRigistration_end------------------------------------------
%% --------------------------------------------ICP--------------------------------------------------------------
function [] = displayProcessOfICP(data_source,data_target,iteration,num_Figure)
    %
    % data_source  ��3 * n
    % data_target  ��3 * n
    %
        if iteration == 1
            % h = figure( 'position' , [ left + width + 10 * 1 , bottom , width , hight ] );
            figure(num_Figure);
        end
        scatter3( data_source( 1 , : ) , data_source( 2 , : ) , data_source( 3 , : ) , 'b.' );
        hold on;
        scatter3( data_target( 1 , : ) , data_target( 2 , : ) , data_target( 3 , : ) , 'r.' );
        hold off;
        title( '��׼����չʾ' )
        xlabel( 'x' );
        ylabel( 'y' );
        zlabel( 'z' );
        grid on;
        legend( 'source-file_1(ģ��)' , 'target-file_2' )
        daspect( [1 1 1] );
        pause( 0.1 );
        drawnow

end




%% --------------------------------------------ICP_end----------------------------------------------------------
%% --------------------------------------------Final_end--------------------------------------------------------
function [] = displayFinalQ2P(data_source,data_target)
    %
    % data_source  ��3 * n
    % data_target  ��3 * n
    %

        figure;
        scatter3( data_source( 1 , : ) , data_source( 2 , : ) , data_source( 3 , : ) , 'b.' );
        hold on;
        scatter3( data_target( 1 , : ) , data_target( 2 , : ) , data_target( 3 , : ) , 'r.' );
        hold off;
        title( '���յõ�����תƽ�ƾ���һ�α任չʾ' )
        xlabel( 'x' );
        ylabel( 'y' );
        zlabel( 'z' );
        grid on;
        legend( 'source-file_1(ģ��)' , 'target-file_2' )
        daspect( [1 1 1] );
        drawnow

end


%% --------------------------------------------Final_end--------------------------------------------------------

