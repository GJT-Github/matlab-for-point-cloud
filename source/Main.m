% ���ƴ���
%
% Author ��GJT
% E-mail ��gjt0114@outlook.com

function Main                                      % https://blog.csdn.net/weixin_37610397/article/details/80441523

	clc
	clear
	close all
    
	addpath(genpath('../source/'))                 % addpath �����SGDLibrary-masterĿ¼   genpath �Ƕ�ȡSGDLibrary-masterĿ¼������Ŀ¼
	%% ��ȡ�ļ�
	[P,Q] = readPointCloud();

	%���ƶ����ĵ���
	displayer = displayFunction;
	displayer.displayInitPointCloud(P,Q);          %P :�� Q:��

	% AABB��Χ�м���
    % global longline;
    [P_bin,P_max,P_min,longline] = box(P);          % for Debug
   %[0.1558 0.1522 0.1174]    0.2474
    %�ֱ��ʼ���
    [resolution_P]=computeCloudResolution(P);  % 0.00055 5.5540830e-04
    [resolution_Q]=computeCloudResolution(Q);  % 0.00055 5.4773246e-04

    %���׸��� 2.1.1����Ӧ����ƽ���������
    global r_P_k r_Q_k;
    paper = paperISS;
	r_P_k = paper.paper(P) * 5;  %0.0083
	r_Q_k = paper.paper(Q) * 5;  %0.0075

    

	%% ͨ��8����PCA���� ���������� [pn��qn]
	% PCA���������ƣ�ĳ������Э���������С����ֵ��Ӧ�����������õ㷨����
	k = 8;                                    %����ѡ����8����
	[pn,qn] = normalCaculate(P,Q,k);

	%����Ŀ����Ƶķ�����
	displayer.displayNormalOnSourcePointCloud(P,pn);

	% save ../Datas/MatFiles/Normal_asc.mat         % for Debug
    % save ../Datas/MatFiles/Normal_pcd.mat         % for Debug

	%% ��������ȡ   demo_1
    % close all                                     % for Debug
    % clear all                                     % for Debug
    % clc                                           % for Debug
	% load ../Datas/MatFiles/Normal_asc.mat         % for Debug
    % load ../Datas/MatFiles/Normal_pcd.mat         % for Debug


	% 1��
	[p0,q0,fep,feq,feq0,n1,d1,n2,d2] = featurePoint(P,Q,pn,qn,k);
    

    % 2��ISS������  
    % Note��1����ֵѡȡ�� 2��ȡ�ô���Ǳ�Ե�� 3������Ա�
	r  = 0.05;                                 % ����뾶
	e1 = 0.7;                                  % �м�����ֵ���������ֵ֮�ȵ� ��ֵ
	e2 = 0.4;                                  % �м�����ֵ����С����ֵ֮�ȵ� ��ֵ  

	%   r  = 0.005;                            % ����뾶
	% 	e1 = 0.6;                              % �м�����ֵ���������ֵ֮�ȵ� ��ֵ
	% 	e2 = 0.3;  
	% for e1 =0.3:0.2:0.9                      % for Debug e1 �� e2 ��ͬȡֵ�ؼ���Ӱ��
		% for e2 = 0.1:0.1:0.8
% 		 tic
% 			[p0,q0,fep,feq,feq0,n1,d1,n2,d2] = keyPointOfISS(P,Q,r,e1,e2);  
% 		 toc
		% end
	% input(string);
	% close all
	% end

	% 	mean(d1(2,:))                        % for Debug           
	% 	mean(d2(2,:))                        % for Debug
	%   [P_bin,P_max,P_min] = box(P);        % for Debug



	%����������ȡ��������
	% displayer.displayFinalPickKeyPoint(p0,q0);

	% save ../Datas/MatFiles/FP.mat           % for Debug


	%% ��������/����  demo_2
	% 1��PFH
	% clear all                              % for Debug
	% close all                              % for Debug
	% clc                                    % for Debug
	% load ../Datas/MatFiles/FP.mat          % for Debug
% 	tic
%       global r_P_k ;
% 		r_PFH = r_P_k/5; %0.03
        % r_PFH = 0.003; %0.03
		% [vep,veq] = PFHCaculate(P,Q,p0,q0,fep,feq,pn,qn,n1,d1,n2,d2,r_PFH);
	% 	[vep] = pfhDescriptor(P,fep,pn,n1,d1) ;
% 	toc

	% ����ĳ��PFH����
	% displayer.displayPFHOfKeyPoint(vep);

	% save ../Datas/MatFiles/PFHC.mat         % for Debug
	
	
	% 2��FPFH
	% clear all                               % for Debug
	% close all                               % for Debug
	% clc                                     % for Debug
	% load ../Datas/MatFiles/FP.mat           % for Debug
	r_P = 0.003;
	r_Q = 0.003;

	%��������ת����400 * n תΪ 1*400*n Ԫ������
	% for i = 1:size(n1,1)                    % for Debug
	   % idx_P{i} = n1(:,i)';                 % for Debug
	   % dis_P{i} = d1(:,i)';                 % for Debug
	   % idx_Q{i} = n2(:,i)';                 % for Debug
	   % dis_Q{i} = d2(:,i)';                 % for Debug
	% end                                     % for Debug
	% tic
		% [vep,veq] = FPFHCaculate(P,Q,pn,qn,fep,feq,r_P,r_Q,idx_P,dis_P,idx_Q,dis_Q);
		[vep,veq] = FPFHCaculate(P,Q,pn,qn,fep,feq,r_P,r_Q);
	% toc
	%����ĳ��FPFH����
    % displayer = displayFunction;             % for Debug
	displayer.displayFPFHOfKeyPoint(vep);

	% save ../Datas/MatFiles/FPFHC.mat         % for Debug


	%% demo_3
	%% ��ƥ���޳�
	% close all                               % for Debug
	% clear all                               % for Debug
	% clc                                     % for Debug
	% load ../Datas/MatFiles/PFHC.mat         % for Debug
	% load ../Datas/MatFiles/FPFHC.mat        % for Debug
	e_Delet_Distance  = 0.05;                 %0.05  for bun0*.asc
	e_RANSAC_Distance = 0.005;                %0.005 for bun0*.asc
	[p0,q0,feq,nv] = removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq,e_Delet_Distance,e_RANSAC_Distance);

	% save ../Datas/MatFiles/RWM.mat          % for Debug


	%%  ����������
	% load ../Datas/MatFiles/RWM.mat          % for Debug
	% RMSE(p0,q0)


	%% ����ƥ��/��׼
	% close all                             % for Debug
	% clear all                             % for Debug
	% clc                                   % for Debug
	% load ../Datas/MatFiles/RWM.mat        % for Debug
	[Q1,R_Coarse,T_Coarse] = coarseRegistration(P,Q,fep,feq,nv);
    disp(['coarse Registration RMSE:',num2str( RMSE(P,Q1))]);

% 	save ../Datas/MatFiles/CR.mat           % for Debug


	%% ����׼
	% close all                             % for Debug
	% clear all                             % for Debug
	% clc                                   % for Debug
	% load ../Datas/MatFiles/CR.mat         % for Debug

    [R_Final,T_Final] = finalResgistration(P,Q1);

%     save ../Datas/MatFiles/FR.mat          % for Debug


	%% ���յ���תƽ�ƾ���
    % load ../Datas/MatFiles/FR.mat          % for Debug
	R = R_Final * R_Coarse;
	T = R_Final * T_Coarse + T_Final;

	Q1_temp = R_Coarse * Q + T_Coarse * ones( 1 , size( Q , 2 ) );
    Q2 = R * Q + T * ones( 1 , size( Q , 2 ) );
% 	H = [R,T;0 0 0 1];                     % P = H * Q  -->  P = R * Q + T

	displayer.displayFinalQ2P(P,Q1_temp);
    title('����׼���')
	displayer.displayFinalQ2P(P,Q2);

end




%�����������ļ�
function [P,Q] = readPointCloud()

% 	file1='../Datas/bun045.asc';
% 	file2='../Datas/bun000.asc';

	% file1='../Datas/rabbit.pcd';
	% file2='../Datas/rabbit_z_45.pcd';

	file1='../Datas/sdanford/bun000.ply';
	file2='../Datas/sdanford/bun045.ply';


% 	S=pcread(file1);
% 	T=pcread(file2);
% 	rmse(S,T,0,0.001);
    

	tic         %��ʱ��ʼ for Debug
	[P,Q]= readPointCloudDatas(file1,file2);
	toc         %��ʱ�������Զ���ӡ���е�ʱ�� for Debug

%     r_cmatrix = ones(size(P')).*[1 0 0];
%     b_cmatrix = ones(size(Q')).*[0 0 1];
%     S1 = pointCloud(P','Color',r_cmatrix);
%     Q0 = pointCloud(Q','Color',b_cmatrix);
%     pcshow(S1);
%     hold on
%     pcshow(Q0);

    disp(['origin RMSE:',num2str(RMSE(P,Q))]);

end

%���Ʒֱ��ʼ���
function [resolution]=computeCloudResolution(cloud)
% cloud 3 * n
%
	NS = createns(cloud','NSMethod','kdtree');
	[~,dis] = knnsearch(NS,cloud','k',2);

	%����ϴ��10%�ĵ���Ϊ�������޳���������ֱ���
	dis_nearest = sort(dis(:,2));
	dis_nearest(end-round(size(dis_nearest,1)*0.1):end)=[];  
    resolution = mean(dis_nearest);
end


function [R_Final,T_Final]=finalResgistration(targetPointCloud,sourcePointCloud)
% 3��icp����׼

    P = targetPointCloud;
    Q1 = sourcePointCloud;

    % 1.
	[R_Final,T_Final] = fineRegistration(P,Q1);           % Q1 --transform--> P

    % 2.
	% [R_Final,T_Final] = icp(Q1',P');                    % Q1 --transform--> P
	% [R_Final,T_Final] = icp(Q',P');                     % Q1 --transform--> P

    % 3.�Դ�ICP�޳��ڵ��ʺͲ��޳��ڵ�Ľ�������ȫ���õ�ԭʼ�����ݣ��ڵ���ֻ�Լ���RMSE��Ч
%     r_point_cmatrix = ones(size(P')).*[1 0 0];
%     b_point_cmatrix = ones(size(Q1')).*[0 0 1];
%     P_temp = pointCloud(P',"Color",r_point_cmatrix);
%     Q_temp = pointCloud(Q1',"Color",b_point_cmatrix);
%     [H_Final,transformedCloud,rmse] = pcregistericp(P_temp,Q_temp,'Verbose',true,'Metric','pointToPlane','InlierRatio',0.95,'Tolerance',[0.00001,0.002],'MaxIterations',100);
%     R_Final = H_Final.Rotation;
%     T_Final = H_Final.Translation';


end