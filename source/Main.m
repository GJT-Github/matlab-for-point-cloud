% ���ƴ���
%
% Author ��GJT
% E-mail ��gjt0114@outlook.com

function Main   %https://blog.csdn.net/weixin_37610397/article/details/80441523

	clc
	clear
	close all

	
	addpath(genpath('../source/'))   %addpath �����SGDLibrary-masterĿ¼   genpath �Ƕ�ȡSGDLibrary-masterĿ¼������Ŀ¼
	%% ��ȡ�ļ�

% 	file1='../Datas/bun045.asc';
% 	file2='../Datas/bun000.asc';

	file1='../Datas/rabbit.pcd';
	file2='../Datas/rabbit_z_45.pcd';

	tic         %��ʱ��ʼ for Debug

	[P,Q]= readPointCloudDatas(file1,file2);

	toc         %��ʱ�������Զ���ӡ���е�ʱ�� for Debug

	%���ƶ����ĵ���
	displayer = displayFunction;
	displayer.displayInitPointCloud(P,Q);   %P :�� Q:��


    [P_bin,P_max,P_min] = box(P);       % for Debug

	%% ͨ��8����PCA���� ���������� [pn��qn]
	% PCA���������ƣ�ĳ������Э���������С����ֵ��Ӧ�����������õ㷨����
	k = 8;                                    %����ѡ����8����
	[pn,qn] = normalCaculate(P,Q,k);

	%����Ŀ����Ƶķ�����
	displayer.displayNormalOnSourcePointCloud(P,pn);

% 	save ../Datas/MatFiles/Normal_asc.mat       % for Debug
%     save ../Datas/MatFiles/Normal_pcd.mat       % for Debug

	%% ��������ȡ   demo_1
%     close all
%     clear all
%     clc
% 	  load ../Datas/MatFiles/Normal_asc.mat       % for Debug
%     load ../Datas/MatFiles/Normal_pcd.mat       % for Debug

	% 1��
% 	[p0,q0,fep,feq,feq0,n1,d1,n2,d2] = featurePoint(P,Q,pn,qn,k);

    % 2��ISS������  
    % Note��1����ֵѡȡ�� 2��ȡ�ô���Ǳ�Ե�� 3������Ա�
	r  = 0.05;                             % ����뾶
	e1 = 0.7;                              % �м�����ֵ���������ֵ֮�ȵ� ��ֵ
	e2 = 0.4;                              % �м�����ֵ����С����ֵ֮�ȵ� ��ֵ  

%   r  = 0.005;                            % ����뾶
% 	e1 = 0.6;                              % �м�����ֵ���������ֵ֮�ȵ� ��ֵ
% 	e2 = 0.3;    
    tic
	[p0,q0,fep,feq,feq0,n1,d1,n2,d2] = keyPointOfISS(P,Q,r,e1,e2);  
    toc
% 	mean(d1(2,:))                        % for Debug           
% 	mean(d2(2,:))                        % for Debug
%     [P_bin,P_max,P_min] = box(P);       % for Debug



	%����������ȡ��������
% 	displayer.displayFinalPickKeyPoint(p0,q0);

% 	save ../Datas/MatFiles/FP.mat           % for Debug


	%% ��������/����  demo_2
	% 1��PFH
	% clear all                              % for Debug
	% close all                              % for Debug
	% clc                                    % for Debug
% 	load ../Datas/MatFiles/FP.mat            % for Debug
tic
	r_PFH = 0.5; %0.03
	[vep,veq] = PFHCaculate(P,Q,p0,q0,fep,feq,pn,qn,n1,d1,n2,d2,r_PFH);
% 	[vep] = pfhDescriptor(P,fep,pn,n1,d1) ;
toc
	% ����ĳ��PFH����
    % global posionFigureX;         % for bedug
    % global posionFigureY;         % for bedug
    % global posionFigureZ;         % for bedug
    % global posionFigureN;         % for bedug
    % posionFigureX = 10;           % for bedug
    % posionFigureY = 350;          % for bedug
    % posionFigureZ = 500;          % for bedug
	% posionFigureN = 400;          % for bedug
	displayer.displayPFHOfKeyPoint(vep);

	save ../Datas/MatFiles/PFHC.mat         % for Debug
	
	
	% 2��FPFH
	% clear all                               % for Debug
	% close all                               % for Debug
	% clc                                     % for Debug
	% load ../Datas/MatFiles/FP.mat           % for Debug
	% r_P = 0.005;
	% r_Q = 0.005;

	%��������ת����400 * n תΪ 1*400*n Ԫ������
	% for i = 1:size(n1,1)
	   % idx_P{i} = n1(:,i)';
	   % dis_P{i} = d1(:,i)';
	   % idx_Q{i} = n2(:,i)';
	   % dis_Q{i} = d2(:,i)';
	% end
% tic
	% [vep,veq] = FPFHCaculate(P,Q,pn,qn,fep,feq,r_P,r_Q,idx_P,dis_P,idx_Q,dis_Q);
	% [vep,veq] = FPFHCaculate(P,Q,pn,qn,fep,feq,r_P,r_Q);
% toc
	%����ĳ��FPFH����
    % displayer = displayFunction;             % for Debug
	% displayer.displayFPFHOfKeyPoint(vep);

% 	save ../Datas/MatFiles/FPFHC.mat         % for Debug


	%% demo_3
	%% ��ƥ���޳�
	close all                             % for Debug
	clear all                             % for Debug
	clc                                   % for Debug
	load ../Datas/MatFiles/PFHC.mat       % for Debug
% 	load ../Datas/MatFiles/FPFHC.mat      % for Debug
	e_Delet_Distance  = 0.5;                    %0.05  for bun0*.asc
	e_RANSAC_Distance = 0.005;                  %0.005 for bun0*.asc
	[p0,q0,feq,nv] = removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq,e_Delet_Distance,e_RANSAC_Distance);

% 	save ../Datas/MatFiles/RWM.mat          % for Debug


	%%  ����������
	%load ../Datas/MatFiles/RWM.mat         % for Debug
	RMSE(p0,q0)


	%% ����ƥ��/��׼
	% close all                             % for Debug
	% clear all                             % for Debug
	% clc                                   % for Debug
	% load ../Datas/MatFiles/RWM.mat        % for Debug
	[Q1,R_Coarse,T_Coarse] = coarseRegistration(P,Q,fep,feq,nv);

% 	save ../Datas/MatFiles/CR.mat           % for Debug


	%% ����׼
	% close all                             % for Debug
	% clear all                             % for Debug
	% clc                                   % for Debug
	% load ../Datas/MatFiles/CR.mat         % for Debug
	[R_Final,T_Final] = fineRegistration(P,Q1);           % Q1 --transform--> P
	% [R_Final,T_Final] = icp(Q1',P');                    % Q1 --transform--> P
	% [R_Final,T_Final] = icp(Q',P');                     % Q1 --transform--> P
    
    % save ../Datas/MatFiles/FR.mat          % for Debug


	%% ���յ���תƽ�ƾ���
    % load ../Datas/MatFiles/FR.mat            % for Debug
	% R = R_Final * R_Coarse;
	% T = R_Final * T_Coarse + T_Final;
	% H = [R,T;0 0 0 1];                       % P = H * Q  -->  P = R * Q + T
% 
	% displayer.displayFinalQ2P(P,R * Q + T * ones(1,size( Q , 2 )));


end


