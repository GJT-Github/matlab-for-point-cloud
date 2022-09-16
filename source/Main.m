% ���ƴ���
%
% Author ��GJT
% E-mail ��gjt0114@outlook.com

function main   %https://blog.csdn.net/weixin_37610397/article/details/80441523

	clc
	clear
	close all

	%global axe;

	addpath(genpath('../source/'))   %addpath �����SGDLibrary-masterĿ¼   genpath �Ƕ�ȡSGDLibrary-masterĿ¼������Ŀ¼
	%% ��ȡ�ļ�

	file1='../Datas/bun045.asc';
	file2='../Datas/bun000.asc';

	% file1='../Datas/rabbit.pcd';
	% file2='../Datas/rabbit.pcd';

	% file1='../Datas/rabbit.ply';
	% file2='../Datas/Scene3.ply';

	tic         %��ʱ��ʼ for Debug

	[P,Q]= readPointCloudDatas(file1,file2);

	toc         %��ʱ�������Զ���ӡ���е�ʱ�� for Debug
	% t=toc     % for Debug

	%���ƶ����ĵ���
	displayer = displayFunction;
	displayer.displayInitPointCloud(P,Q);


	%% ͨ��8����PCA���� ���������� [pn��qn]
	% PCA���������ƣ�ĳ������Э���������С����ֵ��Ӧ�����������õ㷨����
	k = 8;                                    %����ѡ����8����
	[pn,qn] = normalCaculate(P,Q,k);

	%����Ŀ����Ƶķ�����
	displayer.displayNormalOnSourcePointCloud(P,pn);

	save ../Datas/MatFiles/Normal.mat       % for Debug


	%% ��������ȡ   demo_1
	% load ../Datas/MatFiles/Normal.mat     % for Debug
	[p0,q0,fep,feq,feq0,n1,d1,n2,d2] = featurePoint(P,Q,pn,qn,k);

	save ../Datas/MatFiles/FP.mat           % for Debug


	%% ��������/����  demo_2
	% 1��PFH
	% load ../Datas/MatFiles/FP.mat         % for Debug

	% [vep,veq] = PFHCaculate(P,Q,p0,q0,fep,feq,pn,qn,n1,d1,n2,d2);

	% ����ĳ��PFH����
	% displayer.displayPFHOfKeyPoint(vep);

	% save ../Datas/MatFiles/PFHC.mat         % for Debug
	
	
	% 2��FPFH
    close all
	clear
	clc
	load ../Datas/MatFiles/FP.mat         % for Debug
	r_P = 0.005;
	r_Q = 0.005;

	%��������ת����400 * n תΪ 1*400*n Ԫ������
	% for i = 1:size(n1,1)
	   % idx_P{i} = n1(:,i)';
	   % dis_P{i} = d1(:,i)';
	   % idx_Q{i} = n2(:,i)';
	   % dis_Q{i} = d2(:,i)';
	% end
tic
	% [vep,veq] = FPFHCaculate(P,Q,pn,qn,fep,feq,r_P,r_Q,idx_P,dis_P,idx_Q,dis_Q);
	[vep,veq] = FPFHCaculate(P,Q,pn,qn,fep,feq,r_P,r_Q);
toc
	%����ĳ��FPFH����
    displayer = displayFunction;
	displayer.displayFPFHOfKeyPoint(vep);

	save ../Datas/MatFiles/FPFHC.mat         % for Debug


	%% demo_3
	%% ��ƥ���޳�
	% load ../Datas/MatFiles/PFHC.mat       % for Debug
	% load ../Datas/MatFiles/FPFHC.mat       % for Debug
	[p0,q0,feq,nv] = removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq);

	save ../Datas/MatFiles/RWM.mat          % for Debug


	%%  ����������
	%load ../Datas/MatFiles/RWM.mat         % for Debug
	RMSE(p0,q0)


	%% ����ƥ��/��׼
	% close all                             % for Debug
	% clear                                 % for Debug
	% clc                                   % for Debug
	% load ../Datas/MatFiles/RWM.mat        % for Debug
	[Q1,R_Coarse,T_Coarse] = coarseRegistration(P,Q,fep,feq,nv);

	save ../Datas/MatFiles/CR.mat           % for Debug


	%% ����׼
	close all
	clear
	clc
	load ../Datas/MatFiles/CR.mat           % for Debug
	[R_Final,T_Final] = fineRegistration(P,Q1);           % Q1 --transform--> P
	% [R_Final,T_Final] = icp(Q1',P');                    % Q1 --transform--> P
	% [R_Final,T_Final] = icp(Q',P');                     % Q1 --transform--> P
    
    save ../Datas/MatFiles/FR.mat           % for Debug


	%% ���յ���תƽ�ƾ���
    load ../Datas/MatFiles/FR.mat           % for Debug
	R = R_Final * R_Coarse;
	T = R_Final * T_Coarse + T_Final;
	H = [R,T;0 0 0 1];                       % P = H * Q  -->  P = R * Q + T

	displayer.displayFinalQ2P(P,R * Q + T * ones(1,size( Q , 2 )));


end


