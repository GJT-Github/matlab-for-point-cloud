% 点云处理
%
% Author ：GJT
% E-mail ：gjt0114@outlook.com

function Main                                      % https://blog.csdn.net/weixin_37610397/article/details/80441523

	clc
	clear
	close all
    
	addpath(genpath('../source/'))                 % addpath 是添加SGDLibrary-master目录   genpath 是读取SGDLibrary-master目录所有子目录
	%% 读取文件
	[P,Q] = readPointCloud();

	%绘制读到的点云
	displayer = displayFunction;
	displayer.displayInitPointCloud(P,Q);          %P :红 Q:蓝

	% AABB包围盒计算
    % global longline;
    [P_bin,P_max,P_min,longline] = box(P);          % for Debug
   %[0.1558 0.1522 0.1174]    0.2474
    %分辨率计算
    [resolution_P]=computeCloudResolution(P);  % 0.00055 5.5540830e-04
    [resolution_Q]=computeCloudResolution(Q);  % 0.00055 5.4773246e-04

    %文献复现 2.1.1自适应点云平均距离计算
    global r_P_k r_Q_k;
    paper = paperISS;
	r_P_k = paper.paper(P) * 5;  %0.0083
	r_Q_k = paper.paper(Q) * 5;  %0.0075

    

	%% 通过8邻域PCA建立 法向量估计 [pn、qn]
	% PCA法向量估计：某点邻域协方差矩阵最小特征值对应特征向量即该点法向量
	k = 8;                                    %邻域选定：8邻域
	[pn,qn] = normalCaculate(P,Q,k);

	%绘制目标点云的法向量
	displayer.displayNormalOnSourcePointCloud(P,pn);

	% save ../Datas/MatFiles/Normal_asc.mat         % for Debug
    % save ../Datas/MatFiles/Normal_pcd.mat         % for Debug

	%% 特征点提取   demo_1
    % close all                                     % for Debug
    % clear all                                     % for Debug
    % clc                                           % for Debug
	% load ../Datas/MatFiles/Normal_asc.mat         % for Debug
    % load ../Datas/MatFiles/Normal_pcd.mat         % for Debug


	% 1、
	[p0,q0,fep,feq,feq0,n1,d1,n2,d2] = featurePoint(P,Q,pn,qn,k);
    

    % 2、ISS特征点  
    % Note：1、阈值选取？ 2、取得大多是边缘？ 3、横向对比
	r  = 0.05;                                 % 邻域半径
	e1 = 0.7;                                  % 中间特征值与最大特征值之比的 阈值
	e2 = 0.4;                                  % 中间特征值与最小特征值之比的 阈值  

	%   r  = 0.005;                            % 邻域半径
	% 	e1 = 0.6;                              % 中间特征值与最大特征值之比的 阈值
	% 	e2 = 0.3;  
	% for e1 =0.3:0.2:0.9                      % for Debug e1 和 e2 不同取值关键点影响
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



	%绘制最终提取的特征点
	% displayer.displayFinalPickKeyPoint(p0,q0);

	% save ../Datas/MatFiles/FP.mat           % for Debug


	%% 特征计算/描述  demo_2
	% 1、PFH
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

	% 绘制某点PFH描述
	% displayer.displayPFHOfKeyPoint(vep);

	% save ../Datas/MatFiles/PFHC.mat         % for Debug
	
	
	% 2、FPFH
	% clear all                               % for Debug
	% close all                               % for Debug
	% clc                                     % for Debug
	% load ../Datas/MatFiles/FP.mat           % for Debug
	r_P = 0.003;
	r_Q = 0.003;

	%近邻数据转换，400 * n 转为 1*400*n 元胞数组
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
	%绘制某点FPFH描述
    % displayer = displayFunction;             % for Debug
	displayer.displayFPFHOfKeyPoint(vep);

	% save ../Datas/MatFiles/FPFHC.mat         % for Debug


	%% demo_3
	%% 误匹配剔除
	% close all                               % for Debug
	% clear all                               % for Debug
	% clc                                     % for Debug
	% load ../Datas/MatFiles/PFHC.mat         % for Debug
	% load ../Datas/MatFiles/FPFHC.mat        % for Debug
	e_Delet_Distance  = 0.05;                 %0.05  for bun0*.asc
	e_RANSAC_Distance = 0.005;                %0.005 for bun0*.asc
	[p0,q0,feq,nv] = removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq,e_Delet_Distance,e_RANSAC_Distance);

	% save ../Datas/MatFiles/RWM.mat          % for Debug


	%%  均方根评价
	% load ../Datas/MatFiles/RWM.mat          % for Debug
	% RMSE(p0,q0)


	%% 特征匹配/配准
	% close all                             % for Debug
	% clear all                             % for Debug
	% clc                                   % for Debug
	% load ../Datas/MatFiles/RWM.mat        % for Debug
	[Q1,R_Coarse,T_Coarse] = coarseRegistration(P,Q,fep,feq,nv);
    disp(['coarse Registration RMSE:',num2str( RMSE(P,Q1))]);

% 	save ../Datas/MatFiles/CR.mat           % for Debug


	%% 精配准
	% close all                             % for Debug
	% clear all                             % for Debug
	% clc                                   % for Debug
	% load ../Datas/MatFiles/CR.mat         % for Debug

    [R_Final,T_Final] = finalResgistration(P,Q1);

%     save ../Datas/MatFiles/FR.mat          % for Debug


	%% 最终的旋转平移矩阵
    % load ../Datas/MatFiles/FR.mat          % for Debug
	R = R_Final * R_Coarse;
	T = R_Final * T_Coarse + T_Final;

	Q1_temp = R_Coarse * Q + T_Coarse * ones( 1 , size( Q , 2 ) );
    Q2 = R * Q + T * ones( 1 , size( Q , 2 ) );
% 	H = [R,T;0 0 0 1];                     % P = H * Q  -->  P = R * Q + T

	displayer.displayFinalQ2P(P,Q1_temp);
    title('粗配准结果')
	displayer.displayFinalQ2P(P,Q2);

end




%读点云数据文件
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
    

	tic         %计时开始 for Debug
	[P,Q]= readPointCloudDatas(file1,file2);
	toc         %计时结束，自动打印运行的时间 for Debug

%     r_cmatrix = ones(size(P')).*[1 0 0];
%     b_cmatrix = ones(size(Q')).*[0 0 1];
%     S1 = pointCloud(P','Color',r_cmatrix);
%     Q0 = pointCloud(Q','Color',b_cmatrix);
%     pcshow(S1);
%     hold on
%     pcshow(Q0);

    disp(['origin RMSE:',num2str(RMSE(P,Q))]);

end

%点云分辨率计算
function [resolution]=computeCloudResolution(cloud)
% cloud 3 * n
%
	NS = createns(cloud','NSMethod','kdtree');
	[~,dis] = knnsearch(NS,cloud','k',2);

	%距离较大的10%的点作为噪声点剔除掉，计算分辨率
	dis_nearest = sort(dis(:,2));
	dis_nearest(end-round(size(dis_nearest,1)*0.1):end)=[];  
    resolution = mean(dis_nearest);
end


function [R_Final,T_Final]=finalResgistration(targetPointCloud,sourcePointCloud)
% 3种icp精配准

    P = targetPointCloud;
    Q1 = sourcePointCloud;

    % 1.
	[R_Final,T_Final] = fineRegistration(P,Q1);           % Q1 --transform--> P

    % 2.
	% [R_Final,T_Final] = icp(Q1',P');                    % Q1 --transform--> P
	% [R_Final,T_Final] = icp(Q',P');                     % Q1 --transform--> P

    % 3.自带ICP剔除内点率和不剔除内点的近邻搜索全部用的原始点数据，内点率只对计算RMSE有效
%     r_point_cmatrix = ones(size(P')).*[1 0 0];
%     b_point_cmatrix = ones(size(Q1')).*[0 0 1];
%     P_temp = pointCloud(P',"Color",r_point_cmatrix);
%     Q_temp = pointCloud(Q1',"Color",b_point_cmatrix);
%     [H_Final,transformedCloud,rmse] = pcregistericp(P_temp,Q_temp,'Verbose',true,'Metric','pointToPlane','InlierRatio',0.95,'Tolerance',[0.00001,0.002],'MaxIterations',100);
%     R_Final = H_Final.Rotation;
%     T_Final = H_Final.Translation';


end