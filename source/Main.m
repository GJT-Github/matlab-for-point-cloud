

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

tic         %��ʱ��ʼ

[P,Q]= readPointCloudDatas(file1,file2);

toc         %��ʱ�������Զ���ӡ���е�ʱ��
% t=toc   

%���ƶ����ĵ���
% displayInitPointCloud(P,Q);
displayer = displayFunction;
displayer.displayInitPointCloud(P,Q);


%% ͨ��8����PCA���� ���������� [pn��qn]
% PCA���������ƣ�ĳ������Э���������С����ֵ��Ӧ�����������õ㷨����
k=8;                                    %����ѡ����8����

pn = lsqnormest(P, k);                  %�����������Ӻ���lsqnormest��P�����з�����
qn = lsqnormest(Q, k);

%����Ŀ����Ƶķ�����
displayer.displayNormalOnSourcePointCloud(P,pn);


%% ��������ȡ   demo_1
[p0,q0,fep,feq,feq0,n1,d1,n2,d2] = featurePoint(P,Q,pn,qn,k);

%% PFH��������/����  demo_2
[vep,veq] = PFHCaculate(P,Q,p0,q0,fep,feq,pn,qn,n1,d1,n2,d2);
save main.mat
%% demo_3
%% ��ƥ���޳�
% load main.mat
[p0,q0,feq,nv] = removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq);

%save main.mat

%% ����������
%load main.mat
RMSE(p0,q0)


%% ����ƥ��/��׼
coarseRegistration(P,Q,fep,feq,nv);


%% ����׼
fineRegistration();


end








%% ��ͼ��װ
%�����ƻ���
% function [] = displayInitPointCloud(P,Q)
%     global axe;
%     global posionFigureX;
%     global posionFigureY;
%     global posionFigureZ;
%     global posionFigureN;
%     posionFigureX = 10;
%     posionFigureY = 350;
%     posionFigureZ = 500;
% 	posionFigureN = 400;

% 	figure(1);                               %�������ĵ���ͼ
% 	% set(gcf,'position',[10 350 500 400]);
% 	set(gcf,'position',[posionFigureX,posionFigureY,posionFigureZ,posionFigureN]);
% 	axe(1)=subplot(221);
% 	plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot��ͼ�������ֱ�ȡP�е�1.2.3�����е���Ϊ�����ᣬr��ʾ��ɫ
% 	hold on
% 	plot3(Q(1,:),Q(2,:),Q(3,:),'b.');
% 	title('ģ������볡�����Ƴ�ʼλ��')
% 	view(3)
% end

% %����������
% function [] = displayNormalOnSourcePointCloud(P,normal)
%     global axe;
%     global posionFigureX;
%     global posionFigureY;
%     global posionFigureZ;
%     global posionFigureN;
% 	figure(2);
%     set(gcf,'position',[posionFigureX + 510,posionFigureY,posionFigureZ,posionFigureN]);
% 	plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot��ͼ�������ֱ�ȡP�е�1.2.3�����е���Ϊ�����ᣬr��ʾ��ɫ
% 	hold on
% 	quiver3( P(1,:) , P(2,:) , P(3,:)  ,  normal(1,:) , normal(2,:) , normal(3,:) ,'g');
% 	xlabel('x');ylabel('y');zlabel('z');
% 	title('Դ���Ʒ�������ʾ');
% end
