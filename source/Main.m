clc
clear
close all


addpath(genpath('../source/'))   %addpath �����SGDLibrary-masterĿ¼   genpath �Ƕ�ȡSGDLibrary-masterĿ¼������Ŀ¼
%% ��ȡ�ļ�

file1='../Datas/bun045.asc';
file2='../Datas/bun000.asc';

data1 = ascread(file1);      %��ȡasc���͵��ƣ�ֵΪ2��1�е�cell�������п��Բ�ͬ���ͣ�{1}Ϊ����40097points��{2}Ϊ3��40097���������
data2 = ascread(file2);      %��ȡasc���͵��ƣ�ֵΪ2��1�е�cell�������п��Բ�ͬ���ͣ�{1}Ϊ����40097points��{2}Ϊ3��40256���������
 
P = data1{2};                %PΪ3��40097�о��󣬵���data1����cell���ڶ���
Q = data2{2};


% ���ƶ����ĵ���
figure;                                  %�������ĵ���ͼ
axe(1)=subplot(221);
plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot��ͼ�������ֱ�ȡP�е�1.2.3�����е���Ϊ�����ᣬr��ʾ��ɫ
hold on
plot3(Q(1,:),Q(2,:),Q(3,:),'b.');
title('ģ������볡�����Ƴ�ʼλ��')
view(3)

%% ͨ��8����PCA���� ���������� [pn��qn]
k=8;                                    %����ѡ����8����

pn = lsqnormest(P, k);                  %�����������Ӻ���lsqnormest��P�����з�����
qn = lsqnormest(Q, k);


%% ��������ȡ   demo_1
[p0,q0,fep,feq,feq0,n1,d1,n2,d2] = featurePoint(P,Q,pn,qn,k);

%% PFH��������/����  demo_2
[vep,veq] = PFHCaculate(P,Q,p0,q0,fep,feq,pn,qn,n1,d1,n2,d2);

%% demo_3
%%��ƥ���޳�
[p0,q0,feq,nv]=removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq);

save main.mat

%% ����������
load main.mat
RMSE(p0,q0)


%%����ƥ��/��׼
registration(P,Q,fep,feq,nv);


