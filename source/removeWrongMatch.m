function [p0,q0,feq,nv]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq)      %
% [p0,q0,feq]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq) ���޳���ƥ���
%      
%  �������  
%    P       ��ģ����� 3 * n
%    Q       ���������� 3 * n
%    p0      ��ģ����Ƶ������㼯��      3 * n 
%    q0      ���������Ƶ������㼯��      3 * n
%    fep     ��ģ����������������      1 * n
%    feq     ���������������������      1 * n
%    feq0    �������������������������  1 * n
%    vep     �����������������PFH����   64 * n
%    veq     �����������������PFH����   64 * n
%  
%
%  �������
%    p0      ���޳���ƥ���ʣ����ȷƥ���Ӧ��,ģ����������� �޳��� 3 * n
%    q0      ���޳���ƥ���ʣ����ȷƥ���Ӧ��,�������������� �޳��� 3 * n
%    feq     ���޳���ƥ���ʣ����ȷƥ���Ӧ��,�������������� �޳��� 1 * n
%    nv      ��PFH�����е�����ڵ����� �޳���  1 * n
%
%  Author��GJT 
%  E-mail��gjt0114@outlook.com


% load PFH2.mat


%% ����ֱ��ͼ ����������ƥ���ϵ ���� �����Բ���Լ��
[nv,d]=knnsearch(vep',veq');           %vep',veq'ȡ��Ӧ������ת�ã�knnsearch(X, Y) ����������X���ҵ��ֱ�����������Y ÿ�������� ����� �ھ�����nv������d
nv=nv';                                %��ת��
d=d';

%% ��ʾģ���볡�������㼰�� ����� ����
displayer = displayFunction;
displayer.displayPointCloudAndLine(P,Q,p0,q0,fep,feq,nv);


%% 1���޳� �����Զ�Ķ�Ӧ��
[p0,q0,feq,nv] = DeleteDisdence(P,Q,fep,feq,feq0,nv);

%���� ɾ���������0.05���ͼ
displayer.displayDeleteDisdencePointCloudAndLine(P,Q,p0,q0,fep,feq,nv);


%% 2�����Բ���Լ��
[p0,q0,feq,nv] = RigidInvariantConstraints(P,Q,fep,feq,feq0,nv);

%��ͼ������feq��nv��Ĺؼ��㼰������
displayer.displayRigidInvariantConstraintsPointCloudAndLine(P,Q,p0,q0,fep,feq,nv);


%% 3��ʹ���������һ�����㷨RANSACȷ��ƥ���ϵ
aa = 500;    %��������
[p0,q0,feq,nv] = RANSAC(P,Q,fep,feq,feq0,aa,nv);

%RANSAC�޳���ƥ���Ľ������
displayer.displayRANSACPointCloudAndLine(P,Q,p0,q0,fep,feq,nv);


