function [p0,q0,feq,nv]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq)      %
% [p0,q0,feq]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq) ���޳���ƥ���
%      
%  �������  
%    P       ��ģ�����
%    Q       ����������
%    p0      ��ģ����Ƶ������㼯��
%    q0      ���������Ƶ������㼯��
%    fep     ��ģ����������������
%    feq     ���������������������
%    feq0    �������������������������
%    vep     �����������������PFH����
%    veq     �����������������PFH����
%  
%
%  �������
%    p0      ���޳���ƥ���ʣ����ȷƥ���Ӧ��,ģ����������� �޳��� 
%    q0      ���޳���ƥ���ʣ����ȷƥ���Ӧ��,�������������� �޳���
%    feq     ���޳���ƥ���ʣ����ȷƥ���Ӧ��,�������������� �޳���
%    nv      ��PFH�����е�����ڵ����� �޳���
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


