function [R_final,T_final]=icp(data1,data2)
%%
%   data2 = R * data1 + repmat(T,1,controldatanum)
%  �������
%    data1    : Դ����     n * 3
%    data2    : Ŀ�����   n * 3
%
%  �������
%    R_final  : ���յ���ת����  3 * 3
%    T_final  : ���յ�ƽ�ƾ���  3 * 1
%
%
%Ѱ�ҵı任��ϵdata2=Rdata1+T
% https://download.csdn.net/download/weixin_42663213/86112432?utm_medium=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-11-86112432-download-86199821.dl_default_comparev1&depth_1-utm_source=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-11-86112432-download-86199821.dl_default_comparev1&dest=https%3A%2F%2Fdownload.csdn.net%2Fdownload%2Fweixin_42663213%2F86112432&spm=1003.2020.3001.6616.19
%%
%��������ѡȡ���Ƶ�
%data1=load('a.dat');
%data2=load('b.dat');

e_iteration = 0.00001;

mean_e=0;

figure(1);
plot3(data1(:,1),data1(:,2),data1(:,3),'r.');
hold on;
plot3(data2(:,1),data2(:,2),data2(:,3),'b.');
% title('ԭʼ����');
axis tight equal;
hold off;

% 
% mean_e=0;
[m,~]=size(data2);
controldata1=data1;%ѡȡ���Ƶ�
[controldatanum,~]=size(controldata1);
controldata2=zeros(controldatanum,3);
%%
%��ʼ��
R=[1,0,0;0,1,0;0,0,1];
T=[0,0,0];
last_E=0;
iteration=100;
R_Intermediate=zeros(3,3,iteration);
T_Intermediate=zeros(3,1,iteration);
delta_Intermediate=zeros(iteration,1);
index=zeros(controldatanum,1);
e_Intermediate=zeros(iteration,1);

%%
%����
for iter=1:iteration
  %Ѱ�ҿ��Ƶ�Ķ�Ӧ��
  for i=1:controldatanum
    temp_data1=repmat(controldata1(i,:),m,1);
    diff=sqrt(sum((temp_data1-data2).^2,2));
    [minvalue,index(i,1)]=min(diff);
    controldata2(i,:)=data2(index(i,1),:);
  end

  %%
  %����ȷ���Ĺ�ϵ�����RT
  centroid1=mean(controldata1);
  centroid2=mean(controldata2);
  demeancontroldata1=controldata1-repmat(centroid1,controldatanum,1);
  demeancontroldata2=controldata2-repmat(centroid2,controldatanum,1);
  H=demeancontroldata1'*demeancontroldata2;
  [U,S,V]=svd(H);
  R=V*U';
  T=(centroid2-centroid1)';
  R_Intermediate(:,:,iter)=R;
  T_Intermediate(:,:,iter)=T;

  %%
  %�������õ���RT����任֮��ĵ�
  controldata1=R * controldata1' + repmat(T,1,controldatanum);

  controldata1=controldata1';%�µĿ��Ƶ�
  % E=norm(controldata1-controldata2,2);
  indice=knnsearch(controldata2,controldata1);
  % diff=sqrt(sum((temp_data1-data2).^2,2));
  diff=zeros(controldatanum,1);
  for i=1:controldatanum
     diff(i,1)=sqrt( sum( ( controldata1( i,: )-controldata2( indice(i,1),: ) ).^2,2 ) ); 
  end

  E=sum(diff);
  e_Intermediate(iter,1)=E/controldatanum;
  delta=abs(E-last_E)/controldatanum ;%�м���������
  delta_Intermediate(iter,1)=delta;
  if(delta<e_iteration)
    mean_e=e_Intermediate(iter,1)
    disp('-----------------------------------------------------');
    disp('2�����ε�������ֵС����ֵ�����������ˣ���������');
    disp('  ��������ֵ��ֵ��');
    disp(num2str(e_iteration));
    disp('  ���δε�����');
    disp(num2str(delta));
    break;
  end
  last_E=E;
  iter               %$��ʾ��������

  displayer = displayFunction;                                          %$��ʾ��������
  displayer.displayProcessOfICP(controldata2',controldata1',iter,5);    %$

end

if iter == iteration
  disp('-----------------------------------------------------');
  disp('1�����������ﵽ���ֵ��');
end

figure(2);                                     %
plot(1:iter,delta_Intermediate(1:iter,1)');    %
xlabel('��������');ylabel('delta');            %
figure(3);                                     %
plot(1:iter,e_Intermediate(1:iter,1)');        %
xlabel('��������');ylabel('loss');             %

% e=0;
% for i=1:iter
%    e = e+ e_Intermediate(i,1); 
% end

%%
%�������յ�R��T
temp_R=eye(3);
temp_T=zeros(3,1);
for i=1:iter
   temp_R=R_Intermediate(:,:,i)*temp_R; 
   temp_T=R_Intermediate(:,:,i)*temp_T+T_Intermediate(:,:,i);
end
R_final=temp_R
T_final=temp_T

data1_transformed=R_final*data1'+repmat(T_final,1,size(data1,1));

data1_transformed=data1_transformed';

figure(4);
plot3(data1_transformed(:,1),data1_transformed(:,2),data1_transformed(:,3),'r.')
hold on;
plot3(data2(:,1),data2(:,2),data2(:,3),'b.')

indice1=knnsearch(data2,data1_transformed);
diff1=zeros(controldatanum,1);
for i=1:controldatanum
   diff1(i,1)=sqrt(sum((data1_transformed(i,:)-data2(indice1(i,1),:)).^2,2)); 
end
title('ICP results')
axis equal tight;
end

% hold off;
%  save data.asc -ascii data1_transformed;