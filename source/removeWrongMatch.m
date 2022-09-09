function [p0,q0,feq,nv]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq)      %
% [p0,q0,feq]= removeWrongMatch(P,Q,p0,q0,fep,feq,feq0,vep,veq) ���޳���ƥ���
%      
%  �������  
%      ��
%
%
%  �������
%      ��
%   


% load PFH2.mat

%����ֱ��ͼ ����������ƥ���ϵ ���� �����Բ���Լ��
[nv,d]=knnsearch(vep',veq');           %vep',veq'ȡ��Ӧ������ת�ã�knnsearch(X, Y) ����������X���ҵ��ֱ�����������Y ÿ�������� ����� �ھ�����nv������d
nv=nv';                                %��ת��
d=d';

%% ��ʾģ���볡�������㼰�� ����� ����
%p0=P(:,fep);
%q0=Q(:,feq);

% ���� Ŀ����� �� Դ����
%figure;
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
view(3)



%% �޳� �����Զ�Ķ�Ӧ��
dist=zeros(size(nv));                 %����0���󣬴�С���ھ���nv
for i=1:length(nv)
    dist(1,i)=norm(Q(:,feq(i))-P(:,fep(nv(1,i))));%������Q(:,feq(i))��ʾQ�����еĵ�i���ؼ��㣬P(:,fep(nv(1,i))��ʾP�����о���Q�е�i�ؼ��������
end

feq(dist>0.05)=[];                    %��feq��ɾ���������0.05������㣨�ؼ��㣩��ָ��
nv(dist>0.05)=[];                     %��nv��ɾ���������0.05�����������ӵ�ָ��


%���� ɾ���������0.05���ͼ
p0=P(:,fep);
q0=Q(:,feq0);

%figure;
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
title('ȥ���������0.05�����������');
view(3)



%% ���Բ���Լ��
num=zeros(size(feq));                 %����ͬfeq��С���������������
for i=1:length(nv)
    a=0;
    for j=1:length(nv)
        if i==j
            continue                  %������䣬ֱ�ӿ�ʼ��һ��ѭ��
        end
        dq=norm(Q(:,feq(i))-Q(:,feq(j)));         %norm��Q�� ��i���ؼ������j���ؼ���� ������  
        dp=norm(P(:,fep(nv(i)))-P(:,fep(nv(j)))); %norm��P�о���Q�е�i���ؼ�������������Q�е�j���ؼ��������Ķ�����
        if abs(dp-dq)/(dp+dq)<0.02    %absȡ����ֵ�������ж�(dp-dq)/(dp+dq)<0.02
            a=a+1;                    %�ڵ�i���ؼ����£����������ļ���һ��
        end
    end
    num(1,i)=a;
end
num1=sort(num,'descend');             %sort���� descend�����ս�������
feq(num<num1(10))=[];                 %����num<num1(10)����ɾ�����������Ĺؼ���ָ��
nv(num<num1(10))=[];                  %����num<num1(10)����ɾ������������������ָ��

%��ͼ������feq��nv��Ĺؼ��㼰������
p0=P(:,fep);
q0=Q(:,feq0);

%figure;
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
view(3)

%ʹ���������һ�����㷨RANSACȷ��ƥ���ϵ
aa=500;                               %��������
b0=0;
c0=zeros(size(feq));
while aa
    n=length(nv);
    a=randperm(n);
    feq1=feq(a(1:3));
    nv1=nv(a(1:3));
    
    [R,T] = Quater_Registration(Q(:,feq1)',P(:,fep(nv1))');
    Q0 = R * Q(:,feq) + repmat(T,1,n);
    
    dist=Q0-P(:,fep(nv));
    b=0;
    c=zeros(1,n);
    for i=1:n
%         if norm(dist(:,i))<0.0012
        if norm(dist(:,i))<0.005
            b=b+1;
            c(i)=1;
        end
    end
    if b>b0
        b0=b;
        c0=c;
    end
    aa=aa-1;
end
feq(c0<1)=[];
nv(c0<1)=[];

p0=P(:,fep);
q0=Q(:,feq0);

%figure;
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
view(3)
