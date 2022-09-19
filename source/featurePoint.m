function [p0,q0,fep,feq,feq0,n1,d1,n2,d2]= featurePoint(P,Q,pn,qn,k)      %��������ȡ
% b = featurePoint(P,Q,pn,qn,k) ����ȡ������
%
%  �������                         
%     P ������Ŀ�����     3 * n
%     Q ������Դ����       3 * n
%     pn ��Ŀ����Ʒ�����  3 * n
%     qn ��Դ���Ʒ�����    3 * n
%
%  �������
%     p0 ��Ŀ�����������  3 * n 
%     q0 ��Դ����������    3 * n
%    
%     fep  ��Ŀ��������������� 1 * n
%     feq  : Դ��������������   1 * n
%    
%     pn   : Ŀ����Ʒ�����  3 * n
%     qn   ��Դ���Ʒ�����    3 * n
%    
%     n1   ��Ŀ����� 400�� ����� 400 * n
%     n2   ��Դ���� 400�� �����   400 * n
%       
%     d1   ��Ŀ����� 400������� ���� ��С�������� 400 * n
%     d2   ��Դ���� 400�������� ���� ��С�������� 400 * n
%
%  Author��GJT 
%  E-mail��gjt0114@outlook.com


%%
%%����������
pt=zeros(1,size(P,2));                  %����1��P��������0��  
qt=zeros(1,size(Q,2));

% Ŀ�����������������
[n1,d1] = knnsearch(transpose(P), transpose(P), 'k', 400);      %����ȡ���� �����400���� ��
                                                                %n1Ϊ���صĵ�����������վ����������
                                                                %d1Ϊ������õ�ľ��룬��������
                                                                %���Ϊn*400��
n1=transpose(n1);                       %�ֱ��n1��d1ȡת��Ϊ400*n��
d1=transpose(d1);


  %�����������ݾ���-P ����ÿһ���Ӧ8����� ����ֵ�D�Dpt
% % %for i=1:size(P,2)                                               % size(P,2)���ؾ���P������   
% % %    pt(1,i)=1/k*sum(abs(transpose(pn(:,n1(2:k+1,i)))*pn(:,i))); %n1(2:k+1,i)ȡn1���ؾ���ָ������е�i�е�2--k+1�У�ֵΪ������8*1����
% %                                                                  %transpose(pn(:,n1(2:k+1,i)))Ϊȡ��i���������8����ķ�������ת�ã�Ϊ8*3��
% %                                                                  %transpose(pn(:,n1(2:k+1,i)))*pn(:,i)Ϊ�ٳ�i�㷨����3*1,ֵΪ�������˻�Ϊ8*1
% %                                                                  %abs()Ϊȡ����ֵ������sum()Ϊ��ͺ�����
% %                                                                  %pt(1,i)Ϊ ��i����ķ������������8����ķ����� ��ˡ�ȡ����ֵ���ۼӡ�ȡƽ��ֵ��Ľ��������ptΪ1*40097          

for i=1:size(P,2)         
    temp_1 = n1(2:k+1,i);                                       %8�������������������ѯ�㣩    8*1
    temp_2 = pn(:,temp_1);                                      %8����㷨����                 3*8
    temp_3 = transpose(temp_2);                                 %                             8*3  
    temp_4 = temp_3 * pn(:,i);                                  %��ѯ�㷨��������������  �ڻ� ��ֵԽ��Խƽ̹�� 8*1 = 8*3 3*1
    temp_5 = abs(temp_4);                                          
    temp = sum(temp_5);                                         %                1*1
    pt(1,i)=1/k*temp;                                           %�õ���i�����8������������ֵ���������ڻ�����ֵ�͵ľ�ֵ�� 1*size(P,2)
end
clear temp temp_1 temp_2 temp_3 temp_4 temp_5


% Դ����������������

[n2,d2] = knnsearch(transpose(Q), transpose(Q), 'k', 400);

n2=transpose(n2);
d2=transpose(d2);

for i=1:size(Q,2)
    qt(1,i)=1/k*sum(abs(transpose(qn(:,n2(2:k+1,i)))*qn(:,i))); %������
end



%%ѡȡ������
ptt=find(pt<mean(pt));                  % mean(pt)��pt40097ƽ��ֵ������һ������   pttֵpt��ΪС��ƽ��ֵ����ָ�꣬������
qtt=find(qt<mean(qt));

p0=P(:,ptt);                            %��ȡP���� С������������ֵ �ĵ�
q0=Q(:,qtt);


%���ƴ���ȡ��������
displayer = displayFunction;
displayer.displayFirstPickKeyPoint(p0,q0);


fep=[];                                %���������վ���0��0�пվ���   ������������
feq=[];                    
  %����ptt��ÿһ��Ԫ�أ�ȡ�� �������������--fep��feq  ������
% % %   
% % %  %for i=ptt                           %��ʾptt��ΪС��ƽ��ֵ����ָ�� 
% % %    %   [~,I]=min(qt(1,n2(1:k+1,i)));  %n1(1:k+1,i)��ȡn1�е�i�е�ǰ9�������ָ�꣬
% % %                                       %����ȡ��pt�е�i�����������9����ķ��������ȡ����ֵ�ۼ�ȡƽ��ֵ
% % %                                       %min(pt(1,n1(1:k+1,i)))��ȡ������Сֵ����i�����������8����ķ��������ȡ����ֵ�ۼ�ȡƽ��ֵ����Сֵ����
% % %                                       %[~,I]��ʾ��������ж�������Ϊ����ǰ���ֵ������Сֵ���� ��ָ�� ��ֵ��I
% % %   


for i=ptt                              %��ʾptt��ΪС��ƽ��ֵ����ָ��
    temp   = n1(1:k+1,i);              %ptt��Ӧ�������������������ѯ�㣩           n1��1�о��ǲ�ѯ�㱾��       ������
    temp   = pt(1,temp);               %ptt��Ӧ��������������ֵ                                ������
    [~,I]  = min(temp);                %I����������ֵ��С��������

    if I==1                            %ɸѡ�� ��ѯ�� ����8������� ��������ֵ��С �ĵ���Ϊ �ؼ���
        fep=[fep,i];                   %��i��ֵ���fep����fepΪ1�ж���
    end
end
clear temp 


for i=qtt 
    [~,I]=min(qt(1,n2(1:k+1,i)));
    if I==1
        feq=[feq,i];
    end
end

feq0=feq;                             %fepΪP���������ڵ���P�е�����


p0=P(:,fep);                          %p0 Ϊ�������������
q0=Q(:,feq);


%����������ȡ��������
displayer.displayFinalPickKeyPoint(p0,q0);


%%%clear axe i data1 data2 file1 file2 I k
%%clearvars -except P Q pn qn n1 d1 d11 n2 d2 d22 fep feq feq0

%save PFH1.mat%���湤�������б���
end
