function [feq,nv] = RigidInvariantConstraints(P,Q,fep,feq,feq0,nv)
%% [p0,q0,feq,nv] = RigidInvariantConstraints(P,Q,fep,feq,feq0,nv)
%                   :   ���Բ���Լ��
%
%  �������  
%    P       ��ģ�����
%    Q       ����������
%    fep     ��ģ����������������
%    feq     ���������������������
%    feq0    �������������������������
%    nv      : PFH�����е�����ڵ�����
%
%  �������
%    p0      ��ģ����Ƶ������㼯�� �޳��� 
%    q0      ���������Ƶ������㼯�� �޳���
%    feq     ��������������������� �޳���
%    nv      : PFH�����е�����ڵ����� �޳���
%   
%  Author��GJT 
%  E-mail��gjt0114@outlook.com



num = zeros(size(feq));                 %����ͬfeq��С���������������

for i = 1:length(nv)
    a =0;
    for j = 1:length(nv)
        if i == j
            continue                  
        end
        dq = norm(Q(:,feq(i))-Q(:,feq(j)));         %norm��Q�� ��i���ؼ������j���ؼ���� ������  
        dp = norm(P(:,fep(nv(i)))-P(:,fep(nv(j)))); %norm��P�о���Q�е�i���ؼ�������������Q�е�j���ؼ��������Ķ�����
        if abs(dp-dq)/(dp+dq)<0.02        %absȡ����ֵ�������ж�(dp-dq)/(dp+dq)<0.02
            a = a + 1;                    %�ڵ�i���ؼ����£����������ļ���һ��
        end
    end
    num(1,i) = a;
end

num1 = sort(num,'descend');              %sort���� descend�����ս�������

% feq(num<num1(10)) = [];                  %����num<num1(10)����ɾ�����������Ĺؼ���ָ��
% nv(num<num1(10))  = [];                  %����num<num1(10)����ɾ������������������ָ��

feq(num<num1(10)) = [];                  %����num<num1(10)����ɾ�����������Ĺؼ���ָ��
nv(num<num1(10))  = [];                  %����num<num1(10)����ɾ������������������ָ��

% p0 = P(:,fep);
% q0 = Q(:,feq0);

end
