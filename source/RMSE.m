function  [E_rmse]= RMSE(p0,q0)      %
% [E_rmse]= RMSE(p0,q0) �����������������ָ��
%      
%  �������  
%  p0      ��Ŀ�����
%  q0      ����������
%
%  �������
%  E_rmse  ������������ֵ
%   
%
%  Author��GJT 
%  E-mail��gjt0114@outlook.com

% disdence=p0(1,:)-q0(1,:);                   //norm(Q(:,feq(i))-P(:,fep(nv(1,i))))  ������
num_of_points = size(p0(1,:));

for i=1:num_of_points(2)
    disdence(1,i)= norm(p0(:,i)-q0(:,i));
end

E_rmse = sqrt((disdence * disdence')/num_of_points(2));




