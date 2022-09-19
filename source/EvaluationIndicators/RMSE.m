function  [E_rmse]= RMSE(p0,q0)      %
% [E_rmse]= RMSE(p0,q0) �����������������ָ��
%      
%  �������  
%  p0      ��Ŀ����� 3 * n
%  q0      ���������� 3 * n
%
%  �������
%  E_rmse  ������������ֵ
%   
%
%  Author��GJT 
%  E-mail��gjt0114@outlook.com

% 1��
% disdence=p0(1,:)-q0(1,:);                   //norm(Q(:,feq(i))-P(:,fep(nv(1,i))))  ������
if size(p0(1,:),2) > size(q0(1,:),2)
	num_of_points = size(q0(1,:),2);
else
	num_of_points = size(p0(1,:),2);
end

% num_of_points = size(p0(1,:));

for i=1:num_of_points
    disdence(1,i)= norm(p0(:,i)-q0(:,i));
end

E_rmse = sqrt((disdence * disdence')/num_of_points);


% % 2��p0��q0ά��Ӧ����ͬ
% E_rmse = sqrt( mean( sum( (p0-q0).^2,1 ) ) );




