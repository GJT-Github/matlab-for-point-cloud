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
	min_flag = 1;
else
	num_of_points = size(p0(1,:),2);
	min_flag = 2;
end

% num_of_points = size(p0(1,:));


% for i=1:num_of_points
%     disdence(1,i)= norm(p0(:,i)-q0(:,i));
% end

% E_rmse = sqrt((disdence * disdence')/num_of_points);

if min_flag == 1
	NS = createns(p0','NSMethod','kdtree');
	[idx_border,dis_border] = knnsearch(NS,q0','k',5);
% [idx_border,dis_border] = knnsearch(p0',q0','k',5);

else
	NS = createns(q0','NSMethod','kdtree');
	[idx_border,dis_border] = knnsearch(NS,p0','k',5);
end

E_rmse = sqrt(mean(dis_border(:,1).^2));

% % 2��p0��q0ά��Ӧ����ͬ
% E_rmse = sqrt( mean( sum( (p0-q0).^2,1 ) ) );




