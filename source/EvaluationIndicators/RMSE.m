function  [E_rmse]= RMSE(p0,q0)      %
% [E_rmse]= RMSE(p0,q0) ：计算均方根，评价指标
%      
%  输入参数  
%  p0      ：目标点云 3 * n
%  q0      ：场景点云 3 * n
%
%  输出参数
%  E_rmse  ：均方根评价值
%   
%
%  Author：GJT 
%  E-mail：gjt0114@outlook.com

% 1、
% disdence=p0(1,:)-q0(1,:);                   //norm(Q(:,feq(i))-P(:,fep(nv(1,i))))  二范数
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


% % 2、p0与q0维度应当相同
% E_rmse = sqrt( mean( sum( (p0-q0).^2,1 ) ) );




