function [pn,qn] = normalCaculate(P,Q,k)      %
% [pn,qn] = normalCaculate(P,Q,k) ：PCA法求取两点云法向量
%      
%  输入参数  
%      P  ：目标点云  3 * n
%      Q  ：源点云    3 * n
%      k  ：k近邻域点数 标量
%
%
%  输出参数
%      pn  :目标点云法向量  3 * n
%      qn  :源点云法向量    3 * n  
%   
%  Author：GJT 
%  E-mail：gjt0114@outlook.com

	pn = lsqnormest(P, k);                  %调用求法向量子函数lsqnormest求P阵所有法向量
	qn = lsqnormest(Q, k);



