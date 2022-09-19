function [P_FPFH,Q_FPFH] = FPFHCaculate(P,Q,pn,qn,fep,feq,r_P,r_Q,idx_P,dis_P,idx_Q,dis_Q)      %
% [P_FPFH,Q_FPFH] = FPFHCaculate(P,Q,pn,qn,fep,feq,r_P,r_Q,idx_P,dis_P,idx_Q,dis_Q) ： 计算两点云集合的FPFH特征描述
%      
%  输入参数  
%     P      : 目标点云  3 * n
%     Q      : 源点云    3 * n
%     pn     : 目标点云法向量  3 * n
%     qn     : 源点云法向量    3 * n
%     fep    : 源点云特征点   1 * n
%     feq    : 目标点云特征点 1 * n
%     r_P    : 目标点云邻域半径，邻域搜索用      
%     r_Q    : 源点云邻域半径，邻域搜索用
%     idx_P  : 目标点云邻域索引    1 * n    |    
%     dis_P  : 目标点云邻域距离    1 * n    |
%     idx_Q  : 源点云邻域索引      1 * n    |    缺省则用KDtree 搜索r邻域点
%     dis_Q  : 源点云邻域距离      1 * n    |
%
%
%  输出参数
%      ：
%   
%  Author：GJT 
%  E-mail：gjt0114@outlook.com

	% P_FPFH = fpfhdescriptor(P,pn,fep,r_P,idx_P,dis_P);
	% Q_FPFH = fpfhdescriptor(Q,qn,feq,r_Q,idx_Q,dis_Q);



  %   if nargin < 8 && nargin > 12     %nargin判断变量个数
  %       error('no bandwidth specified')
  %   end

  %   if nargin < 9
		% P_FPFH = fpfhdescriptor(P,pn,fep,r_P);
		% Q_FPFH = fpfhdescriptor(Q,qn,feq,r_Q);
  %   end
  %   if nargin == 12
		% P_FPFH = fpfhdescriptor(P,pn,fep,r_P,idx_P,dis_P);
		% Q_FPFH = fpfhdescriptor(Q,qn,feq,r_Q,idx_Q,dis_Q);
  %   end

    if nargin == 8
    	P_FPFH = fpfhdescriptor(P,pn,fep,r_P);
  		Q_FPFH = fpfhdescriptor(Q,qn,feq,r_Q);
    elseif nargin == 12
    	P_FPFH = fpfhdescriptor(P,pn,fep,r_P,idx_P,dis_P);
  		Q_FPFH = fpfhdescriptor(Q,qn,feq,r_Q,idx_Q,dis_Q);
    else
    	error('no bandwidth specified')
    end


	










