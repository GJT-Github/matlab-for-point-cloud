function [p0,q0,fep,feq,feq0,n1,d1,n2,d2] = keyPointOfISS(P,Q, r, e1,e2,idx,dis)
% idx_feature = My_ISS(p, r, e1,e2,idx,dis)  : ISS内在签名特征点提取
%
% https://download.csdn.net/download/xxiaotouming/10325431?utm_medium=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&depth_1-utm_source=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&dest=https%3A%2F%2Fdownload.csdn.net%2Fdownload%2Fxxiaotouming%2F10325431&spm=1003.2020.3001.6616.4
% 输入参数
%    p    : 点云矩阵  3 * n
%    r    : 邻域半径  单位：m
%    e1   : 中间特征值与最大特征值之比的 阈值
%    e2   : 中间特征值与最小特征值之比的 阈值
%    idx  : 邻域点索引，元包数组，从小到大距离排序，第一个是邻域中心点，即查询点
%    dis  : 邻域点距离，元包数组，从小到大距离排序，第一个是邻域中心点，即查询点
%
% 输出参数
%   idx_feature  : 点云p的ISS特征点索引  p为3*n  idx_feature为 p 列索引
%
%
%
%
%  Author：https://download.csdn.net/download/xxiaotouming/10325431?utm_medium=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&depth_1-utm_source=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&dest=https%3A%2F%2Fdownload.csdn.net%2Fdownload%2Fxxiaotouming%2F10325431&spm=1003.2020.3001.6616.4
%          adiusted by GJT
%  E-mail：gjt0114@outlook.com  of GJT

% if nargin < 5
    % error('no bandwidth specified')
% end
% if nargin < 6
    % fep = My_ISS(P, r, e1,e2);
    % feq = My_ISS(Q, r, e1,e2);
% end
% if nargin == 7
  % fep = My_ISS(P, r, e1,e2,idx,dis);
  % feq = My_ISS(Q, r, e1,e2,idx,dis);
% end

switch nargin
  case 5
    fep = My_ISS(P, r, e1,e2);
    feq = My_ISS(Q, r, e1,e2);
  case 7
    fep = My_ISS(P, r, e1,e2,idx,dis);
    feq = My_ISS(Q, r, e1,e2,idx,dis);
  otherwise
    error('no bandwidth specified')    
end



	feq0 = feq;

	p0 = P(:,fep);
	q0 = Q(:,feq);

  [n1,d1] = knn(P);
  [n2,d2] = knn(Q);

end



function [n1,d1] = knn(P)
    [n1,d1] = knnsearch(transpose(P), transpose(P), 'k', 400);      %依次取各点 最近的400个点 ；
                                                                    %n1为返回的点的列数，按照距离递增排序；
                                                                    %d1为各点与该点的距离，递增排序；
                                                                    %结果为n*400阵
    n1=transpose(n1);                       %分别对n1，d1取转置为400*n阵
    d1=transpose(d1);

end