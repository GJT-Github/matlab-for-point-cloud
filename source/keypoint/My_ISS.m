function idx_feature = My_ISS(p, r, e1,e2,idx,dis)
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

% 输入参数判别
if nargin < 4
    error('no bandwidth specified')
end
if nargin < 5
    Mdl = createns(p','NSMethod','kdtree','Distance','minkowski','p',2);
    [idx,dis] = rangesearch(Mdl,p',r);
end

numpts = size(p,2);           %总点数
flag = zeros(1,numpts);       %特征点的标识 1 * n
% tzz = zeros(size(p));

for i = 1:numpts
    if length(idx{i})<2
        continue
    end
    x = p(:,idx{i}(2:end));   %r邻域点坐标
    w = 1./dis{i}(2:end);
    p_bar = p(:,i);           % 中心点坐标
    P = repmat(w,3,1).*(x - repmat(p_bar,1,size(x,2))) * ...
        transpose(x - repmat(p_bar,1,size(x,2)));             %spd matrix P
    P = P./sum(w);
    % if any(isnan(P(:)))
    %     save debug.mat 
    % end
    [~,D] = eig(P);
    lam = sort(abs(diag(D)),'descend');                       % 三个特征值由大到小排列
    if lam(2)/lam(1)<=e1 &&lam(3)/lam(2)<e2
        flag(i)=1;
    end
    % tzz(:,i)=lam;
end

% tzz(1,:)=tzz(2,:)./tzz(1,:);
% tzz(2,:)=tzz(3,:)./tzz(2,:);
% tzz(3,:)=[];
idx_feature = find(flag);               %find查找非零元素的索引，3*n 列索引
end