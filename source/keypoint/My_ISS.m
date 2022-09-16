function idx_feature = My_ISS(p, r, e1,e2,idx,dis)
% idx_feature = My_ISS(p, r, e1,e2,idx,dis)  : ISS����ǩ����������ȡ
%
% https://download.csdn.net/download/xxiaotouming/10325431?utm_medium=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&depth_1-utm_source=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&dest=https%3A%2F%2Fdownload.csdn.net%2Fdownload%2Fxxiaotouming%2F10325431&spm=1003.2020.3001.6616.4
% �������
%    p    : ���ƾ���  3 * n
%    r    : ����뾶  ��λ��m
%    e1   : �м�����ֵ���������ֵ֮�ȵ� ��ֵ
%    e2   : �м�����ֵ����С����ֵ֮�ȵ� ��ֵ
%    idx  : �����������Ԫ�����飬��С����������򣬵�һ�����������ĵ㣬����ѯ��
%    dis  : �������룬Ԫ�����飬��С����������򣬵�һ�����������ĵ㣬����ѯ��
%
% �������
%   idx_feature  : ����p��ISS����������  pΪ3*n  idx_featureΪ p ������
%
%
%
%
%  Author��https://download.csdn.net/download/xxiaotouming/10325431?utm_medium=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&depth_1-utm_source=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&dest=https%3A%2F%2Fdownload.csdn.net%2Fdownload%2Fxxiaotouming%2F10325431&spm=1003.2020.3001.6616.4
%          adiusted by GJT
%  E-mail��gjt0114@outlook.com  of GJT

% ��������б�
if nargin < 4
    error('no bandwidth specified')
end
if nargin < 5
    Mdl = createns(p','NSMethod','kdtree','Distance','minkowski','p',2);
    [idx,dis] = rangesearch(Mdl,p',r);
end

numpts = size(p,2);           %�ܵ���
flag = zeros(1,numpts);       %������ı�ʶ 1 * n
% tzz = zeros(size(p));

for i = 1:numpts
    if length(idx{i})<2
        continue
    end
    x = p(:,idx{i}(2:end));   %r���������
    w = 1./dis{i}(2:end);
    p_bar = p(:,i);           % ���ĵ�����
    P = repmat(w,3,1).*(x - repmat(p_bar,1,size(x,2))) * ...
        transpose(x - repmat(p_bar,1,size(x,2)));             %spd matrix P
    P = P./sum(w);
    % if any(isnan(P(:)))
    %     save debug.mat 
    % end
    [~,D] = eig(P);
    lam = sort(abs(diag(D)),'descend');                       % ��������ֵ�ɴ�С����
    if lam(2)/lam(1)<=e1 &&lam(3)/lam(2)<e2
        flag(i)=1;
    end
    % tzz(:,i)=lam;
end

% tzz(1,:)=tzz(2,:)./tzz(1,:);
% tzz(2,:)=tzz(3,:)./tzz(2,:);
% tzz(3,:)=[];
idx_feature = find(flag);               %find���ҷ���Ԫ�ص�������3*n ������
end