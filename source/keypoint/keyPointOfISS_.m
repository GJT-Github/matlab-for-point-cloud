%����
function [p0,q0,fep,feq,feq0,n1,d1,n2,d2] = keyPointOfISS_(P,Q, r, e1,e2,idx,dis)
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

  % global r_P_k r_Q_k;
  [n1,d1,r_P_k] = knn(P);
  [n2,d2,r_Q_k] = knn(Q);

  displayer = displayFunction;                %for Debug
  
  % for i=1:15                                %for Debug

    i=4;
    switch nargin
      case 5
        fep = My_ISS(P, r_P_k * i, e1,e2);
        feq = My_ISS(Q, r_Q_k * i, e1,e2);

        % fep = My_ISS(P, r, e1,e2);
        % feq = My_ISS(Q, r, e1,e2);
      case 7
        fep = My_ISS(P, r_P_k * i, e1,e2,idx,dis);
        feq = My_ISS(Q, r_Q_k * i, e1,e2,idx,dis);

        % fep = My_ISS(P, r, e1,e2,idx,dis);
        % feq = My_ISS(Q, r, e1,e2,idx,dis);
      otherwise
        error('no bandwidth specified')    
    end

  	feq0 = feq;

  	p0 = P(:,fep);
  	q0 = Q(:,feq);

    % num_p0_r(i) = size(p0,2);                %for Debug

    % figure;                                  %for Debug
    displayer.displayFinalPickKeyPoint(p0,q0); %for Debug

    % i                                        %for Debug
  % end                                        %for Debug

  % figure                                     %for Debug
  % plot(num_p0_r)                             %for Debug 
  % xlabel('i * den')                          %for Debug

end



function [n1,d1,r_k] = knn(P)
    [n1,d1] = knnsearch(transpose(P), transpose(P), 'k', 400);      %����ȡ���� �����400���� ��
                                                                    %n1Ϊ���صĵ�����������վ����������
                                                                    %d1Ϊ������õ�ľ��룬��������
                                                                    %���Ϊn*400��
    n1=transpose(n1);                       %�ֱ��n1��d1ȡת��Ϊ400*n��
    d1=transpose(d1);
    
    %paper repetition
    paper = paperISS;
    r_k = paper.paper(d1);
    border_point = paper.borderPoint(P);

end

