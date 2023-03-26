%һ�ֻ��ڽ�������ؼ����Ż��ĵ�����׼����
function  paperFunction = paperISS 

paperFunction.paper =@paper;
paperFunction.borderPoint =@borderPoint;
% paperFunction.myway =@myway;

end


function [r_k]=paper(P)
    % d1   : Ŀ����� 400������� ���� ��С�������� 400 * n
       
    % save knn.mat                                                            %for Debug
    % load knn.mat                                                            %for Debug
    % for i=1:39                                                              %for Debug
        %20����
    %     k=10*i;                                                             %for Debug  ����Խ��ƽ���뾶Խ��
        k = 20;                                 %����ֵ
        beita = 0.5;                            %ȡֵ[0,1]

        [~,d1]=knnsearch(transpose(P), transpose(P), 'k', k); 

        d_near = d1(2,:);                       %�����            a
        
        d_aver = mean(d1(2:k+1,:),1);           %20��������ֵ
        d_point_mean = mean(d_aver);            %����k��������ֵ b
       
        %Devia = sqrt(mean((d1(2:k+1,:) - repmat(d_aver,k,1)).^2));                   %��׼��
        Devia = sqrt(mean((d1(2:k+1,:) - repmat(d_point_mean,k,size(d1,2))).^2));     %��׼��
        
        b = ( d_aver <= (d_point_mean + beita * Devia) );
        den = (d_aver * b' + d_near * (~b)') / size(b,2);   %����ƽ������
    %     den_record(i) = den;                                                %for Debug
    % end                                                                     %for Debug
    % plot(den_record)                                                        %for Debug
       r_k = 1 * den;
       
end


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%3.13%%%%%%%%%%%%%%%%%%%%%%%%%

% function [r_k]=myway(P)
% %Date :

%     k = 20;                                 %����ֵ
%     [~,d1]=knnsearch(transpose(P), transpose(P), 'k', k+1); 

%     d1 = d1';
%     d_near = d1(2,:);                       %�����            a
%     d_no_self = d1(2:k+1,:);

%     mean_k_neighbor = mean(d_no_self,1);    %20��������ֵ
%     StandardDeviation_k_neighbor = sqrt(mean((d_no_self - repmat(mean_k_neighbor,k,1)).^2));     %��׼��

%     n=1;
%     %ȡ��[��-n*��,��+n*��]�ڵĵ�
%     index_low_to_max_range  = (d_no_self <=  repmat(mean_k_neighbor + n * StandardDeviation_k_neighbor,k,1));
%     low_to_max_range        = index_low_to_max_range.*d_no_self;
%     index_in_range          = (low_to_max_range >=  repmat(mean_k_neighbor - n * StandardDeviation_k_neighbor,k,1));
%     in_range                = index_in_range.*d_no_self;

%     sum_num_in_range        = sum(index_in_range,1); 
%     distance_every_neighbor = sum(in_range,1)./sum_num_in_range;

%     r_k=mean(distance_every_neighbor);
% end



function [border_point,indx_border,No_indx_border] = borderPoint(P,p0,varargin)
    % P             : ���� 3 * n ����
    % border_point  ����� 3 * m �߽��ڻ�߽����

    % Ĭ�ϲ������� e_num Ĭ��Ϊ10
     %https://www.cnblogs.com/gshang/p/14532104.html
    p = inputParser;                                  % ���������������
    addParameter(p,'e_num',10);                       % ���ñ�������Ĭ�ϲ���
    parse(p,varargin{:});                             % ������������н����������⵽ǰ��ı�������ֵ������±���ȡֵ

    %���������С��Χ�жԽ��߳���
    [~,~,~,longline] = box(P);     
                      
    % for n=1:50                                                                %for Debug �߽����ֵ
        % n=10;                                        %�������ڵ�����ֵ

        %%�߽���ж�
        %����KDtree
        NS = createns(P','NSMethod','kdtree');
        %�뾶����
        r_border = longline/60;                       % rabit 0.3   100   
        [idx_border,dis_border] = rangesearch(NS,p0',r_border);
        %ͳ��r_border�뾶�µ���
        for i=1:size(idx_border,1)
            num(i) = size(idx_border{i},2); 
        end
        %�߽��
        e_num_border = p.Results.e_num;                             % �߽����ֵ
        No_indx_border  = find(num>e_num_border);        % <Ϊ�߽磬>Ϊ�Ǳ߽� ���߽��������
        indx_border  = find(num<e_num_border);
        border_point = p0(:,No_indx_border);             % �߽����
        
        %չʾ�߽��
        % figure;                                                              
        % plot3(P(1,:),P(2,:),P(3,:),'.');                                     
        % hold on;                                                             
        % plot3(border_point(1,:),border_point(2,:),border_point(3,:),'r.');   
        % num_point(n)=size(border_point,2);                                     %for Debug  �������ڵ�����ֵ�Ա߽���Ӱ��
    % end                                                                        %for Debug

    % figure;                                                                    %for Debug
    % plot(num_point);                                                           %for Debug
end




