%һ�ֻ��ڽ�������ؼ����Ż��ĵ�����׼����
function  paperFunction = paperISS 

paperFunction.paper =@paper;
paperFunction.borderPoint =@borderPoint;

end


function [r_k]=paper(d1)
    % d1   : Ŀ����� 400������� ���� ��С�������� 400 * n
       
    % save knn.mat                                                            %for Debug
    % load knn.mat                                                            %for Debug
    % for i=1:39                                                              %for Debug
        %20����
    %     k=10*i;                                                             %for Debug  ����Խ��ƽ���뾶Խ��
        k = 20;                                 %����ֵ
        beita = 0.5;                            %ȡֵ[0,1]

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


function [border_point] = borderPoint(P,p0)
    % P             : ���� 3 * n ����
    % border_point  ����� 3 * m �߽��ڻ�߽����

    [~,~,~,longline] = box(P);                       %���������С��Χ�жԽ��߳���
    % for n=1:50                                                                %for Debug �߽����ֵ
        n=10;                                        %�������ڵ�����ֵ
        %%�߽���ж�
        %����KDtree
        NS = createns(P','NSMethod','kdtree');
        %�뾶����

        r_border = longline/100;                     % rabit 0.3   
        [idx_border,dis_border] = rangesearch(NS,p0',r_border);
        %ͳ��r_border�뾶�µ���
        for i=1:size(idx_border,1)
            num(i) = size(idx_border{i},2); 
        end
        %�߽��
        e_num_border = n;                             % �߽����ֵ
        indx_border  = find(num>e_num_border);        % <Ϊ�߽磬>Ϊ�Ǳ߽� ���߽��������
        border_point = p0(:,indx_border);              % �߽����
        
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




