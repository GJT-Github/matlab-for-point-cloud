%一种基于降采样后关键点优化的点云配准方法
function  paperFunction = paperISS 

paperFunction.paper =@paper;
paperFunction.borderPoint =@borderPoint;

end


function [r_k]=paper(d1)
       
    %     save knn.mat                          %for Debug
    % load knn.mat                                %for Debug
    % for i=1:39                                  %for Debug
        %20邻域
    %     k=10*i;                                 %for Debug  邻域越大平均半径越大
        k = 20;
        beita = 0.5;                            %取值[0,1]
        d_near = d1(2,:);                       %最近点
        d_aver = mean(d1(2:k+1,:),1);           %20邻域距离均值
        d_point_mean = mean(d_aver);            %整体点云k邻域均值
       
        %Devia = sqrt(mean((d1(2:k+1,:) - repmat(d_aver,k,1)).^2));     %标准差
        Devia = sqrt(mean((d1(2:k+1,:) - repmat(d_point_mean,k,size(d1,2))).^2));     %标准差
        
        b = ( d_aver <= (d_point_mean + beita * Devia) );
        den = (d_aver * b' + d_near * (~b)') / size(b,2);  %点云平均距离
    %     den_record(i) = den;                     %for Debug
    % end                                          %for Debug
    % plot(den_record)                             %for Debug
       r_k = 1 * den;
       
end


function [border_point] = borderPoint(P)
    
    %%边界点判断
    %构建KDtree
    NS = createns(P','NSMethod','kdtree');

    %半径检索
    r_border = 0.3;
    [idx_border,dis_border] = rangesearch(NS,P',r_border);

    %统计r_border半径下点数
    for i = 1:size(idx_border,1)
        num(i) = size(idx_border{i},2); 
    end

    %边界点
    e_num_border = 20;
    indx_border = find(num<e_num_border);
    border_point = P(:,indx_border);

end




