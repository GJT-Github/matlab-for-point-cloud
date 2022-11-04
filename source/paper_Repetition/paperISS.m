%一种基于降采样后关键点优化的点云配准方法
function  paperFunction = paperISS 

paperFunction.paper =@paper;
paperFunction.borderPoint =@borderPoint;

end


function [r_k]=paper(d1)
    % d1   : 目标点云 400个邻域点 距离 从小到大排序 400 * n
       
    % save knn.mat                                                            %for Debug
    % load knn.mat                                                            %for Debug
    % for i=1:39                                                              %for Debug
        %20邻域
    %     k=10*i;                                                             %for Debug  邻域越大平均半径越大
        k = 20;                                 %领域值
        beita = 0.5;                            %取值[0,1]

        d_near = d1(2,:);                       %最近点            a
        
        d_aver = mean(d1(2:k+1,:),1);           %20邻域距离均值
        d_point_mean = mean(d_aver);            %整体k邻域距离均值 b
       
        %Devia = sqrt(mean((d1(2:k+1,:) - repmat(d_aver,k,1)).^2));                   %标准差
        Devia = sqrt(mean((d1(2:k+1,:) - repmat(d_point_mean,k,size(d1,2))).^2));     %标准差
        
        b = ( d_aver <= (d_point_mean + beita * Devia) );
        den = (d_aver * b' + d_near * (~b)') / size(b,2);   %点云平均距离
    %     den_record(i) = den;                                                %for Debug
    % end                                                                     %for Debug
    % plot(den_record)                                                        %for Debug
       r_k = 1 * den;
       
end


function [border_point] = borderPoint(P,p0)
    % P             : 输入 3 * n 点云
    % border_point  ：输出 3 * m 边界内或边界外点

    [~,~,~,longline] = box(P);                       %输入点云最小包围盒对角线长度
    % for n=1:50                                                                %for Debug 边界点阈值
        n=10;                                        %邻域球内点数阈值
        %%边界点判断
        %构建KDtree
        NS = createns(P','NSMethod','kdtree');
        %半径检索

        r_border = longline/100;                     % rabit 0.3   
        [idx_border,dis_border] = rangesearch(NS,p0',r_border);
        %统计r_border半径下点数
        for i=1:size(idx_border,1)
            num(i) = size(idx_border{i},2); 
        end
        %边界点
        e_num_border = n;                             % 边界点阈值
        indx_border  = find(num>e_num_border);        % <为边界，>为非边界 ，边界点列索引
        border_point = p0(:,indx_border);              % 边界点云
        
        %展示边界点
        % figure;                                                              
        % plot3(P(1,:),P(2,:),P(3,:),'.');                                     
        % hold on;                                                             
        % plot3(border_point(1,:),border_point(2,:),border_point(3,:),'r.');   
        % num_point(n)=size(border_point,2);                                     %for Debug  邻域球内点数阈值对边界点的影响
    % end                                                                        %for Debug

    % figure;                                                                    %for Debug
    % plot(num_point);                                                           %for Debug
end




