%一种基于降采样后关键点优化的点云配准方法
function  paperFunction = paperISS 

paperFunction.paper =@paper;
paperFunction.borderPoint =@borderPoint;
% paperFunction.myway =@myway;

end


function [r_k]=paper(P)
    % d1   : 目标点云 400个邻域点 距离 从小到大排序 400 * n
       
    % save knn.mat                                                            %for Debug
    % load knn.mat                                                            %for Debug
    % for i=1:39                                                              %for Debug
        %20邻域
    %     k=10*i;                                                             %for Debug  邻域越大平均半径越大
        k = 20;                                 %领域值
        beita = 0.5;                            %取值[0,1]

        [~,d1]=knnsearch(transpose(P), transpose(P), 'k', k); 

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


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%3.13%%%%%%%%%%%%%%%%%%%%%%%%%

% function [r_k]=myway(P)
% %Date :

%     k = 20;                                 %领域值
%     [~,d1]=knnsearch(transpose(P), transpose(P), 'k', k+1); 

%     d1 = d1';
%     d_near = d1(2,:);                       %最近点            a
%     d_no_self = d1(2:k+1,:);

%     mean_k_neighbor = mean(d_no_self,1);    %20邻域距离均值
%     StandardDeviation_k_neighbor = sqrt(mean((d_no_self - repmat(mean_k_neighbor,k,1)).^2));     %标准差

%     n=1;
%     %取出[μ-n*σ,μ+n*σ]内的点
%     index_low_to_max_range  = (d_no_self <=  repmat(mean_k_neighbor + n * StandardDeviation_k_neighbor,k,1));
%     low_to_max_range        = index_low_to_max_range.*d_no_self;
%     index_in_range          = (low_to_max_range >=  repmat(mean_k_neighbor - n * StandardDeviation_k_neighbor,k,1));
%     in_range                = index_in_range.*d_no_self;

%     sum_num_in_range        = sum(index_in_range,1); 
%     distance_every_neighbor = sum(in_range,1)./sum_num_in_range;

%     r_k=mean(distance_every_neighbor);
% end



function [border_point,indx_border,No_indx_border] = borderPoint(P,p0,varargin)
    % P             : 输入 3 * n 点云
    % border_point  ：输出 3 * m 边界内或边界外点

    % 默认参数设置 e_num 默认为10
     %https://www.cnblogs.com/gshang/p/14532104.html
    p = inputParser;                                  % 函数的输入解析器
    addParameter(p,'e_num',10);                       % 设置变量名和默认参数
    parse(p,varargin{:});                             % 对输入变量进行解析，如果检测到前面的变量被赋值，则更新变量取值

    %输入点云最小包围盒对角线长度
    [~,~,~,longline] = box(P);     
                      
    % for n=1:50                                                                %for Debug 边界点阈值
        % n=10;                                        %邻域球内点数阈值

        %%边界点判断
        %构建KDtree
        NS = createns(P','NSMethod','kdtree');
        %半径检索
        r_border = longline/60;                       % rabit 0.3   100   
        [idx_border,dis_border] = rangesearch(NS,p0',r_border);
        %统计r_border半径下点数
        for i=1:size(idx_border,1)
            num(i) = size(idx_border{i},2); 
        end
        %边界点
        e_num_border = p.Results.e_num;                             % 边界点阈值
        No_indx_border  = find(num>e_num_border);        % <为边界，>为非边界 ，边界点列索引
        indx_border  = find(num<e_num_border);
        border_point = p0(:,No_indx_border);             % 边界点云
        
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




