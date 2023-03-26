function MyWay = myway
%MYWAY 自己的方法
% 点云平均距离估算
MyWay.AverageDisdance =@AverageDisdance;


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%3.13%%%%%%%%%%%%%%%%%%%%%%%%%

function [r_k]=AverageDisdance(P)
%Date :3.16
%点云平均距离计算
%P   ：输入点云 
%r_k ：输出点云平均距离

    k = 20;                                 %领域值
    [~,d1]=knnsearch(transpose(P), transpose(P), 'k', k+1); 

    d1 = d1';
    d_near = d1(2,:);                       %最近点            a
    d_no_self = d1(2:k+1,:);

    mean_k_neighbor = mean(d_no_self,1);    %20邻域距离均值
    StandardDeviation_k_neighbor = sqrt(mean((d_no_self - repmat(mean_k_neighbor,k,1)).^2));     %标准差

    n=1;
    %取出[μ-n*σ,μ+n*σ]内的点
    index_low_to_max_range  = (d_no_self <=  repmat(mean_k_neighbor + n * StandardDeviation_k_neighbor,k,1));
    low_to_max_range        = index_low_to_max_range.*d_no_self;
    index_in_range          = (low_to_max_range >=  repmat(mean_k_neighbor - n * StandardDeviation_k_neighbor,k,1));
    in_range                = index_in_range.*d_no_self;

    sum_num_in_range        = sum(index_in_range,1); 
    distance_every_neighbor = sum(in_range,1)./sum_num_in_range;

    r_k=mean(distance_every_neighbor);
end