function RMSE2(P,Q1)
%计算误差
kd_tree = KDTreeSearcher(P','BucketSize',10);%建立kd树，利用Kd-tree找出对应点集
[index, dist] = knnsearch(kd_tree, Q1');%利用kd树查询最近点，及其距离
disp(['平均误差err=',num2str(mean(dist))]);
%计算均方误差
RMSE = sqrt(dist.'*dist/length(Q1));
disp(['均方根误差err=',num2str(RMSE)]);