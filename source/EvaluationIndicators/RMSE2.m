function RMSE2(P,Q1)
%�������
kd_tree = KDTreeSearcher(P','BucketSize',10);%����kd��������Kd-tree�ҳ���Ӧ�㼯
[index, dist] = knnsearch(kd_tree, Q1');%����kd����ѯ����㣬�������
disp(['ƽ�����err=',num2str(mean(dist))]);
%����������
RMSE = sqrt(dist.'*dist/length(Q1));
disp(['���������err=',num2str(RMSE)]);