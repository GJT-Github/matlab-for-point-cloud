function rmse = rmse(ptCloudTformed,pointcloud,op_dis,dis)
data1 = ptCloudTformed.Location;
data2 = pointcloud.Location;  
[~,d1] = knnsearch(data2, data1, 'k', 1);
if op_dis == 1
d1(find(d1 > dis)) = [];
end
rmse = sqrt(sum(d1.^2)/length(d1));
disp(['rmse = ',num2str(rmse),' mm']);