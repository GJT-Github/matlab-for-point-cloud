function [outpointcloud] = my_Harris3D(pointcloud)
%HARRIS3D_MY 此处显示有关此函数的摘要
%   此处显示详细说明
pointdata = pointcloud.Location; % n*3

[~,d1] = knnsearch(pointdata,pointdata,'k',400);
paper = paperISS;
mr  = paper.paper(d1) ;


%------------------ 法向量求解 --------------------------------
% mr = pointcloud_mr(pointcloud);
kp=6;
normal = pcnormals(pointcloud,kp);

%------------------协方差矩阵 PCA求法向量--------------------
% [indx,~] = rangesearch(pointdata,pointdata,mr*kp);
% normal = zeros(3,length(pointdata));
% for i=1:length(pointdata)
%     nei_point =pointdata(indx{i}(2:end),:);
%     cur_point =pointdata(i,:);
%     mean_point =mean(nei_point);
%     n = length(indx{i}(2:end));
%     Nei = nei_point - repmat(mean_point,n,1);
%     C = (Nei)' *  (Nei) /n;
%     [V,~] = eig(C);
%     nor = V(:,1);
%     if size(nei_point,1)<1
%         continue
%     elseif size(nei_point,1)<2
%        flag = dot(nor,nei_point - cur_point);
%     else
%             flag = dot(nor,sum(repmat(cur_point,n,1) -nei_point));
%     end
%     if flag <0
%         nor = -nor;
%     end
%     normal(:,i) = nor;
% end
% normal = normal';
%-----------------特征度计算------------------
[indx1,~] = rangesearch(pointdata,pointdata,4*mr);
R3d = zeros(length(pointdata),1);
for i=1:length(pointdata)
    nei_normal =normal(indx1{i}(2:end),:)';
    n = length(indx1{i}(2:end));
    M3d = (nei_normal * nei_normal')./n;
    % R3d(i) = det(M3d) - 0.04 *(trace(M3d)^2);
    R3d(i) =det(M3d)/(trace(M3d))^2;
end
%---------------特征点提取----------------------
index=find(R3d>-0.3);
fea_ind = [];
[indx2,~] = rangesearch(pointdata,pointdata,2*mr);
for j =1:length(index)
    nei_p = indx2{index(j)};
    nei_p_r3d = R3d(nei_p);
    [~,ind]=max(nei_p_r3d);
    if ind(1) == 1
        fea_ind = [fea_ind ;index(j)];
    end
end
feature_point =pointdata(fea_ind,:);
outpointcloud = pointCloud(feature_point);


%----------------展示----------------------------
% pcshow(intputcloud.Location,'r');
% hold on; 
% pcshow(harris3D_point.Location,'b',"MarkerSize",100)


end

