function [feq,nv] = RANSAC(P,Q,fep,feq,feq0,aa,nv,e_RANSAC_Distance)
% = () ：% 随机采样一致性算法
%      
%  输入参数 
%    P       ：模板点云
%    Q       ：场景点云
%    fep     ：模板点云特征点的索引
%    feq     ：场景点云特征点的索引
%    feq0    ：场景点云特征点的索引备份 
%    aa      : 抽样次数
%    nv      : PFH描述中的最近邻点索引
%    e_RANSAC_Distance ：%随机采样一致性距离阈值
% 
%  输出参数
%    p0      ：模板点云的特征点集合 剔除后 
%    q0      ：场景点云的特征点集合 剔除后
%    feq     ：场景点云特征点的索引 剔除后
%    nv      : PFH描述中的最近邻点索引 剔除后
%   
%  Author：GJT 
        %  E-mail：gjt0114@outlook.com


    n = length(nv);
    b0 = 0;                                     %记录对应点欧式距离小于阈值的 最多对应点数
    c0 = zeros(size(feq));
    while aa

        %随机点选择
        a    = randperm(n);                     % 随机数生成
        feq1 = feq(a(1:3));                     % 3个随机点的索引   
        nv1  = nv(a(1:3));                      % 3个
        
        %平移旋转矩阵计算
        [R,T] = Quater_Registration( Q(:,feq1)' , P(:,fep(nv1))' );

        %将平移旋转矩阵应用到模板点云
        Q0 = R * Q(:,feq) + repmat(T,1,n);
        
        %统计随机点得到的变换矩阵下 小于距离阈值的正确匹配对应点数
        difference = Q0 - P(:,fep(nv));
      
        b = 0;                                    %记录对应点欧式距离小于阈值的 总点数
        c = zeros(1,n);                           %记录对应点欧式距离小于阈值的 索引

        for i = 1:n
            % if norm(difference(:,i))<0.0012
            if norm(difference(:,i)) < e_RANSAC_Distance    %0.005 for bun0*.asc
                b = b + 1;
                c(i) = 1;
            end
        end

        if b > b0
            b0 = b;                               %记录 最多对应点数
            c0 = c;                               %记录最多对应点数对应的邻域点索引
        end

        aa = aa - 1;

    end

    %参数返回，最终筛选的正确匹配
    feq(c0<1) = [];
    nv(c0<1)  = [];

%     p0 = P(:,fep);
%     q0 = Q(:,feq0);



end