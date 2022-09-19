function [vep] = pfhDescriptor(P,fep,pn,n1,d1)      %
% function [vep] = pfhDescriptor(P,p0,fep,pn,n1,d1)      %
% [vep,veq] = PFHCaculate(P,Q,p0,q0,fep,feq,pn,qn,n1,d1,n2,d2) ：PFH特征描述
%      
%  输入参数  
%    P     : 点云  3 * n
% 
%    fep   : 点云特征点 1 * n
%
%    pn    : 点云法向量  3 * n
%    
%    n1    ：点云 400个 邻域点 400 * n
%   
%    d1    ：点云 400个邻域点 距离 从小到大排序 400 * n
%
%  输出参数
%    vep   : 点云PFH描述描述,第i列表示第i个特征点的PFH描述  64 * length(fep)
%  
%
%  Author：GJT 
%  E-mail：gjt0114@outlook.com


    vep=zeros(64,length(fep));                     %定于64行，fep长列矩阵
    % pn0=pn(:,fep);                                %从pn中找出特征点法向量集
    n11=n1(:,fep);                                %从n1中找出特征点邻域集的索引
    d11=d1(:,fep);                                %从d1中找出特征点邻域集的距离
    id1=d11<0.003;                                %符号矩阵，给定r邻域半径
    %% 计算 目标点云 的PFH特征描述

    % tic                                        %%%计时开始
    %遍历每个特征点提出 r邻域内的特征点
    for i=1:length(fep)
    %    pp=p0(:,i);                              %特征点坐标
    %    pn00=pn0(:,i);                           %特征点法向量
        pr0=n11(id1(:,i),i);                     %特征点r邻域内点的 索引向量
        pn1=pn(:,pr0);                           %特征点r邻域内点的 法向量集
        pr=P(:,pr0(1:end));                      %特征点r邻域内     点集
        pfh=zeros(64,1);
        
        %遍历r邻域内特征点并 去除特征点本身
        for j1=1:length(pr0)
            zuobiao0 = pr(:,j1);                   %提取领域内某特征点（中心点）坐标
            fashi0   = pn1(:,j1);                  %提取某特征点的法矢
            
            zuobiao1 = pr;                         %提取特征点r领域内所有点
            zuobiao1(:,j1) = [];                   %在r邻域中删除特征点自身
            
            fashi1   = pn1;                        %提取特征点r领域内所有点的法矢
            fashi1(:,j1)   = [];                   %在r领域内删除特征点法矢
            
            %%%遍历去掉特征点本身后的r邻域内特征点，建立局部坐标，统计邻域PFH值
            for j2 = 1:length(pr0) - 1             %删除特征点后少一列
                zuobiao2 = zuobiao1(:,j2);         %一个r邻域点坐标
                fashi2   = fashi1(:,j2);           %一个r邻域点的法矢S
                
                u = fashi0;                        %根据选定特征点的 法向量 建立局部坐标系，建立u轴
                v = cross(u,(zuobiao2 - zuobiao0)./norm(zuobiao2 - zuobiao0));     %norm(zuobiao2-zuobiao0)求2范数，cross(A,B)返回A叉乘B，整体建立v轴
    %             v = v./(power(sum(v.^2),1/2));
                w = cross(u,v);                    %返回u叉乘v建立第w轴
                
                SY = [dot(v,fashi2);...            %特征描述参数计算  dot(A,B)返回为A点乘B 即α = ν· n_t    (内积/数量积)
                                                   %   … 为 续行符
                      dot(u , (zuobiao2 - zuobiao0) ./ norm(zuobiao2 - zuobiao0) );...   %特征描述参数计算返回 Φ = u · （（p_t - p_s）/d）
                      cos( atan( dot(w,fashi2) ./ dot(u,fashi2) ) )];                    %特征描述参数计算返回  cosθ = cos( arctan(w · n_t , u · n_t) )
                      %atan( dot(w,fashi2) ./ dot(u,fashi2) ) ];

                a = double( SY > -0.5 ) + double( SY > 0 ) + double( SY > 0.5 );    %%%三个点划分四个区间，实现每个特征区间的四细分
                a = a(1) * 16 + a(2) * 4 + a(3) + 1;                                %%%四进制中（000~333）表示: a(1)a(2)a(3) + 1 ,加1目的是 pfh索引下标从1开始
                                                                                    %%%  -> 十进制表示:a(1) * 4^2 + a(2) * 4^1 + a(3)*4^0 + 1
                
                pfh(a) = pfh(a) + 1;
                
            end    
        end
        vep(:,i) = pfh;
        % i
    end
    % toc                                      %计时结束

end
