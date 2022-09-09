function [vep,veq] = PFHCaculate(P,Q,p0,q0,fep,feq,pn,qn,n1,d1,n2,d2)      %
% [vep,veq] = PFHCaculate(p0,fep,q0,feq) ：
%      
%  输入参数  
%  p0    ：目标点云
%  q0    : 源点云
%
%  feq   : 源点云特征点
%  fep   : 目标点云特征点
%  
%  n1   ：目标点云 400个 邻域点
%  n2   ：源点云 400个 邻域点
%
%  d1   ：目标点云 400个邻域点 距离 从小到大排序
%  d2   ：源点云 400个邻域点点 距离 从小到大排序
%
%  输出参数
%  vep   : 目标点云PFH描述描述
%  veq   : 源点云PFH特征描述
%  
%
%  Author：GJT


vep=zeros(64,length(fep));                     %定于64行，fep长列矩阵
veq=zeros(64,length(feq));


% p0=P(:,fep);                                  %从P中找出特征点集
% q0=Q(:,feq);

pn0=pn(:,fep);                                %从pn中找出特征点法向量集
qn0=qn(:,feq);

n11=n1(:,fep);                                %从n1中找出特征点邻域集的索引
d11=d1(:,fep);                                %从d1中找出特征点邻域集的距离

id1=d11<0.003;                                %符号矩阵，给定r邻域半径

n22=n2(:,feq);
d22=d2(:,feq);

id2=d22<0.003;

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
end
% t=toc                                      %计时结束

%% 计算 源点云 的PFH特征描述

for i=1:length(feq)

    pr0=n22(id2(:,i),i);                     %特征点r邻域索引向量
    pn1=qn(:,pr0);                           %特征点r邻域法向量集
    pr=Q(:,pr0);                             %特征点r邻域点集
    pfh=zeros(64,1);
    for j1=1:length(pr0)
        zuobiao0=pr(:,j1);                   %查询点坐标
        fashi0=pn1(:,j1);                    %查询点法矢
        zuobiao1=pr;
        zuobiao1(:,j1)=[];                   %在r邻域中删除查询点
        fashi1=pn1;
        fashi1(:,j1)=[];
        for j2=1:length(pr0)-1
            zuobiao2=zuobiao1(:,j2);         %一个r邻域点坐标
            fashi2=fashi1(:,j2);             %一个r邻域点的法矢
            u=fashi0;                        %建立局部坐标系
            v=cross(u,(zuobiao2-zuobiao0)./norm(zuobiao2-zuobiao0));
            w=cross(u,v);
            SY=[dot(v,fashi2);...
                dot(u,(zuobiao2-zuobiao0)./norm(zuobiao2-zuobiao0));... 
                cos(atan(dot(w,fashi2)./dot(u,fashi2)))];
            a = double(SY>0)+double(SY>0.5)+double(SY>-0.5);
            a=a(1)*16+a(2)*4+a(3)+1;
            pfh(a)=pfh(a)+1;
        end
    end
    veq(:,i)=pfh;
end



%% 绘制目标点云的第144个特征描述子

figure;
axe(1)=subplot(231);
bar(vep(:,144));                      %bar和bar3分别用来绘制二维和三维竖直方图，绘制vep中第144列描述子
axis([0 64 0 1200])                   %axis([xmin xmax ymin ymax])设置当前坐标轴 x轴和y轴的范围

title('第144个关键点的PFH特征描述子')


% for i=1:length(vep)
% %     subplot(1,length(vep)/10-0.4,i)
%     plot(1:64,vep(:,i))
% %     hold on
%     pause(0.02)
% end
%%% clear i j1 j2 a u v w zuobiao0 zuobiao1 zuobiao2 SY fashi0 fashi1 fashi2 pfh 
%% clear vars -except vep veq P Q fep feq feq0
% save PFH2.mat
