function FPFH=fpfhdescriptor(P,pn,fep,r,idx_p,dis_p)

%      
%  输入参数  
%     P      : 点云            3 * n 
%     pn     : 目标点云法向量  3 * n   
%     fep    : 源点云特征点    1 * n     
%     r      : 目标点云邻域半径，邻域搜索用    标量
%     idx_p  : 源点云邻域索引，元胞数组   1 * n       |    缺省则用KDtree 搜索r邻域点
%     dis_p  : 源点云邻域距离，元胞数组   1 * n       |
%
%
%  输出参数
%    FPFH    ：FPFH描述，第i行代表第i个特征点对应的FPFH描述    33 * length(fep)
%   
%  Author：GJT 
%  E-mail：gjt0114@outlook.com

    if nargin < 4%nargin判断变量个数
        error('no bandwidth specified')
    end
    if nargin < 5
        Mdl_p = createns(P','NSMethod','kdtree','Distance','minkowski','p',2);%
        [idx_p,dis_p]=rangesearch(Mdl_p,P',r);
    end

    FPFH = zeros(length(fep),33);%创建零矩阵
    loop = 0;
    for i=fep
        loop = loop + 1;
        idx_rneighbor = idx_p{i};
        dis_rneighbor = dis_p{i};
        SPFH = My_SPFH(P,pn,idx_rneighbor,dis_rneighbor);
        SPFH2 = zeros(1,33);
        for j = 2:length(idx_rneighbor)
            idx_rneighbor2 = idx_p{idx_rneighbor(j)};
            dis_rneighbor2 = dis_p{idx_rneighbor(j)};
            rnei_SPFH = My_SPFH(P,pn,idx_rneighbor2,dis_rneighbor2);
            weight = dis_rneighbor(j);
            SPFH2 = SPFH2+rnei_SPFH./weight;
        end
        FPFH(loop,:) = SPFH + SPFH2./(length(idx_rneighbor)-1);
    end
    FPFH = FPFH';
end


function SPFH=My_SPFH(P,nor_p,idx_rneighbor,dis_rneighbor)
    rneighbor = P(:,idx_rneighbor);
    nor_rneighbor = nor_p(:,idx_rneighbor);
    u = nor_rneighbor(:,1);
    SPFH = zeros(1,33);
    for j=2:length(idx_rneighbor)
        v = cross(u,(rneighbor(:,j)-rneighbor(:,1))./dis_rneighbor(j));
        w = cross(u,v);
        alpha = dot(v,nor_rneighbor(:,j));
        vote = uint8(ceil((alpha+1)*(11/2)));
        if vote<=0
            vote=1;
        elseif vote>11
            vote = 11;
        end
        SPFH(vote)=SPFH(vote)+1;
        phi = dot(u,(rneighbor(:,j)-rneighbor(:,1))./dis_rneighbor(j));
        vote = uint8(ceil((phi+1)*(11/2)));
        if vote<=0
            vote=1;
        elseif vote>11
            vote = 11;
        end
        SPFH(vote+11)=SPFH(vote+11)+1;
        theta = sin(atan(dot(w,nor_rneighbor(:,j))./dot(u,nor_rneighbor(:,j))));
        vote = uint8(ceil((theta+1)*(11/pi)));
        if vote<=0
            vote=1;
        elseif vote>11
            vote = 11;
        end
        vote = uint8(vote);
        SPFH(vote+22)=SPFH(vote+22)+1;
    end
end