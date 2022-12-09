function FPFH=fpfhdescriptor(P,pn,fep,r,idx_p,dis_p)

%      
%  �������  
%     P      : ����            3 * n 
%     pn     : Ŀ����Ʒ�����  3 * n   
%     fep    : Դ����������    1 * n     
%     r      : Ŀ���������뾶������������    ����
%     idx_p  : Դ��������������Ԫ������   1 * n       |    ȱʡ����KDtree ����r�����
%     dis_p  : Դ����������룬Ԫ������   1 * n       |
%
%
%  �������
%    FPFH    ��FPFH��������i�д����i���������Ӧ��FPFH����    33 * length(fep)
%   
%  Author��GJT 
%  E-mail��gjt0114@outlook.com

    if nargin < 4%nargin�жϱ�������
        error('no bandwidth specified')
    end
    if nargin < 5
        Mdl_p = createns(P','NSMethod','kdtree','Distance','minkowski','p',2);%
        [idx_p,dis_p]=rangesearch(Mdl_p,P',r);
    end

    FPFH = zeros(length(fep),33);%���������
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