function [vep,veq] = PFHCaculate(P,Q,p0,q0,fep,feq,pn,qn,n1,d1,n2,d2)      %
% [vep,veq] = PFHCaculate(p0,fep,q0,feq) ��
%      
%  �������  
%  p0    ��Ŀ�����
%  q0    : Դ����
%
%  feq   : Դ����������
%  fep   : Ŀ�����������
%  
%  n1   ��Ŀ����� 400�� �����
%  n2   ��Դ���� 400�� �����
%
%  d1   ��Ŀ����� 400������� ���� ��С��������
%  d2   ��Դ���� 400�������� ���� ��С��������
%
%  �������
%  vep   : Ŀ�����PFH��������
%  veq   : Դ����PFH��������
%  
%
%  Author��GJT


vep=zeros(64,length(fep));                     %����64�У�fep���о���
veq=zeros(64,length(feq));


% p0=P(:,fep);                                  %��P���ҳ������㼯
% q0=Q(:,feq);

pn0=pn(:,fep);                                %��pn���ҳ������㷨������
qn0=qn(:,feq);

n11=n1(:,fep);                                %��n1���ҳ����������򼯵�����
d11=d1(:,fep);                                %��d1���ҳ����������򼯵ľ���

id1=d11<0.003;                                %���ž��󣬸���r����뾶

n22=n2(:,feq);
d22=d2(:,feq);

id2=d22<0.003;

%% ���� Ŀ����� ��PFH��������

% tic                                        %%%��ʱ��ʼ
%����ÿ����������� r�����ڵ�������
for i=1:length(fep)
%    pp=p0(:,i);                              %����������
%    pn00=pn0(:,i);                           %�����㷨����
    pr0=n11(id1(:,i),i);                     %������r�����ڵ�� ��������
    pn1=pn(:,pr0);                           %������r�����ڵ�� ��������
    pr=P(:,pr0(1:end));                      %������r������     �㼯
    pfh=zeros(64,1);
    
    %����r�����������㲢 ȥ�������㱾��
    for j1=1:length(pr0)
        zuobiao0 = pr(:,j1);                   %��ȡ������ĳ�����㣨���ĵ㣩����
        fashi0   = pn1(:,j1);                  %��ȡĳ������ķ�ʸ
        
        zuobiao1 = pr;                         %��ȡ������r���������е�
        zuobiao1(:,j1) = [];                   %��r������ɾ������������
        
        fashi1   = pn1;                        %��ȡ������r���������е�ķ�ʸ
        fashi1(:,j1)   = [];                   %��r������ɾ�������㷨ʸ
        
        %%%����ȥ�������㱾����r�����������㣬�����ֲ����꣬ͳ������PFHֵ
        for j2 = 1:length(pr0) - 1             %ɾ�����������һ��
            zuobiao2 = zuobiao1(:,j2);         %һ��r���������
            fashi2   = fashi1(:,j2);           %һ��r�����ķ�ʸS
            
            u = fashi0;                        %����ѡ��������� ������ �����ֲ�����ϵ������u��
            v = cross(u,(zuobiao2 - zuobiao0)./norm(zuobiao2 - zuobiao0));     %norm(zuobiao2-zuobiao0)��2������cross(A,B)����A���B�����彨��v��
%             v = v./(power(sum(v.^2),1/2));
            w = cross(u,v);                    %����u���v������w��
            
            SY = [dot(v,fashi2);...            %����������������  dot(A,B)����ΪA���B ���� = �͡� n_t    (�ڻ�/������)
                                               %   �� Ϊ ���з�
                  dot(u , (zuobiao2 - zuobiao0) ./ norm(zuobiao2 - zuobiao0) );...   %���������������㷵�� �� = u �� ����p_t - p_s��/d��
                  cos( atan( dot(w,fashi2) ./ dot(u,fashi2) ) )];                    %���������������㷵��  cos�� = cos( arctan(w �� n_t , u �� n_t) )
                  %atan( dot(w,fashi2) ./ dot(u,fashi2) ) ];

            a = double( SY > -0.5 ) + double( SY > 0 ) + double( SY > 0.5 );    %%%�����㻮���ĸ����䣬ʵ��ÿ�������������ϸ��
            a = a(1) * 16 + a(2) * 4 + a(3) + 1;                                %%%�Ľ����У�000~333����ʾ: a(1)a(2)a(3) + 1 ,��1Ŀ���� pfh�����±��1��ʼ
                                                                                %%%  -> ʮ���Ʊ�ʾ:a(1) * 4^2 + a(2) * 4^1 + a(3)*4^0 + 1
            
            pfh(a) = pfh(a) + 1;
            
        end    
    end
    vep(:,i) = pfh;
end
% t=toc                                      %��ʱ����

%% ���� Դ���� ��PFH��������

for i=1:length(feq)

    pr0=n22(id2(:,i),i);                     %������r������������
    pn1=qn(:,pr0);                           %������r����������
    pr=Q(:,pr0);                             %������r����㼯
    pfh=zeros(64,1);
    for j1=1:length(pr0)
        zuobiao0=pr(:,j1);                   %��ѯ������
        fashi0=pn1(:,j1);                    %��ѯ�㷨ʸ
        zuobiao1=pr;
        zuobiao1(:,j1)=[];                   %��r������ɾ����ѯ��
        fashi1=pn1;
        fashi1(:,j1)=[];
        for j2=1:length(pr0)-1
            zuobiao2=zuobiao1(:,j2);         %һ��r���������
            fashi2=fashi1(:,j2);             %һ��r�����ķ�ʸ
            u=fashi0;                        %�����ֲ�����ϵ
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



%% ����Ŀ����Ƶĵ�144������������

figure;
axe(1)=subplot(231);
bar(vep(:,144));                      %bar��bar3�ֱ��������ƶ�ά����ά��ֱ��ͼ������vep�е�144��������
axis([0 64 0 1200])                   %axis([xmin xmax ymin ymax])���õ�ǰ������ x���y��ķ�Χ

title('��144���ؼ����PFH����������')


% for i=1:length(vep)
% %     subplot(1,length(vep)/10-0.4,i)
%     plot(1:64,vep(:,i))
% %     hold on
%     pause(0.02)
% end
%%% clear i j1 j2 a u v w zuobiao0 zuobiao1 zuobiao2 SY fashi0 fashi1 fashi2 pfh 
%% clear vars -except vep veq P Q fep feq feq0
% save PFH2.mat
