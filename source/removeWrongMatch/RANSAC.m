function [feq,nv] = RANSAC(P,Q,fep,feq,feq0,aa,nv,e_RANSAC_Distance)
% = () ��% �������һ�����㷨
%      
%  ������� 
%    P       ��ģ�����
%    Q       ����������
%    fep     ��ģ����������������
%    feq     ���������������������
%    feq0    ������������������������� 
%    aa      : ��������
%    nv      : PFH�����е�����ڵ�����
%    e_RANSAC_Distance ��%�������һ���Ծ�����ֵ
% 
%  �������
%    p0      ��ģ����Ƶ������㼯�� �޳��� 
%    q0      ���������Ƶ������㼯�� �޳���
%    feq     ��������������������� �޳���
%    nv      : PFH�����е�����ڵ����� �޳���
%   
%  Author��GJT 
        %  E-mail��gjt0114@outlook.com


    n = length(nv);
    b0 = 0;                                     %��¼��Ӧ��ŷʽ����С����ֵ�� ����Ӧ����
    c0 = zeros(size(feq));
    while aa

        %�����ѡ��
        a    = randperm(n);                     % ���������
        feq1 = feq(a(1:3));                     % 3������������   
        nv1  = nv(a(1:3));                      % 3��
        
        %ƽ����ת�������
        [R,T] = Quater_Registration( Q(:,feq1)' , P(:,fep(nv1))' );

        %��ƽ����ת����Ӧ�õ�ģ�����
        Q0 = R * Q(:,feq) + repmat(T,1,n);
        
        %ͳ�������õ��ı任������ С�ھ�����ֵ����ȷƥ���Ӧ����
        difference = Q0 - P(:,fep(nv));
      
        b = 0;                                    %��¼��Ӧ��ŷʽ����С����ֵ�� �ܵ���
        c = zeros(1,n);                           %��¼��Ӧ��ŷʽ����С����ֵ�� ����

        for i = 1:n
            % if norm(difference(:,i))<0.0012
            if norm(difference(:,i)) < e_RANSAC_Distance    %0.005 for bun0*.asc
                b = b + 1;
                c(i) = 1;
            end
        end

        if b > b0
            b0 = b;                               %��¼ ����Ӧ����
            c0 = c;                               %��¼����Ӧ������Ӧ�����������
        end

        aa = aa - 1;

    end

    %�������أ�����ɸѡ����ȷƥ��
    feq(c0<1) = [];
    nv(c0<1)  = [];

%     p0 = P(:,fep);
%     q0 = Q(:,feq0);



end