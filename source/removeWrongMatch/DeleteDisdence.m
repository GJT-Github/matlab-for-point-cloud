function [feq,nv,e_Delet_Distance] = DeleteDisdence(P,Q,fep,feq,feq0,nv)
%  [p0,q0,feq,nv] = DeleteDisdence(P,Q,fep,feq,feq0,nv)  : �޳� �����Զ�Ķ�Ӧ��
%
%  �������  
%    P       ��ģ�����
%    Q       ����������
%    fep     ��ģ����������������
%    feq     ���������������������
%    feq0    �������������������������
%    nv      : PFH�����е�����ڵ�����
%
%  �������
%    p0      ��ģ����Ƶ������㼯�� �޳���
%    q0      ���������Ƶ������㼯�� �޳���
%    feq     ��������������������� �޳��� 
%    nv      : PFH�����е�����ڵ����� �޳���
%   
%  Author��GJT 
%  E-mail��gjt0114@outlook.com



    dist = zeros(size(nv));                 %����0���󣬴�С���ھ���nv
    for i = 1:length(nv)
        dist(1,i) = norm( Q( :,feq(i) ) - P( :,fep( nv(1,i) ) ) );%������Q(:,feq(i))��ʾQ�����еĵ�i���ؼ��㣬P(:,fep(nv(1,i))��ʾP�����о���Q�е�i�ؼ��������
    end

    e_Delet_Distance = 1.0*mean(dist);

    feq(dist>e_Delet_Distance) = [];                    %��feq��ɾ���������0.05������㣨�ؼ��㣩��ָ��
    nv(dist>e_Delet_Distance)  = [];                    %��nv��ɾ���������0.05�����������ӵ�ָ��

%     p0 = P(:,fep);
%     q0 = Q(:,feq0);

%     %���� ɾ���������0.05���ͼ
%     displayer = displayFunction;
%     displayer.displayDeleteDisdencePointCloudAndLine(P,Q,p0,q0,fep,feq,nv,e_Delet_Distance);

end