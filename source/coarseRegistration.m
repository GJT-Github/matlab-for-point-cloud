function  []= coarseRegistration(P,Q,fep,feq,nv)      %
%   []= registration(P,Q,fep,feq,nv) ������׼
%      
%  �������  
%    P       :ģ�����
%    Q       :��������
%    fep     :ģ����������������
%    feq     :�������������������
%    nv      :PFH�����е�����ڵ�����
%
%
%  �������
%    
%   
%  Author��GJT 
%  E-mail��gjt0114@outlook.com


%% ��Ԫ�ش���׼
%�����Ӻ���Quater_Registration��׼�㷨������ΪQ����P����ƥ��ĳ�ֵ�����Ϊ��ת����R��ƽ������T
[R,T] = Quater_Registration(Q(:,feq)',P(:,fep(nv))');

%Ӧ�ñ任����
Q1    = R * Q + repmat(T,1,size(Q,2));

%����ƥ����
displayer = displayFunction;
displayer.displayRigistration(P,Q1);

disp('��ȷƥ���Ե�����Ϊ��');
disp(length(nv));

clear axe a aa b b0 c c0 i j n num R T x y z dp dq

end

