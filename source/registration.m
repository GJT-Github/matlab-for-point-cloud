function  []= registration(P,Q,fep,feq,nv)      %
% = () ��
%      
%  �������  
%      ��
%
%
%  �������
%      ��
%   



%% ��Ԫ������׼
[R,T] = Quater_Registration(Q(:,feq)',P(:,fep(nv))');%�����Ӻ���Quater_Registration��׼�㷨������ΪQ����P����ƥ��ĳ�ֵ�����Ϊ��ת����R��ƽ������T
Q1    = R * Q + repmat(T,1,size(Q,2));



%����ƥ����
%figure;
axe(6)=subplot(236);
plot3(P(1,:),P(2,:),P(3,:),'r.');

hold on
plot3(Q1(1,:),Q1(2,:),Q1(3,:),'b.');
% hold on
% plot3(Q(1,:),Q(2,:),Q(3,:),'g.');
disp('��ȷƥ���Ե�����Ϊ��')
disp(length(nv));
xlabel('x');ylabel('y');zlabel('z');
title('��Ԫ����׼');
view(3)
linkaxes(axe(2:6),'xy')
clear axe a aa b b0 c c0 i j n num R T x y z dp dq

