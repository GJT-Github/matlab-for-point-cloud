function  []= registration(P,Q,fep,feq,nv)      %
% = () ：
%      
%  输入参数  
%      ：
%
%
%  输出参数
%      ：
%   



%% 四元数法配准
[R,T] = Quater_Registration(Q(:,feq)',P(:,fep(nv))');%调用子函数Quater_Registration配准算法，输入为Q矩阵P矩阵匹配的初值，输出为旋转矩阵R和平移向量T
Q1    = R * Q + repmat(T,1,size(Q,2));



%绘制匹配结果
%figure;
axe(6)=subplot(236);
plot3(P(1,:),P(2,:),P(3,:),'r.');

hold on
plot3(Q1(1,:),Q1(2,:),Q1(3,:),'b.');
% hold on
% plot3(Q(1,:),Q(2,:),Q(3,:),'g.');
disp('正确匹配点对的数量为：')
disp(length(nv));
xlabel('x');ylabel('y');zlabel('z');
title('四元数配准');
view(3)
linkaxes(axe(2:6),'xy')
clear axe a aa b b0 c c0 i j n num R T x y z dp dq

