function  [Q1,R,T]= coarseRegistration(P,Q,fep,feq,nv)      %
%   []= registration(P,Q,fep,feq,nv) ：粗配准
%      
%  输入参数  
%    P       :模板点云  3 * n
%    Q       :场景点云  3 * n
%    fep     :模板点云特征点的索引      1 * n
%    feq     :场景点云特征点的索引      1 * n
%    nv      :PFH描述中的最近邻点索引   1 * n
%
%
%  输出参数
%    Q1       ：变换后的点云  3 * n
%    R        ：旋转矩阵      
%    T        ：平移矩阵      3 * 1
%   
%  Author：GJT 
%  E-mail：gjt0114@outlook.com


%% 四元素粗配准
%调用子函数Quater_Registration配准算法，输入为Q矩阵P矩阵匹配的初值，输出为旋转矩阵R和平移向量T
[R,T] = Quater_Registration(Q(:,feq)',P(:,fep(nv))');

%应用变换矩阵
Q1    = R * Q + repmat(T,1,size(Q,2));

%% 绘制匹配结果
displayer = displayFunction;
displayer.displayRigistration(P,Q1);

disp('正确匹配点对的数量为：');
disp(length(nv));

% clear axe a aa b b0 c c0 i j n num R T x y z dp dq

end

