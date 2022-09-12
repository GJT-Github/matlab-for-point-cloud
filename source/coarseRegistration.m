function  []= coarseRegistration(P,Q,fep,feq,nv)      %
%   []= registration(P,Q,fep,feq,nv) ：粗配准
%      
%  输入参数  
%    P       :模板点云
%    Q       :场景点云
%    fep     :模板点云特征点的索引
%    feq     :场景点云特征点的索引
%    nv      :PFH描述中的最近邻点索引
%
%
%  输出参数
%    
%   
%  Author：GJT 
%  E-mail：gjt0114@outlook.com


%% 四元素粗配准
%调用子函数Quater_Registration配准算法，输入为Q矩阵P矩阵匹配的初值，输出为旋转矩阵R和平移向量T
[R,T] = Quater_Registration(Q(:,feq)',P(:,fep(nv))');

%应用变换矩阵
Q1    = R * Q + repmat(T,1,size(Q,2));

%绘制匹配结果
displayer = displayFunction;
displayer.displayRigistration(P,Q1);

disp('正确匹配点对的数量为：');
disp(length(nv));

clear axe a aa b b0 c c0 i j n num R T x y z dp dq

end

