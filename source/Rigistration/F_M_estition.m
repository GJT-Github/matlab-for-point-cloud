% M-估计
function [ m_estition ] = F_M_estition( difference_Value , B_baoHeDu )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%F_M_estition：M-估计的惩罚函数
%
%输入参数：
%    difference_Value ：差值
%    B_baoHeDu        : 控制函数饱和度参数，一般取2.5（paper中指明的）
%
%输出参数：
%    m_estition       : 一个维度的M-估计惩罚函数值
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2022.3.19 GJT       函数实现
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

e = difference_Value .^2;
n = size( e , 2 );
for i = 1 : n
    if  e(i) > B_baoHeDu
        m_estition(i) = B_baoHeDu^2 / 2 ; 
    else
    	m_estition(i) =   B_baoHeDu^2 / 2 * ( 1 - ( 1 - ( e(i) / B_baoHeDu ).^2 ).^3 ); 
    end
end
