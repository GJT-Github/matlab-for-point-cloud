% M-����
function [ m_estition ] = F_M_estition( difference_Value , B_baoHeDu )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%F_M_estition��M-���Ƶĳͷ�����
%
%���������
%    difference_Value ����ֵ
%    B_baoHeDu        : ���ƺ������ͶȲ�����һ��ȡ2.5��paper��ָ���ģ�
%
%���������
%    m_estition       : һ��ά�ȵ�M-���Ƴͷ�����ֵ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2022.3.19 GJT       ����ʵ��
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
