function [P_FPFH,Q_FPFH] = FPFHCaculate(P,Q,pn,qn,fep,feq,r_P,r_Q,idx_P,dis_P,idx_Q,dis_Q)      %
% [P_FPFH,Q_FPFH] = FPFHCaculate(P,Q,pn,qn,fep,feq,r_P,r_Q,idx_P,dis_P,idx_Q,dis_Q) �� ���������Ƽ��ϵ�FPFH��������
%      
%  �������  
%     P      : Ŀ�����  3 * n
%     Q      : Դ����    3 * n
%     pn     : Ŀ����Ʒ�����  3 * n
%     qn     : Դ���Ʒ�����    3 * n
%     fep    : Դ����������   1 * n
%     feq    : Ŀ����������� 1 * n
%     r_P    : Ŀ���������뾶������������      
%     r_Q    : Դ��������뾶������������
%     idx_P  : Ŀ�������������    1 * n    |    
%     dis_P  : Ŀ������������    1 * n    |
%     idx_Q  : Դ������������      1 * n    |    ȱʡ����KDtree ����r�����
%     dis_Q  : Դ�����������      1 * n    |
%
%
%  �������
%      ��
%   
%  Author��GJT 
%  E-mail��gjt0114@outlook.com

	% P_FPFH = fpfhdescriptor(P,pn,fep,r_P,idx_P,dis_P);
	% Q_FPFH = fpfhdescriptor(Q,qn,feq,r_Q,idx_Q,dis_Q);



  %   if nargin < 8 && nargin > 12     %nargin�жϱ�������
  %       error('no bandwidth specified')
  %   end

  %   if nargin < 9
		% P_FPFH = fpfhdescriptor(P,pn,fep,r_P);
		% Q_FPFH = fpfhdescriptor(Q,qn,feq,r_Q);
  %   end
  %   if nargin == 12
		% P_FPFH = fpfhdescriptor(P,pn,fep,r_P,idx_P,dis_P);
		% Q_FPFH = fpfhdescriptor(Q,qn,feq,r_Q,idx_Q,dis_Q);
  %   end

    if nargin == 8
    	P_FPFH = fpfhdescriptor(P,pn,fep,r_P);
  		Q_FPFH = fpfhdescriptor(Q,qn,feq,r_Q);
    elseif nargin == 12
    	P_FPFH = fpfhdescriptor(P,pn,fep,r_P,idx_P,dis_P);
  		Q_FPFH = fpfhdescriptor(Q,qn,feq,r_Q,idx_Q,dis_Q);
    else
    	error('no bandwidth specified')
    end


	










