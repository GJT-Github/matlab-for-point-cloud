%% ���������㼯p��q�ĸ��Ա任������p��q�Ĵ�СҪһ��
function [R, t] = rigidTransform3D(p, q)
n = cast(size(p, 1), 'like', p);
m = cast(size(q, 1), 'like', q);
% ȥ���Ļ�
pmean = sum(p,1)/n;
p2 = bsxfun(@minus, p, pmean);
qmean = sum(q,1)/m;
q2 = bsxfun(@minus, q, qmean);
% ��Э����������SVD�ֽ�
C = p2'*q2;
[U,~,V] = svd(C);
R = V*diag([1 1 sign(det(U*V'))])*U';
t = qmean' - R*pmean';
end