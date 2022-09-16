%% 计算特征点
Mdl_p = createns(P','NSMethod','kdtree','Distance','minkowski','p',2);%建立KD树
[idx_rn_p,dis_p]=rangesearch(Mdl_p,P',0.005);
idx_fe_p = My_ISS(P,0.005,0.8,0.4,idx_rn_p,dis_p);
p1=P(:,idx_fe_p);
pn1=pn(:,idx_fe_p);
Mdl_q = createns(Q','NSMethod','kdtree','Distance','minkowski','p',2);
[idx_rn_q,dis_q]=rangesearch(Mdl_q,Q',0.005);
idx_fe_q = My_ISS(Q,0.005,0.8,0.4,idx_rn_q,dis_q);
q1=Q(:,idx_fe_q);
qn1=qn(:,idx_fe_q);