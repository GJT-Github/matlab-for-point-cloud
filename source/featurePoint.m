function [p0,q0,fep,feq,feq0,n1,d1,n2,d2]= featurePoint(P,Q,pn,qn,k)      %特征点提取
% b = featurePoint(P,Q,pn,qn,k) ：提取特征点
%
%  输入参数                         
%     P ：输入目标点云     3 * n
%     Q ：输入源点云       3 * n
%     pn ：目标点云法向量  3 * n
%     qn ：源点云法向量    3 * n
%
%  输出参数
%     p0 ：目标点云特征点  3 * n 
%     q0 ：源点云特征点    3 * n
%    
%     fep  ：目标点云特征点 列 索引 1 * n
%     feq  : 源点云特征点   列 索引   1 * n
%        
%     n1   ：目标点云 400个 邻域点 400 * n
%     n2   ：源点云   400个 邻域点 400 * n
%       
%     d1   ：目标点云 400个邻域点 距离 从小到大排序 400 * n
%     d2   ：源点云   400个邻域点 距离 从小到大排序 400 * n
%
%  Author：GJT 
%  E-mail：gjt0114@outlook.com

% % 剔除边缘点
% P_old = P;
% Q_old = Q;

% [P,p_idx,~]=border(P,150);
% [Q,q_idx,~]=border(Q,200);

%%%%%%%%%%%%%%%%%%%%%%%%%%%3.24%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%出图用
% for i=1:4
%    [P_mvBor,p_idx,~]=border(P,i*50,1,i);
% end
for i=1:4
   [Q_mvBor,q_idx,~]=border(Q,100+i*50,2,i);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pn(:,p_idx)=[];
qn(:,q_idx)=[];

%%%%%%%%%%%%%%%%%%%%%%%3.17:save rs1_1%%%%%%%%%%%%%%%%%%%%%%%%%%%
% a=pointCloud(P');
% pcwrite(a,'rm_rs_1.pcd');
% a_b=pointCloud(P_old(:,p_idx)');
% pcwrite(a_b,'bor_rs_1.pcd');
% 
% b=pointCloud(Q');
% pcwrite(b,'rm_rs_2.pcd');
% b_b=pointCloud(Q_old(:,q_idx)');
% pcwrite(b_b,'bor_rs_2.pcd');
% 
% clean a,a_b,b,b_b,P_old,Q_old

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% [p0,fep,n1,d1]=keypointOfNormalDotMean(P,pn,k);
% [q0,feq,n2,d2]=keypointOfNormalDotMean(Q,qn,k);
% feq0 = feq;
% %绘制粗提取的特征点
% displayer = displayFunction;
% % paper = paperISS;
% % global r_P_k r_Q_k;
% % r_P_k = paper.paper(d1) * 5;
% % r_Q_k = paper.paper(d2) * 5;
  
% displayer.displayFirstPickKeyPoint(p0,q0);
% p0 = P(:,fep);
% q0 = Q(:,feq);
% % 绘制最终提取的特征点
% displayer.displayFinalPickKeyPoint(p0,q0);


% for n=0:50:200
n=70;
    [p0,fep,n1,d1]=keypointOfNormalDotMean(P,pn,k,n);
    [q0,feq,n2,d2]=keypointOfNormalDotMean(Q,qn,k,n);
    feq0 = feq;
    
    %绘制粗提取的特征点
    displayer = displayFunction;
    
    % paper = paperISS;
    % global r_P_k r_Q_k;
    % r_P_k = paper.paper(d1) * 5;
    % r_Q_k = paper.paper(d2) * 5;
      
    displayer.displayFirstPickKeyPoint(p0,q0);
    disp(['粗',num2str(n),' : ',num2str(size(p0,2))])
    
    
    p0 = P(:,fep);
    q0 = Q(:,feq);
    
    %绘制最终提取的特征点
    displayer.displayFinalPickKeyPoint(p0,q0);
    disp(['精',num2str(n),' : ',num2str(size(p0,2))])
% end
%---------------------------------------------------------------------------------------------

% %%
% %%计算特征度
% pt=zeros(1,size(P,2));                  %创建1行P阵列数列0阵  
% qt=zeros(1,size(Q,2));


% % 目标点云特征度量计算
% [n1,d1] = knnsearch(transpose(P), transpose(P), 'k', 400);      %依次取各点 最近的400个点 ；
%                                                                 %n1为返回的点的列数，按照距离递增排序；
%                                                                 %d1为各点与该点的距离，递增排序；
%                                                                 %结果为n*400阵
% n1=transpose(n1);                       %分别对n1，d1取转置为400*n阵
% d1=transpose(d1);


%   %遍历点云数据矩阵-P 计算每一点对应8邻域的 特征值――pt
% % % %for i=1:size(P,2)                                               % size(P,2)返回矩阵P的列数   
% % % %    pt(1,i)=1/k*sum(abs(transpose(pn(:,n1(2:k+1,i)))*pn(:,i))); %n1(2:k+1,i)取n1返回距离指标矩阵中第i列的2--k+1行，值为列数；8*1矩阵
% % %                                                                  %transpose(pn(:,n1(2:k+1,i)))为取第i个点最近的8个点的法向量并转置，为8*3；
% % %                                                                  %transpose(pn(:,n1(2:k+1,i)))*pn(:,i)为再乘i点法向量3*1,值为法向量乘积为8*1
% % %                                                                  %abs()为取绝对值函数；sum()为求和函数；
% % %                                                                  %pt(1,i)为 第i个点的法向量与其最近8个点的法向量 相乘、取绝对值、累加、取平均值后的结果，最终pt为1*40097          

% for i=1:size(P,2)         
%     temp_1 = n1(2:k+1,i);                                       %8邻域点列索引（不含查询点）    8*1
%     temp_2 = pn(:,temp_1);                                      %8邻域点法向量                 3*8
%     temp_3 = transpose(temp_2);                                 %                             8*3  
%     temp_4 = temp_3 * pn(:,i);                                  %查询点法向量与邻域法向量  内积 （值越大越平坦） 8*1 = 8*3 3*1
%     temp_5 = abs(temp_4);                                          
%     temp = sum(temp_5);                                         %                1*1
%     pt(1,i)=1/k*temp;                                           %得到第i个点的8邻域特征度量值（法向量内积绝对值和的均值） 1*size(P,2)
% end
% clear temp temp_1 temp_2 temp_3 temp_4 temp_5


% % 源点云特征度量计算

% [n2,d2] = knnsearch(transpose(Q), transpose(Q), 'k', 400);

% n2=transpose(n2);
% d2=transpose(d2);

% for i=1:size(Q,2)
%     qt(1,i)=1/k*sum(abs(transpose(qn(:,n2(2:k+1,i)))*qn(:,i))); %行向量
% end


% %%选取特征点
% ptt=find(pt<mean(pt));                  % mean(pt)求pt40097平均值，返回一个数；   ptt值pt中为小于平均值的列指标，行向量
% qtt=find(qt<mean(qt));

% p0=P(:,ptt);                            %提取P阵中 小于特征度量均值 的点
% q0=Q(:,qtt);


% %绘制粗提取的特征点
% displayer = displayFunction;

% paper = paperISS;
%  global r_P_k r_Q_k;
%   r_P_k = paper.paper(d1) * 5;
%   r_Q_k = paper.paper(d2) * 5;
  
% displayer.displayFirstPickKeyPoint(p0,q0);


% fep=[];                                %定义俩个空矩阵，0行0列空矩阵   特征点列索引
% feq=[];                    
%   %遍历ptt中每一个元素，取出 特征点的列索引--fep、feq  行向量
% % % %   
% % % %  %for i=ptt                           % 表示ptt中为小于平均值的列指标 
% % % %    %   [~,I]=min(qt(1,n2(1:k+1,i)));  % n1(1:k+1,i)提取n1中第i列的前9个最近点指标，
% % % %                                       % 再提取出pt中第i个点与其最近9个点的法向量相乘取绝对值累加取平均值
% % % %                                       % min(pt(1,n1(1:k+1,i)))再取其中最小值（第i个点与其最近8个点的法向量相乘取绝对值累加取平均值的最小值）；
% % % %                                       % [~,I]表示如果函数有多个输出，为忽略前面的值，将最小值所在 列指标 赋值给I
% % % %   


% for i=ptt                              % 表示ptt中为小于平均值的列指标
%     temp   = n1(1:k+1,i);              % ptt对应邻域点列索引（包含查询点）           n1第1行就是查询点本身       列向量
%     temp   = pt(1,temp);               % ptt对应邻域点的特征度量值                                行向量
%     [~,I]  = min(temp);                % I：特征度量值最小的列索引

%     if I==1                            % 筛选出 查询点 在其8邻域点中 特征度量值最小 的点作为 关键点
%         fep=[fep,i];                   % 将i的值填充fep矩阵，fep为1行多列
%     end
% end
% clear temp 


% for i=qtt 
%     [~,I]=min(qt(1,n2(1:k+1,i)));
%     if I==1
%         feq=[feq,i];
%     end
% end

% feq0=feq;                             % fep为P中特征点在点云P中的索引


% p0=P(:,fep);                          % p0 为特征点坐标矩阵
% q0=Q(:,feq);


% %绘制最终提取的特征点
% displayer.displayFinalPickKeyPoint(p0,q0);


% %%%clear axe i data1 data2 file1 file2 I k
% %%clearvars -except P Q pn qn n1 d1 d11 n2 d2 d22 fep feq feq0

%save PFH1.mat%保存工作区所有变量
end

%---------------------------------------------------------------------------------------
% function [P_cloud,indx,No_indx]=border(P,e_num)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%3.24%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [P_cloud,indx,No_indx]=border(P,e_num,i,j)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%-------------------------
% 剔除掉边界点 将paper中的方法引入

paper = paperISS;
[~,indx,No_indx] = paper.borderPoint(P,P,'e_num',e_num);  %15


%展示剔除前后边界的效果
figure
plot3(P(1,:),P(2,:),P(3,:),'b.')
hold on
plot3(P(1,No_indx),P(2,No_indx),P(3,No_indx),'r.')
hold off
P_cloud=P(:,No_indx);

%%%%%%%%%%%%%%%%%%%%%%%3.24%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%保存出图
P_No = pointCloud(P(:,No_indx)');
P_In = pointCloud(P(:,indx)');
pcwrite(P_No,"../Datas/rs" + num2str(i) + "_No_" + num2str(j) + ".pcd");
pcwrite(P_In,"../Datas/rs" + num2str(i) + "_In_" + num2str(j) + ".pcd");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%-------------------------
end


function [p0,fep,n1,d1]=keypointOfNormalDotMean(P,pn,k,n)
% 法向量内积均值计算特征点提取
% 输入数据：
%     P  : 输入点云数据           3 * n
%     pn : 输入点云数据的法向量   3 * n
%     k  : k邻域大小              标量


%% 计算特征度
    pt=zeros(1,size(P,2));                  %创建1行P阵列数列0阵  
    % 目标点云特征度量计算
    [n1,d1] = knnsearch(transpose(P), transpose(P), 'k', 400);      %依次取各点 最近的400个点 ；
    n1=transpose(n1);                       %分别对n1，d1取转置为400*n阵
    d1=transpose(d1);

    for i=1:size(P,2)
        pt(1,i)=1/k * sum( abs( transpose( pn(:,n1(2:k+1,i)) )*pn(:,i) )); %行向量
    end

%% 选取特征点
    %粗提取
%     ptt=find(pt<mean(pt));                  % ptt：pt中小于平均值的列指标，行向量


sigma_pt = sqrt(sum((pt-repmat(mean(pt),1,size(pt,2))).^2))/size(pt,2);

    ptt=find(pt<mean(pt)+n*sigma_pt);                  % ptt：pt中小于平均值的列指标，行向量
    p0=P(:,ptt);                            % 提取P阵中 小于特征度量均值 的点

    fep=[];                                 % 特征点列索引

    %精提取
    %遍历ptt中每一个元素，取出 特征点的列索引--fep 行向量
    for i=ptt 
        [~,I]=min(pt(1,n1(1:k+1,i)));
        if I==1
            fep=[fep,i];
        end
    end
    % p0=P(:,fep);                          % p0 为特征点坐标矩阵


end
