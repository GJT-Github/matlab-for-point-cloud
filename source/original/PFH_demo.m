clc
clear
close all
%% 特征点的提取
file1='../Datas/bun045.asc';
file2='../Datas/bun000.asc';

%%读取asc文件
data1 = ascread(file1);                  %读取asc类型点云，读到的数据为cell类型的2*1形式，1*1为点总数，2*1为所有点坐标值
data2 = ascread(file2);

P = data1{2};                            %截取所有点云坐标数据
Q = data2{2};
% moban1=Q(1,:)>-0.1;
% moban2=Q(1,:)<-0.06;
% moban3=Q(2,:)>0.1;
% moban4=Q(2,:)<0.15;
% ROI=moban1&moban2&moban3&moban4;
% Q=Q(:,ROI);
% data2{1}=size(Q,2);

k=8;                                     %8邻域

%%通过8邻域PCA建立法向量
pn = lsqnormest(P, k);                   %调用求法向量子函数lsqnormest求P阵所有法向量
qn = lsqnormest(Q, k);

figure;                                  %画读到的点云图
axe(1)=subplot(221);
plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot绘图函数，分别取P中第1.2.3行所有点作为坐标轴，r表示颜色
hold on
plot3(Q(1,:),Q(2,:),Q(3,:),'b.');

x_mid = median(P(1,:));
y_mid = median(P(2,:));
z_mid = median(P(3,:));


plot3(x_mid,y_mid,z_mid,'b*');
title('模板点云与目标点云')
view(2)                                  %设置图像视点函数，可以改变要观察的角度（面）



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%
% b=5;
% 
% for i=1:100:length(pn)                    %根据最近邻点连接直线
%     
%     c = b * power((pn(1,i) - P(1,i))^2 + ( pn(2,i) - P(2,i) )^2 + (pn(3,i) - P(3,i))^2 , 1/2);
% %     c=10;
%     hold on
%     x=[P(1,i),P(1,i) + pn(1,i)];%直线的起点x坐标与终点x坐标，注意起点为场景中第一点，终点为对应其在模型中最近点
%     y=[P(2,i),P(2,i) + pn(2,i)];
%     z=[P(3,i),P(3,i) + pn(3,i)];
%     plot3(x,y,z,'g-');
%     plot3((P(1,i) + pn(1,i))/c,(P(2,i) + pn(2,i))/c,(P(3,i) + pn(3,i))/c,'b.');
%     view(2)
% %     surfnorm(x,y,z);
% %     pause(0.02)
% end
% x_min = min(P(1,:));
% x_max = max(P(1,:));
% x_mid = median(P(1,:));
% %%%根据 点向式空间直线方程（x - x0） 画法向量
% axe(2)=subplot(222);
% % figure
% plot3(P(1,:),P(2,:),P(3,:),'r.');        %plot绘图函数，分别取P中第1.2.3行所有点作为坐标轴，r表示颜色
% for i=1:length(pn)                    %根据最近邻点连接直线
    
    
%      hold on
%      if(mod(i,100)==0)
%         x=P(1,i):0.001:x_mid;%直线的起点x坐标与终点x坐标，注意起点为场景中第一点，终点为对应其在模型中最近点
%         y= pn(2,i) / pn(1,i) .* (x - P(1,i)) + P(2,i);
%         z= pn(3,i) / pn(1,i) .* (x - P(1,i)) + P(3,i);
%         plot3(x,y,z,'g-');

    
%     %pause(0.00000002)
%      end
% end

quiver3( P(1,:) , P(2,:) , P(3,:)  ,  pn(1,:) , pn(2,:) , pn(3,:) );
xlabel('x');ylabel('y');zlabel('z');
title('源点云法向量显示');
view(2)
% 
% 
% 
% 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%计算特征度
pt=zeros(1,size(P,2));                  %创建1行P阵列数列0阵  
qt=zeros(1,size(Q,2));

[n1,d1] = knnsearch(transpose(P), transpose(P), 'k', 400);      %依次取各点 最近的400个点 ；
                                                                %n1为返回的点的列数，按照距离递增排序；
                                                                %d1为各点与该点的距离，递增排序；
                                                                %结果为n*400阵
n1=transpose(n1);                       %分别对n1，d1取转置为400*n阵
d1=transpose(d1);

  %遍历点云数据矩阵-P 计算每一点对应8邻域的 特征值——pt
% % %for i=1:size(P,2)                                               % size(P,2)返回矩阵P的列数   
% % %    pt(1,i)=1/k*sum(abs(transpose(pn(:,n1(2:k+1,i)))*pn(:,i))); %n1(2:k+1,i)取n1返回距离指标矩阵中第i列的2--k+1行，值为列数；8*1矩阵
% %                                                                  %transpose(pn(:,n1(2:k+1,i)))为取第i个点最近的8个点的法向量并转置，为8*3；
% %                                                                  %transpose(pn(:,n1(2:k+1,i)))*pn(:,i)为再乘i点法向量3*1,值为法向量乘积为8*1
% %                                                                  %abs()为取绝对值函数；sum()为求和函数；
% %                                                                  %pt(1,i)为 第i个点的法向量与其最近8个点的法向量 相乘、取绝对值、累加、取平均值后的结果，最终pt为1*40097          

for i=1:size(P,2)         
    temp_1 = n1(2:k+1,i);                                       %8邻域点列索引（不含查询点）    8*1
    temp_2 = pn(:,temp_1);                                      %8邻域点法向量                 3*8
    temp_3 = transpose(temp_2);                                 %                             8*3  
    temp_4 = temp_3 * pn(:,i);                                  %查询点法向量与邻域法向量  内积 （值越大越平坦） 8*1 = 8*3 3*1
    temp_5 = abs(temp_4);                                          
    temp = sum(temp_5);                                         %                1*1
    pt(1,i)=1/k*temp;                                           %得到第i个点的8邻域特征度量值（法向量内积绝对值和的均值） 1*size(P,2)
end
clear temp temp_1 temp_2 temp_3 temp_4 temp_5

[n2,d2] = knnsearch(transpose(Q), transpose(Q), 'k', 400);
n2=transpose(n2);
d2=transpose(d2);
for i=1:size(Q,2)
    qt(1,i)=1/k*sum(abs(transpose(qn(:,n2(2:k+1,i)))*qn(:,i))); %行向量
end


%%选取特征点
ptt=find(pt<mean(pt));                  % mean(pt)求pt40097平均值，返回一个数；   ptt值pt中为小于平均值的列指标，行向量
qtt=find(qt<mean(qt));
p0=P(:,ptt);                            %提取P阵中 小于特征度量均值 的点
q0=Q(:,qtt);

% figure;                                 %显示p0 q0特征点，特征为该点法向量点成最近8个点的法向量绝对值累加的平均值小于所有平均值的平均
axe(2)=subplot(222);
plot3(p0(1,:),p0(2,:),p0(3,:),'r.');
hold on
plot3(q0(1,:),q0(2,:),q0(3,:),'b.');
title('模板点云与目标点云的特征点粗提取')
view(2)
% daspect([1 1 1]);                      %三维角度出图

fep=[];                                %定义俩个空矩阵，0行0列空矩阵   特征点列索引
feq=[];                    
  %遍历ptt中每一个元素，取出 特征点的列索引--fep、feq  行向量
% % %   
% % %  %for i=ptt                           %表示ptt中为小于平均值的列指标 
% % %    %   [~,I]=min(qt(1,n2(1:k+1,i)));  %n1(1:k+1,i)提取n1中第i列的前9个最近点指标，
% % %                                       %再提取出pt中第i个点与其最近9个点的法向量相乘取绝对值累加取平均值
% % %                                       %min(pt(1,n1(1:k+1,i)))再取其中最小值（第i个点与其最近8个点的法向量相乘取绝对值累加取平均值的最小值）；
% % %                                       %[~,I]表示如果函数有多个输出，为忽略前面的值，将最小值所在 列指标 赋值给I
% % %   


for i=ptt                              %表示ptt中为小于平均值的列指标
    temp   = n1(1:k+1,i);              %ptt对应邻域点列索引（包含查询点）           n1第1行就是查询点本身       列向量
    temp   = pt(1,temp);               %ptt对应邻域点的特征度量值                                行向量
    [~,I]  = min(temp);                %I：特征度量值最小的列索引

    if I==1                            %筛选出 查询点 在其8邻域点中 特征度量值最小 的点作为 关键点
        fep=[fep,i];                   %将i的值填充fep矩阵，fep为1行多列
    end
end
clear temp 

for i=qtt 
    [~,I]=min(qt(1,n2(1:k+1,i)));
    if I==1
        feq=[feq,i];
    end
end

feq0=feq;                             %fep为P中特征点在点云P中的索引
p0=P(:,fep);                          %p0 为特征点坐标矩阵
q0=Q(:,feq);


% figure;
axe(3)=subplot(223);
plot3(p0(1,:),p0(2,:),p0(3,:),'r.');
title('目标点云关键点精提取')
view(2)
% figure;
axe(4)=subplot(224);
plot3(q0(1,:),q0(2,:),q0(3,:),'b.');
title('模板点云关键点精提取')
view(2)
linkaxes(axe,'xy')

%%%clear axe i data1 data2 file1 file2 I k
%%clearvars -except P Q pn qn n1 d1 d11 n2 d2 d22 fep feq feq0

save PFH1.mat%保存工作区所有变量

