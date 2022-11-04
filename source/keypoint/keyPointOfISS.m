function [p0,q0,fep,feq,feq0,n1,d1,n2,d2] = keyPointOfISS(P,Q, r, e1,e2,idx,dis)
% idx_feature = My_ISS(p, r, e1,e2,idx,dis)  : ISS内在签名特征点提取
%
% https://download.csdn.net/download/xxiaotouming/10325431?utm_medium=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&depth_1-utm_source=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&dest=https%3A%2F%2Fdownload.csdn.net%2Fdownload%2Fxxiaotouming%2F10325431&spm=1003.2020.3001.6616.4
% 输入参数
%    p    : 点云矩阵  3 * n
%    r    : 邻域半径  单位：m
%    e1   : 中间特征值与最大特征值之比的 阈值
%    e2   : 中间特征值与最小特征值之比的 阈值
%    idx  : 邻域点索引，元包数组，从小到大距离排序，第一个是邻域中心点，即查询点
%    dis  : 邻域点距离，元包数组，从小到大距离排序，第一个是邻域中心点，即查询点
%
% 输出参数
%   idx_feature  : 点云p的ISS特征点索引  p为3*n  idx_feature为 p 列索引
%
%
%
%
%  Author：https://download.csdn.net/download/xxiaotouming/10325431?utm_medium=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&depth_1-utm_source=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&dest=https%3A%2F%2Fdownload.csdn.net%2Fdownload%2Fxxiaotouming%2F10325431&spm=1003.2020.3001.6616.4
%          adiusted by GJT
%  E-mail：gjt0114@outlook.com  of GJT



  %显示
  displayer = displayFunction;                %for Debug

  global r_P_k r_Q_k;

  %400邻域检索
  [n1,d1,n2,d2]=Knncaculate(P,Q);

%文献复现 2.1.1自适应点云平均距离计算
  paper = paperISS;
  r_P_k = paper.paper(d1) * 6;
  r_Q_k = paper.paper(d2) * 6;
 

% for i=1:15                                %for Debug

  %ISS关键点提取
    % [p0,q0,fep,feq,feq0]=ISSCaculate(P ,Q ,r ,r ,e1 ,e2 ,idx ,dis);
    [p0,q0,fep,feq,feq0]=ISSCaculate(P ,Q ,r_P_k  ,r_Q_k,e1 ,e2 );

    % displayer.displayFinalPickKeyPoint(p0,q0); %for Debug
    figure;
    % plot3(P(1,:),P(2,:),P(3,:),'y.');
    % hold on
    plot3(p0(1,:),p0(2,:),p0(3,:),'r.');
    title(num2str(size(p0,2)));
    % play_ball(p0);
    % figure;
    % plot3(q0(1,:),q0(2,:),q0(3,:),'b.');

%文献复现 2.2基于球邻域的边界点判断 优化关键点
    % border_point = paper.borderPoint(P);
    p0 = paper.borderPoint(P,p0);
    

    % num_p0_r(i) = size(p0,2);                %for Debug

%     figure;                                      %for Debug
    % displayer.displayFinalPickKeyPoint(p0,q0); %for Debug
    hold on
    plot3(p0(1,:),p0(2,:),p0(3,:),'b.');
    title(num2str(size(p0,2)),['e1=',num2str(e1),'e2=',num2str(e2)]);
    % play_ball(p0);
     axis equal;
    % i                                        %for Debug
% end                                        %for Debug

  % figure                                     %for Debug
  % plot(num_p0_r)                             %for Debug 
  % xlabel('i * den')                          %for Debug


end


function [n1,d1,n2,d2]=Knncaculate(P,Q)

  [n1,d1] = knn(P);
  [n2,d2] = knn(Q);

end

function [n1,d1] = knn(P)
    [n1,d1] = knnsearch(transpose(P), transpose(P), 'k', 400);      %依次取各点 最近的400个点 ；
                                                                    %n1为返回的点的列数，按照距离递增排序；
                                                                    %d1为各点与该点的距离，递增排序；
                                                                    %结果为n*400阵
    n1=transpose(n1);                       %分别对n1，d1取转置为400*n阵
    d1=transpose(d1);

end

function [p0,q0,fep,feq,feq0]=ISSCaculate(P,Q, r_P,r_Q, e1,e2)
% 计算ISS关键点
    fep = My_ISS(P, r_P, e1,e2);
    feq = My_ISS(Q, r_Q, e1,e2);

    feq0 = feq;

    p0 = P(:,fep);
    q0 = Q(:,feq);


end



function play_ball(p0)
    global r_P_k r_Q_k;
    % 绘制集合球
    aplha=0:pi/40:2*pi;
    for i=1:10:size(p0,2)

        % r=2;        
        x=r_P_k *cos(aplha) + p0(1,i);
        y=r_P_k *sin(aplha) + p0(2,i);
        hold on
        % plot3(x,y,repmat(int32(p0(3,1)),size(x),1),'b-');
        plot3(x,y,repelem(p0(3,i),size(x,2)),'g-')
        plot3(p0(1,i),p0(2,i),p0(3,i),'g*');

        x=r_P_k *cos(aplha) + p0(1,i);
        z=r_P_k *sin(aplha) + p0(3,i);
        hold on
        % plot3(x,y,repmat(int32(p0(3,1)),size(x),1),'b-');
        plot3(x,repelem(p0(2,i),size(x,2)),z,'g-')

        y=r_P_k *cos(aplha) + p0(2,i);
        z=r_P_k *sin(aplha) + p0(3,i);
        hold on
        % plot3(x,y,repmat(int32(p0(3,1)),size(x),1),'b-');
        plot3(repelem(p0(1,i),size(x,2)),y,z,'g-')
        hold off
        
    end

end
