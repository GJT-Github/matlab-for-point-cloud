function n = lsqnormest(p, k)      % 定义子函数，对应调用变量P=p,k=k
% PCA主元分析法求法向量
% p : 3*n的数值矩阵
% k : k近邻参数

m = size(p,2);                     % 返回p矩阵的列数
n = zeros(3,m);                    % 定义3行m列的0阵  

% 求取最近零点，利用kunsearch函数
neighbors = transpose( knnsearch( transpose(p) , transpose(p) , 'k', k+1 ) ); % transpose转置矩阵transpose（p）为n*3矩阵，理解为n个行向量（点）
                                                                              % Idx = knnsearch(X,Y,Name,Value) 返回一个列向量列表（行号），X中点与Y中每个点（行数递增）最近的点的行数，还可指定要搜索的最近领域的数量Value及搜索中使用的距离度量Name（即使用哪种距离）。
                                                                              % 数量取k+1原因是X,Y相同，其中一个点为该点本身，距离为0，kunsearch返回值为一个n行k+1列矩阵，（i，j）代表X中距离Y中第i个点第j近点的行号，再进行转置，结果为k+1*n矩阵，值为p矩阵的列数
for i = 1:m                                % for循环，i循环变量，循环终止量m为p矩阵列数，eg：40097
    x = p( : , neighbors( 2 : end , i) );  %  x = p（所有行1-3，（neighbors第2行到最后一行，i）），把p中第i个点的最近点neighbors中的值（列数）即p第i点的最近点赋值给x，x为3*8矩阵，其值为8个最近点的坐标值
    p_bar = 1 / k * sum( x , 2 );          % b=sum(a,dim); a表示矩阵；dim等于1或者2，1表示每一列进行求和，2表示每一行进行求和。 p_bar为一个3*1列向量，表示所有最近点的某轴坐标和求平均值（领域所有点的中心点）
    
    % K邻域协方差矩阵
    P = (x - repmat(p_bar,1,k)) * transpose(x - repmat(p_bar,1,k));     % 构建协方差矩阵中的y*yT
                                                                        % B=repmat(A,M,N)矩阵B是矩阵A的复制品，其中B的维度为[size(A,1)*M,size(A,2)*N]，此处repmat(p_bar,1,k)表示3行8列矩阵，每列都表示中心点
                                                                        % x - repmat(p_bar,1,k)表示各领域点与中心点的差向量，为3*8矩阵
                                                                        % 令y = x - repmat(p_bar,1,k)，P = y*yT，为3*3矩阵
    % 特征向量、特征值计算
    [V,D] = eig(P);                % [V,D]=eig(A)：求矩阵P的全部特征值，构成对角0阵D，并求A的特征向量构成V的列向量。
    
    % 提取 最小 特征值
    [~, idx] = min(diag(D));       % diag(D)为D矩阵中对角元素组成的向量，min(diag(D))表示提取最小特征值,idx表示最小特征值所在位置的列索引
    
    % 最小特征值 对应 特征向量 为 K邻域拟合平面的法向量
    n(:,i) = V(:,idx);             

    
    % 规定方向指向
    flag = p(:,i) - p_bar;         % flag为p中每个点与其领域中心点的差向量
    if dot( n(:,i) , flag ) < 0    % dot表示列向量点积，大于零锐角，小于零钝角
        n(:,i) = -n(:,i);          % 钝角时，法向量反向
    end
end