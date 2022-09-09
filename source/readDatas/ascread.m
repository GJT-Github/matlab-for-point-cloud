function b = ascread(filename)      %read my file
% b = ascread(filename) ：读取asc点云文件(不限asc文件，因为是从文件操作的角度读取点云数据）
%                         第一行为总点数，之后每行为xyz数据的文件都可以
% filename ：输入点云文件路径
% b        ：输出点云元胞数组，第一个元素为总点数，第二个为点云数据
%
%
%

format long;
fi = fopen(filename,'r');           %openfile  'r'读出参数
if fi < 0
  error(sprintf('File %s not found', filename))
end

templine = 1;                       % 
a = sscanf(fgetl(fi), '%d');        %%fgetl从已经打开的文件中读取一行，并且丢掉末尾的换行符，读一个数据
templine = templine +1;

if length(a)==1                     % .asc文件第一行为点总数
    num_of_points = a(1);
end

pointlist = zeros(3,num_of_points);

for vnum = 1 : num_of_points
	%读取第vnum个点的xyz数据
    coord = sscanf(fgetl(fi), '%e %e %e');  %此时光标指向第二行，第一个点xyz数据，读三个数据

    %检测读取的xyz数据是否缺失
    if length(coord) ~= 3
      errmsg = sprintf('Each vertex line must contain three coordinates (error on line %d)', templine);
      error(errmsg);
    end

    templine = templine +1;
    %将点云数据整合
    pointlist(:,vnum) = coord;
end
%整合点云，输出
b = cell({num_of_points;pointlist});