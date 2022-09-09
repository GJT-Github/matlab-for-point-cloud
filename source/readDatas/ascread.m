function b = ascread(filename)      %read my file
% b = ascread(filename) ����ȡasc�����ļ�(����asc�ļ�����Ϊ�Ǵ��ļ������ĽǶȶ�ȡ�������ݣ�
%                         ��һ��Ϊ�ܵ�����֮��ÿ��Ϊxyz���ݵ��ļ�������
% filename ����������ļ�·��
% b        ���������Ԫ�����飬��һ��Ԫ��Ϊ�ܵ������ڶ���Ϊ��������
%
%
%

format long;
fi = fopen(filename,'r');           %openfile  'r'��������
if fi < 0
  error(sprintf('File %s not found', filename))
end

templine = 1;                       % 
a = sscanf(fgetl(fi), '%d');        %%fgetl���Ѿ��򿪵��ļ��ж�ȡһ�У����Ҷ���ĩβ�Ļ��з�����һ������
templine = templine +1;

if length(a)==1                     % .asc�ļ���һ��Ϊ������
    num_of_points = a(1);
end

pointlist = zeros(3,num_of_points);

for vnum = 1 : num_of_points
	%��ȡ��vnum�����xyz����
    coord = sscanf(fgetl(fi), '%e %e %e');  %��ʱ���ָ��ڶ��У���һ����xyz���ݣ�����������

    %����ȡ��xyz�����Ƿ�ȱʧ
    if length(coord) ~= 3
      errmsg = sprintf('Each vertex line must contain three coordinates (error on line %d)', templine);
      error(errmsg);
    end

    templine = templine +1;
    %��������������
    pointlist(:,vnum) = coord;
end
%���ϵ��ƣ����
b = cell({num_of_points;pointlist});