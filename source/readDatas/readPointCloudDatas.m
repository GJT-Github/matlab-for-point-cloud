function  [data_source,data_target]= readPointCloudDatas(P_file,Q_file)      %
% [P,Q]= readPointCloudDatas(P_file,Q_file) ：读取点云文件
%      
%  输入参数  
%    P_file  : 目标点云文件路径
%    Q_file  : 场景点云文件路径
%
%  输出参数
%    P     : 目标点云XYZ点数据, 3*n 的数据矩阵
%    Q     : 场景点云XYZ点数据，3*n 的数据矩阵
%   
%
%  Author：GJT 
%  E-mail：gjt0114@outlook.com


% file1='../Datas/bun045.asc';
% ind=findstr(file1,'.');
% P_file_Type = file1(ind,end);


[~,~,file_Type_P]=fileparts(P_file);               %获取点云后缀名
[~,~,file_Type_Q]=fileparts(Q_file);               %获取点云后缀名

%% 数据载入
switch file_Type_P
	case '.asc'
		% 方法1
		% data_source  = ascread( 'bun045.asc' );
		% data_source  = data_source{ 2 , 1 };

		% 方法2
	    % file1='bun045.asc';
	    %  file2='bun000.asc';
	    data_source   = ascread(P_file);          %读取asc类型点云，值为2行1列的cell矩阵（俩行可以不同类型）{1}为点数40097points，{2}为3行40097列坐标矩阵
% 	    data_target   = ascread(Q_file);          %读取asc类型点云，值为2行1列的cell矩阵（俩行可以不同类型）{1}为点数40097points，{2}为3行40256列坐标矩阵
	    
	    data_source   = data_source{2,1};         %P为3行40097列矩阵，等于data1矩阵（cell）第二行
% 	    data_target   = data_target{2,1};
        

	case '.pcd'
	    % 方法3 (仅限matlab2018以上版本)   pcread : matlab自带的点云数据加载函数,可载入pcd和ply等点云格式   
        % 注：matlab2018以下的版本pcread无法处理.pcd格式
        % data_source = pcread('C:\Users\Administrator\Desktop\vescl\ply\adis800_pc_wave.ply');
        % data_target = pcread('C:\Users\Administrator\Desktop\vescl\result1\scene3_dis500_ref400\sparse_point_cloud.ply');
   
        % 方法4 (matlab2018以下版本pcread读取pcd点云文件解决办法)
        % https://blog.csdn.net/huacaocha123/article/details/115709874
        % 2018以下版本的pcread函数只能读取ply格式点云文件
        %
        data_source = loadpcd(P_file);
%         data_target = loadpcd(Q_file);
        % ptCloud = pointCloud(data_source');     %通过pointCloud函数将点云坐标值转换成点云对象
        % data_source = ptCloud.Location';        %得到的是点云xyz值，和loadpcd返回值一样
        % data_target   = data_target.Location';   
        % pcshow(ptCloud);                        %pcshow函数的输入参数为点云对象



	case '.ply'
	    % 方法3 (仅限matlab2018以上版本)   pcread : matlab自带的点云数据加载函数,可载入pcd和ply等点云格式   
        % 注：matlab2018以下的版本pcread无法处理.pcd格式,只能读取.ply格式文件

        % data_source = pcread('C:\Users\Administrator\Desktop\vescl\ply\adis800_pc_wave.ply');
        % data_target = pcread('C:\Users\Administrator\Desktop\vescl\result1\scene3_dis500_ref400\sparse_point_cloud.ply');
        
	    data_source   = pcread(P_file);          %读取asc类型点云，值为2行1列的cell矩阵（俩行可以不同类型）{1}为点数40097points，{2}为3行40097列坐标矩阵
% 	    data_target   = pcread(Q_file);          %读取asc类型点云，值为2行1列的cell矩阵（俩行可以不同类型）{1}为点数40097points，{2}为3行40256列坐标矩阵
	    
	    data_source   = data_source.Location';         %P为3行40097列矩阵，等于data1矩阵（cell）第二行
% 	    data_target   = data_target.Location';   


        % 方法4 (matlab2018以下版本pcread读取pcd点云文件解决办法)
        % https://blog.csdn.net/huacaocha123/article/details/115709874
        % pt = loadpcd('bunny.pcd');
        % ptCloud = pointCloud(pt');
        % data_source = ptCloud.Location;
        % % pcshow(ptCloud);                    %显示点云

	otherwise
		error(sprintf('File %s not a right type of can be handled point cloud file!', P_file))
end


switch file_Type_Q
	case '.asc'
        data_target   = ascread(Q_file);
        data_target   = data_target{2,1};
    case '.pcd'
        data_target = loadpcd(Q_file);
    case '.ply'
        data_target   = pcread(Q_file); 
        data_target   = data_target.Location';   
   	otherwise
		error(sprintf('File %s not a right type of can be handled point cloud file!', Q_file))
end

