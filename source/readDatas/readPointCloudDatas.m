function  [data_source,data_target]= readPointCloudDatas(P_file,Q_file)      %
% [P,Q]= readPointCloudDatas(P_file,Q_file) ����ȡ�����ļ�
%      
%  �������  
%    P_file  : Ŀ������ļ�·��
%    Q_file  : ���������ļ�·��
%
%  �������
%    P     : Ŀ�����XYZ������, 3*n �����ݾ���
%    Q     : ��������XYZ�����ݣ�3*n �����ݾ���
%   
%
%  Author��GJT 
%  E-mail��gjt0114@outlook.com


% file1='../Datas/bun045.asc';
% ind=findstr(file1,'.');
% P_file_Type = file1(ind,end);


[~,~,file_Type_P]=fileparts(P_file);               %��ȡ���ƺ�׺��
[~,~,file_Type_Q]=fileparts(Q_file);               %��ȡ���ƺ�׺��

%% ��������
switch file_Type_P
	case '.asc'
		% ����1
		% data_source  = ascread( 'bun045.asc' );
		% data_source  = data_source{ 2 , 1 };

		% ����2
	    % file1='bun045.asc';
	    %  file2='bun000.asc';
	    data_source   = ascread(P_file);          %��ȡasc���͵��ƣ�ֵΪ2��1�е�cell�������п��Բ�ͬ���ͣ�{1}Ϊ����40097points��{2}Ϊ3��40097���������
% 	    data_target   = ascread(Q_file);          %��ȡasc���͵��ƣ�ֵΪ2��1�е�cell�������п��Բ�ͬ���ͣ�{1}Ϊ����40097points��{2}Ϊ3��40256���������
	    
	    data_source   = data_source{2,1};         %PΪ3��40097�о��󣬵���data1����cell���ڶ���
% 	    data_target   = data_target{2,1};
        

	case '.pcd'
	    % ����3 (����matlab2018���ϰ汾)   pcread : matlab�Դ��ĵ������ݼ��غ���,������pcd��ply�ȵ��Ƹ�ʽ   
        % ע��matlab2018���µİ汾pcread�޷�����.pcd��ʽ
        % data_source = pcread('C:\Users\Administrator\Desktop\vescl\ply\adis800_pc_wave.ply');
        % data_target = pcread('C:\Users\Administrator\Desktop\vescl\result1\scene3_dis500_ref400\sparse_point_cloud.ply');
   
        % ����4 (matlab2018���°汾pcread��ȡpcd�����ļ�����취)
        % https://blog.csdn.net/huacaocha123/article/details/115709874
        % 2018���°汾��pcread����ֻ�ܶ�ȡply��ʽ�����ļ�
        %
        data_source = loadpcd(P_file);
%         data_target = loadpcd(Q_file);
        % ptCloud = pointCloud(data_source');     %ͨ��pointCloud��������������ֵת���ɵ��ƶ���
        % data_source = ptCloud.Location';        %�õ����ǵ���xyzֵ����loadpcd����ֵһ��
        % data_target   = data_target.Location';   
        % pcshow(ptCloud);                        %pcshow�������������Ϊ���ƶ���



	case '.ply'
	    % ����3 (����matlab2018���ϰ汾)   pcread : matlab�Դ��ĵ������ݼ��غ���,������pcd��ply�ȵ��Ƹ�ʽ   
        % ע��matlab2018���µİ汾pcread�޷�����.pcd��ʽ,ֻ�ܶ�ȡ.ply��ʽ�ļ�

        % data_source = pcread('C:\Users\Administrator\Desktop\vescl\ply\adis800_pc_wave.ply');
        % data_target = pcread('C:\Users\Administrator\Desktop\vescl\result1\scene3_dis500_ref400\sparse_point_cloud.ply');
        
	    data_source   = pcread(P_file);          %��ȡasc���͵��ƣ�ֵΪ2��1�е�cell�������п��Բ�ͬ���ͣ�{1}Ϊ����40097points��{2}Ϊ3��40097���������
% 	    data_target   = pcread(Q_file);          %��ȡasc���͵��ƣ�ֵΪ2��1�е�cell�������п��Բ�ͬ���ͣ�{1}Ϊ����40097points��{2}Ϊ3��40256���������
	    
	    data_source   = data_source.Location';         %PΪ3��40097�о��󣬵���data1����cell���ڶ���
% 	    data_target   = data_target.Location';   


        % ����4 (matlab2018���°汾pcread��ȡpcd�����ļ�����취)
        % https://blog.csdn.net/huacaocha123/article/details/115709874
        % pt = loadpcd('bunny.pcd');
        % ptCloud = pointCloud(pt');
        % data_source = ptCloud.Location;
        % % pcshow(ptCloud);                    %��ʾ����

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

