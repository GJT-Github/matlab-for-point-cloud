clc,clear
% rabbit = pcread('../Datas/rabbit.pcd');
rabbit  = loadpcd('../Datas/rabbit.pcd');

theta = 30;
R = [cosd(theta) sind(theta) 0;
    -sind(theta) cosd(theta) 0;
    0 0 1];
t = [0.05 0.05 0.1];
% T = rigid3d(R,t);
% T = rigidtform3d(R,t);
T = [R t';0 0 0 1];
% tform = affine3d(T);

new_rabbit = pctransform(rabbit,T);
pcshowpair(rabbit,new_rabbit)