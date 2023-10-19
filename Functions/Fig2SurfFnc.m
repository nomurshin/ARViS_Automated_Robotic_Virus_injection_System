function target = Fig2SurfFnc(x_fig,targetonfig,tform_fig_inv,F_camera)
%FIG2SURFFNC この関数の概要をここに記述
%   詳細説明をここに記述
h = x_fig(1);

theta = deg2rad(-x_fig(2));
u = deg2rad(-x_fig(3));
%w = deg2rad(-x_fig(4));

% % Figure上での座標(px)からカメラ視点のHexapod座標(mm)に変換
target = transformPointsInverse(tform_fig_inv,targetonfig);
target = [F_camera(target(:,1),target(:,2)),target(:,1),target(:,2)];

rotX = [1 0 0;0 cos(theta) sin(theta);0 -sin(theta) cos(theta)];
trans = [0, 0, 0];
% Y軸に関して回転 1/20 追加
rotY = [cos(u) 0 sin(u);0 1 0;-sin(u) 0 cos(u)];
% Z軸に関して回転 1/20 追加
%rotZ = [cos(w) -sin(w) 0;sin(w) cos(w) 0;0 0 1];
tform = rigid3d(rotY*rotX,trans);
ptCloudOut = pctransform(pointCloud(target),tform);
target = ptCloudOut.Location;

% 拡大縮小
target = [target(:,1)+h,target(:,2),target(:,3)];



end

