function targetonfig = Surf2FigFnc(x_fig,target,tform_fig)
%Surf2FigFnc 実際の脳表面の座標(mm)から画像上の座標(px)を求める関数。
%   詳細説明をここに記述
h = x_fig(1);

% 拡大縮小
surfgrid = [target(:,1)-h,target(:,2),target(:,3)];

% ｘ軸に関して回転
theta = deg2rad(x_fig(2));
u = deg2rad(x_fig(3));
%w = deg2rad(x_fig(4));
rotX = [1 0 0;0 cos(theta) sin(theta);0 -sin(theta) cos(theta)];
trans = [0, 0, 0];
% Y軸に関して回転 1/20 追加
rotY = [cos(u) 0 sin(u);0 1 0;-sin(u) 0 cos(u)];
% Z軸に関して回転 1/20 追加
%rotZ = [cos(w) -sin(w) 0;sin(w) cos(w) 0;0 0 1];
tform = rigid3d(rotX*rotY,trans);
ptCloudOut = pctransform(pointCloud(surfgrid),tform);
surfgrid = ptCloudOut.Location;
% Figure上での座標(px)に変換]
targetonfig = transformPointsInverse(tform_fig,surfgrid(:,[2,3]));
end

