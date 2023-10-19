function f = surfaceculfcn(x,target,BregmasOnPano)
%SURFACECULFCN 点xの座標を受け取って角度を最適化することにより与えられた点とFigure上で予測される点の距離を最小化する。
%   h = x(1) : 回転軸のDP軸の高さ
%   theta = x(2) : X軸周りの角度　（deg）
%   u = x(3) : Y軸周りの角度 (deg)
%   standardp : Random sampling によって得られた基準点の座標(mm),targetのindex
%   targetonfig : 基準点の画像上での座標(px)
%ppm = 274.45; % pixel per mm;
h = x(1);

% BregmasOnPano(1,:) は最初にAutostitchに入れた画像の移動先。つまり、Bregma.

% 拡大縮小
surfgrid = [target(:,1)-h,target(:,2),target(:,3)];

% ｘ軸に関して回転
theta = deg2rad(x(2));
u = deg2rad(x(3));
%w = deg2rad(x(4));
rotX = [1 0 0;0 cos(theta) sin(theta);0 -sin(theta) cos(theta)];
trans = [0, 0, 0];
% Y軸に関して回転 1/20 追加
rotY = [cos(u) 0 sin(u);0 1 0;-sin(u) 0 cos(u)];
% Z軸に関して回転 1/20 追加 回転は2軸で十分なので、一自由度削減
%rotZ = [cos(w) -sin(w) 0;sin(w) cos(w) 0;0 0 1];
rotZ = eye(3);
tform = rigid3d(rotX*rotY*rotZ,trans);

ptCloudOut = pctransform(pointCloud(surfgrid),tform);
surfgrid = ptCloudOut.Location;
% % Bregmaの位置推定
% ptCloudOut2 = pctransform(pointCloud([0,-h,0]),tform);
% bregma = ptCloudOut2.Location;
% % Figure上での座標(px)に変換]
% surfgrid_for_standard = [bregma([1,3]);surfgrid(flags(:,3),[1,3])];
% 
% tform_fig = fitgeotrans(BregmasOnPano,surfgrid_for_standard,'lwm',12);%パラメーター候補  ｛lwm 12},{polynomial 4}
% targetonfig = transformPointsInverse(tform_fig,surfgrid(:,[1,3]));

surfgrid_for_standard = surfgrid(:,[2,3]);
% surfgrid_for_standard_mirrored = [-surfgrid_for_standard(:,1),surfgrid_for_standard(:,2)];
% surfgrid_for_standard = [surfgrid_for_standard;surfgrid_for_standard_mirrored];
tform_fig = fitgeotrans(BregmasOnPano,surfgrid_for_standard,'lwm',12); %パラメーター候補  ｛lwm 12},{polynomial 4}
targetonfig = transformPointsInverse(tform_fig,surfgrid(:,[2,3]));

%surfgrid = surfgrid * ppm;
%surfgrid = [-a*surfgrid(:,1),surfgrid(:,2),-b*surfgrid(:,3)];
%targetonfig = [surfgrid(:,1),surfgrid(:,3)]+[ones(size(surfgrid,1),1)*bregma(1),ones(size(surfgrid,1),1)*bregma(2)];
% Figure上での距離を計算
%f = sum(diag(pdist2(targetonfig,BregmasOnPano)),'omitnan');
f = sum(vecnorm(targetonfig-BregmasOnPano,2,2),'omitnan');
end

