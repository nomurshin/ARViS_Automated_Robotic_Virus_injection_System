function [targetonfig,x_fig,tform_fig,tform_fig_inv,BregmasOnPano,errorIdx] = RotationMethod(pano,imgc,target)
%ROTATIONMETHOD この関数の概要をここに記述
%   詳細説明をここに記述

pointsPano = detectKAZEFeatures(pano,'Threshold',0.00001);
[featuresPano, pointsPano] = extractFeatures(pano,pointsPano);
BregmasOnMask = [size(imgc,1)/2,size(imgc,2)/2];
BregmasOnPano = zeros(size(imgc,3),2);
errorIdx = true(size(imgc,3),1);
figure;
for i = 1:size(imgc,3)
    I = squeeze(imgc(:,:,i));
    points = detectKAZEFeatures(I,'Threshold',0.00001);
    [features,points] = extractFeatures(I,points);
    indexPairs = matchFeatures(featuresPano,features);
    pointsA = pointsPano(indexPairs(:,1),:);
    pointsB = points(indexPairs(:,2),:);
    if size(pointsB,1) < 40 && i ~= 1 % 後の処理でbregmaの画像は計算に含めないことになっているため。（because there are few matched points in the pic of bregma.)
        errorIdx(i) = false;
        disp(strcat('No. ',num2str(i),' image is omitted because the number of matched features is too small!'));
    end
    [tform,pointsBm,pointsAm] = estimateGeometricTransform(pointsB,pointsA,'affine');
    %imgBp = imwarp(I,tform,'OutputView',imref2d(size(pano)));
    %pointsBmp = transformPointsForward(tform,pointsBm.Location);
    BregmasOnPano(i,:) = transformPointsForward(tform,BregmasOnMask);
%     subplot(2,2,rem(i,4)+1);
%     showMatchedFeatures(imgBp,pano,pointsBmp,pointsAm);
%     hold on
%     scatter(BregmasOnPano(i,1),BregmasOnPano(i,2),50,'b','filled');
%     legend('mask','pano');
end
%BregmasOnPano = [BregmasOnPano(:,1),-(BregmasOnPano(:,2)-BregmasOnPano(1,2))+BregmasOnPano(1,2)];
% movingPoints = [0 0;standard(:,[1,3])];
% tform_fig = fitgeotrans(BregmasOnPano,movingPoints,'lwm',12);
% targetonfig = transformPointsInverse(tform_fig,target(:,[1,3]));
%figure;
%scatter(BregmasOnPano(:,1),BregmasOnPano(:,2),'filled');
% 脳立体モデルの角度を考慮に入れたHexapod to Figure変換
errorIdx(1) = false;
target_rmd = target(errorIdx(2:end),:);
BregmasOnPano_rmd = BregmasOnPano(errorIdx,:);

options = optimoptions(@fmincon,'Algorithm','active-set','PlotFcn','optimplotfval','UseParallel',false,'MaxIterations',2);
%x_fig = [10,1,0.1,0.1];

fvalmin = 100000;
goodsamples = [];

for j = 1:50
    rsampId = randsample(1:length(target_rmd(:,1)),round(length(target_rmd(:,1))*0.8));
    fun_fig = @(x) surfaceculfcn(x,target_rmd(rsampId,:),BregmasOnPano_rmd(rsampId,:));
    x_fig = [10,1,0.1];
    x0 = x_fig + 2*abs(x_fig).*(rand(1,3)-0.5);
    [xi,fval] = fmincon(fun_fig,x0,[],[],[],[],[-500,-20,-20],[500,20,20],[],options);
    if fval < fvalmin
        fvalmin = fval;
        x_fig = xi;
        goodsamples = rsampId;
    end
end
options = optimoptions(@fmincon,'Algorithm','active-set','PlotFcn','optimplotfval','UseParallel',true,'MaxFunctionEvaluations',2.0e3);
fun_fig = @(x) surfaceculfcn(x,target_rmd(goodsamples,:),BregmasOnPano_rmd(goodsamples,:));
x0 = x_fig + 2*abs(x_fig).*(rand(1,3)-0.5);
[x_fig,fval] = fmincon(fun_fig,x0,[],[],[],[],[-500,-20,-20],[500,20,20],[],options);
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
rotZ = eye(3);
tform = rigid3d(rotX*rotY*rotZ,trans);
ptCloudOut = pctransform(pointCloud(surfgrid),tform);
surfgrid = ptCloudOut.Location;
% Bregmaの位置推定
% ptCloudOut2 = pctransform(pointCloud([0,-h,0]),tform);
% bregma = ptCloudOut2.Location;
% % Figure上での座標(px)に変換]
% surfgrid_for_standard = [bregma([1,3]);surfgrid(flags(:,3),[1,3])];
surfgrid_rmd = surfgrid(errorIdx(2:end),:);
surfgrid_for_standard = surfgrid_rmd(goodsamples,[2,3]);
BregmasOnPano_omitted = BregmasOnPano_rmd(goodsamples,:);
tform_fig = fitgeotrans(BregmasOnPano_omitted,surfgrid_for_standard,'lwm',12); %パラメーター候補  ｛lwm 12},{polynomial 4}
tform_fig_inv = fitgeotrans(surfgrid_for_standard,BregmasOnPano_omitted,'lwm',12);
targetonfig = transformPointsInverse(tform_fig,surfgrid(:,2:3));

end

