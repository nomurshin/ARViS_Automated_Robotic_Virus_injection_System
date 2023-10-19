%% Run Autostitch and save pano.jpg to /masks directory.
%% load mask made by U-net
maskc = zeros(figsize,figsize,size(targetim_L,1)+1,'uint8');
dbregma = bregma-[4024/2,3036/2];
for i = 0:size(targetim_L,1)
    vidimage = imresize(imread(strcat('C:\Users\RVIS\Documents\PIMikroMoveMacro\',filename_image,'\masks\standard',num2str(i),'.png')),resize_scale);
    %vidimage = imresize(imread(strcat('.\masks\standard',num2str(i),'.png')),resize_scale);
    if i == 0
            maskbregma = imcrop(vidimage,[size(vidimage,2)/2-cropsize_for_stitch/2,size(vidimage,1)/2-cropsize_for_stitch/2,cropsize_for_stitch-1,cropsize_for_stitch-1]);
            maskc(:,:,i+1) = imcrop(vidimage,[size(vidimage,2)/2-figsize/2,size(vidimage,1)/2-figsize/2,figsize-1,figsize-1]);
    else
        mc = imcrop(vidimage,[size(vidimage,2)/2-figsize/2,size(vidimage,1)/2-figsize/2,figsize-1,figsize-1]);
        maskc(:,:,i+1) = mc;
    end
end
% Make classifier of pixels.
maskc_bold = BoldIsolate(maskc,[5,3]);
randid = randperm(size(imgcs,4),12);
[TC,ValAcc] = trainPixelClassifier(table(reshape(imgcs(:,:,1,randid),[],1),reshape(imgcs(:,:,2,randid),[],1),reshape(imgcs(:,:,3,randid),[],1),reshape(maskc_bold(:,:,randid),[],1)));

% Get panorama image from Autostitch.exe & Get affine transform from mask images to panorama image

pano = rgb2gray(imread(strcat(filename_image,'/masks/pano.jpg')));

targetim_L(:,1) = F(targetim_L(:,2),targetim_L(:,3)); %再実行用
[targetonfig,x_fig,tform_fig,tform_fig_inv,BregmasOnPano,errorIdx] = RotationMethod(pano,maskc,targetim_L); 

e_surf_camera = [surf_L_rmd(:,1)-x_fig(1),surf_L_rmd(:,2),surf_L_rmd(:,3)]; % 21/4/22 e_surf -> standard_l_rmd
theta = deg2rad(x_fig(2));
u = deg2rad(x_fig(3));

% カメラ視点のEstimated surfaceモデルを作成
% Estimated Surfaceを垂直方向からカメラの視点に変換
rotX = [1 0 0;0 cos(theta) sin(theta);0 -sin(theta) cos(theta)];
trans = [0, 0, 0];
% Y軸に関して回転 1/20 追加
rotY = [cos(u) 0 sin(u);0 1 0;-sin(u) 0 cos(u)];
% Z軸に関して回転 1/20 追加

rotZ = eye(3);
tform = rigid3d(rotX*rotY*rotZ,trans);
ptCloudOut = pctransform(pointCloud(e_surf_camera),tform);
e_surf_camera = ptCloudOut.Location;

F_camera = scatteredInterpolant(e_surf_camera(:,2),e_surf_camera(:,3),e_surf_camera(:,1));

%%% OUTPUT %%%
% Function object to convert the coordinates on the stitched image to those
% on 3D surface point clouds.
F2S = @(x) Fig2SurfFnc(x_fig,x,tform_fig_inv,F_camera);
% Function object to convert the coordinates on 3D surface point clouds
% to those on the stitched image.
S2F = @(x) Surf2FigFnc(x_fig,x,tform_fig);

figure;
imshow(pano);
hold on
scatter(targetonfig(:,1),targetonfig(:,2));
hold on
scatter(BregmasOnPano(:,1),BregmasOnPano(:,2),'r','filled');
% Edited on 1/26
mask_p = pano > 50;
figure;
imshow(mask_p);