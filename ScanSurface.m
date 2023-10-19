% SCANSURFACE In this step, the three-dimensional surface of the brain is scanned using an infrared sensor.
% The skull is removed for accurate scanning.
% Based on the surface data, the camera height is automatically adjusted for fine-focused surface images.
% The acquired images are used to segment blood vessels using a convolutional neural network (SA U-Net).
% The segmented vessel images are stitched together, resulting in a 3D
% surface frame and a 2D image frame.
% The two coordinate systems are transformed, and the vessels are mapped to the 3D surface.

%% Execute here to start the vertical video.
% The camera that captures brain surface from the top
vid = videoinput('gentl', 4, 'RGB8'); %Set appropriate channel for your environment
src = getselectedsource(vid);
vid.FramesPerTrigger = 1;

preview(vid);
Device.WAIT();
Device.PLM('x',7);%set soft lower limit

Device.KEN('zero');
Device.VLS(20);
Device.SST('x y z',[0.1,0.1,0.1]);
%% Accurate movement mode
Device.SST('x y z',[0.01 0.01 0.01])
%% Registration Step 1. Register Laser and Pipette Tip Coordinates 
Device.KEN('zero');
RegNPos = Device.qPOS('x y z');
Device.MOV('x',RegNPos(1)-5);
Device.WAIT();
Device.SST('x y z',[0.05 0.05 0.05])

% Execute here to move laser point to the reference point.
Device.MOV('x',RegNPos(1)-10);
Device.WAIT();
Device.MVR('y z',[-14.4322,0.7099]);
Device.WAIT();
Device.MOV('x',RegNPos(1)-3.4713);
Device.WAIT();

Device.KEN('zero');
Vel = [2,0.2];
for i = 1:2
    RegLPos = LaserRegFnc(Device,v2d,Vel(i)); 
end
Device.DRC(4,'1',17);
Device.DRT ( 2, 4, '0' );
pause(2);
RegD = Device.qDRR(4,1,-1);
N2LH = mean(v2d(RegD(end-20:end,2)));
disp(N2LH);
RegLPos(1) = RegLPos(1) - N2LH;
Device.SST('x y z',[0.05 0.05 0.05]);
Device.VLS(15);
Device.MOV('x y z',RegLPos);
Device.WAIT();
%% Retract the pipette tip using linear actuator
%% Execute here to move the laser point to around bregma.
Device.MOV('x y z',[-3, -10,-5]);
Device.WAIT();
%% Accurate movement mode
Device.SST('x y z',[0.01 0.01 0.01])
%% Move laser points to Bregma 
%% Measure Brain Surface with Laser 
Device.KEN('zero');
Device.DRC(4,'1',17);
Device.DRT ( 2, 4, '0' );
pause(2);
RegD = Device.qDRR(4,1,-1);
Device.MVR('x',-mean(v2d(RegD(end-20:end,2))));
Device.WAIT();
BREGMA_L_Pos = Device.qPOS('x y z');
Device.KSF('BREGMA_L');
% Get Sampling Points by Laser EDITED ON 21/3/15
Device.KEN('BREGMA_L');

tic
SR = 150; %Sampling Rate (Hz)aS
Vel = 20; % Scan speed (mm/sec)
zwidth = params.zwidth_for_laser_scan;
ywidth = params.xwidth_for_laser_scan;
stepw = params.stepw_for_laser_scan;
zsteps = -zwidth:stepw:zwidth;
Device.MOV('x y z',[0,-ywidth,-zwidth]);
Device.WAIT();
Device.VLS(Vel);
Device.RTR(round(10000/SR)); 

Device.DRC(1,'x',2);
Device.DRC(2,'y',2);
Device.DRC(3,'z',2);
Device.DRC(4,'1',17);
Device.DRT ( 6, 1, '0' );
Device.DRT ( 6, 2, '0' );
Device.DRT ( 6, 3, '0' );
Device.DRT ( 6, 4, '0' );
for i = 1:length(zsteps)
    if rem(i,2) == 0
        Device.MOV('z', zsteps(i));
        Device.WAIT();
        Device.MOV('y z',[-ywidth,zsteps(i)]);
        Device.WAIT();
    else
        Device.MOV('z', zsteps(i));
        Device.WAIT();
        Device.MOV('y z',[ywidth,zsteps(i)]);
        Device.WAIT();
    end
end

laserdx = Device.qDRR(1,1,-1);
tlen = length(laserdx(:,1));
laserdy = Device.qDRR(2,1,tlen);
laserdz = Device.qDRR(3,1,tlen);
laserdt = Device.qDRR(13,1,tlen);
laserdd = Device.qDRR(4,1,tlen);
toc

X = laserdx(:,2);
D = -v2d(laserdd(:,2));
Z = laserdz(:,2);
Y = laserdy(:,2);

figure;
scatter3(Y,Z,D,2,'r','filled');
ax = gca;
ax.ZDir = 'reverse';
Device.KEN('BREGMA_L');
Device.MOV('x y z',[0 0 0]);
Device.WAIT();
Device.SST('x y z',[0.05 0.05 0.05])
% Edit surface data
targetim_L = importdata(params.initgrids); % 針座標系
surf_L = [D,Y,Z];
% Crop surface
boundpos = params.surf_boundary_pos;
[k,av] = convhull(boundpos);
inpoly = inpolygon(surf_L(:,2),surf_L(:,3),boundpos(k,1),boundpos(k,2));
surf_L = surf_L(inpoly,:);
% Crop surface end
figure;
scatter3(Y,Z,D,1,'g','filled');
hold on
scatter3(surf_L(:,2),surf_L(:,3),surf_L(:,1),3,'b','filled'); 
ax = gca;
ax.ZDir = 'reverse';
hold off
%% MARMOSET VER. Interpolate Brain Surface
[xData, yData, zData] = prepareSurfaceData( surf_L(:,2), surf_L(:,3), surf_L(:,1) );

% 近似タイプとオプションを設定します。
ft = fittype( 'poly22' );
opts = fitoptions( 'Method', 'LinearLeastSquares' );
opts.Robust = 'Bisquare';

% モデルをデータに近似します。
[fitresult, gof] = fit( [xData, yData], zData, ft, opts );
% 外れ値検出
derror = surf_L(:,1) - fitresult(surf_L(:,2),surf_L(:,3));
figure;
histogram(derror);
pd = fitdist(derror,'Stable');
upper_lim = icdf(pd,0.99);
lower_lim = icdf(pd,0.01);
outlier_id = derror > upper_lim | derror < lower_lim;
hold on
xline(upper_lim);
xline(lower_lim);
figure;
plot( fitresult, [xData, yData], zData );
grid on
hold on
scatter3(surf_L(~outlier_id,2),surf_L(~outlier_id,3),surf_L(~outlier_id,1),5,'g','filled');
hold on
scatter3(surf_L(outlier_id,2),surf_L(outlier_id,3),surf_L(outlier_id,1),5,'r','filled');
ax = gca;
ax.ZDir = 'reverse';
surf_L_rmd = surf_L(~outlier_id,:);

F = scatteredInterpolant(surf_L_rmd(:,2),surf_L_rmd(:,3),surf_L_rmd(:,1),'natural');
targetim_L(:,1) = F(targetim_L(:,2),targetim_L(:,3)); 
%% MOUSE VER. Interpolate Brain Surface:
tic
if max(size(gcp)) == 0 % parallel pool needed
    parpool % create the parallel pool
end
nonlcon = @divcon;
centerVid = abs(surf_L(:,2)) < 0.3;
[e_surf,x] = EstimateSurface(surf_L(~centerVid,:),template,nonlcon);
e_centerVid = abs(e_surf(:,2))<0.3;
F = scatteredInterpolant(e_surf(~e_centerVid,2),e_surf(~e_centerVid,3),e_surf(~e_centerVid,1),'natural');
targetim_L(:,1) = F(targetim_L(:,2),targetim_L(:,3));

figure; % ここで表示されるプロットはHexapod座標系と等しい。
scatter3(e_surf(:,2),e_surf(:,3),e_surf(:,1),1,'b','filled');
hold on;
scatter3(targetim_L(:,2),targetim_L(:,3),targetim_L(:,1),5,'r','filled');
hold on;
scatter3(surf_L(:,2),surf_L(:,3),surf_L(:,1),5,'g','filled');
ax = gca;
ax.ZDir = 'reverse';
% 外れ値検出
derror = surf_L(:,1) - F(surf_L(:,2),surf_L(:,3));
figure;
histogram(derror);
pd = fitdist(derror,'Stable');
upper_lim = icdf(pd,0.975);
lower_lim = icdf(pd,0.025);
outlier_id = derror > upper_lim | derror < lower_lim;
hold on
xline(upper_lim);
xline(lower_lim);

figure;
scatter3(surf_L(~outlier_id,2),surf_L(~outlier_id,3),surf_L(~outlier_id,1),5,'g','filled');
hold on
scatter3(surf_L(outlier_id,2),surf_L(outlier_id,3),surf_L(outlier_id,1),5,'r','filled');
ax = gca;
ax.ZDir = 'reverse';
surf_L_rmd = surf_L(~outlier_id,:);
F = scatteredInterpolant(surf_L_rmd(:,2),surf_L_rmd(:,3),surf_L_rmd(:,1),'natural');

%% Capture Vascular Patterns on the Brain Surface
Device.KEN('BREGMA_L');
P2NDiffV = [RegNPos(1)-RegLPos(1),RegNPos(2)-RegLPos(2),RegNPos(3)-RegLPos(3)];
F_L = scatteredInterpolant(surf_L_rmd(:,2)+P2NDiffV(2),surf_L_rmd(:,3)+P2NDiffV(3),surf_L_rmd(:,1)+P2NDiffV(1)); % BREGMA＿L座標系から見た時の針座標系(したがって針座標系の原点は P2NDiff)
% スティッチ用の画像を取得したLaser'座標系から実際に動く針座標系への変換ベクトル(Laser'座標系はLaser座標系からカメラのフォーカスが合うように平行移動した座標)
Device.MOV('x y z',[F_L(P2NDiffV(2),P2NDiffV(3)) P2NDiffV(2) P2NDiffV(3)]);% ここで画面中央より少し下方にBregmaが来るはず。
Device.WAIT();
% Get Image for stitching
% ここでtargetim_LはBREGMA_L座標系において扱われている。
Device.KEN('BREGMA_L');
standard_L = zeros(size(targetim_L,1),3);
tic
start(vid);
for i = 1:length(targetim_L(:,1))
    Device.VLS(15)
    Device.MOV('y z x u v w',[targetim_L(i,2)+P2NDiffV(2),targetim_L(i,3)+P2NDiffV(3),...
        F_L(targetim_L(i,2)+P2NDiffV(2),targetim_L(i,3)+P2NDiffV(3)),0,0,0]);
    Device.WAIT()
    %stoppreview(vid);
    pause(0.15)
    vidimage = getsnapshot(vid);
    %preview(vid)
    standard_L(i,:) = Device.qPOS('x y z'); %BREGMA_L座標系で針先を脳表面に移動させたときの座標
    save(strcat('C:\Users\RVIS\Documents\PIMikroMoveMacro\',filename_image,'\standard',num2str(i),'.mat'), 'vidimage');
    clear vidimage;
end
toc
stop(vid);
standard_N = [standard_L(:,1)-P2NDiffV(1),standard_L(:,2)-P2NDiffV(2),standard_L(:,3)-P2NDiffV(3)]; % 針座標系における画像中心の座標（原点はBREGMA_L)
Device.KEN('BREGMA_L');
Device.MOV('x y z u v w',[F_L(P2NDiffV(2),P2NDiffV(3)),P2NDiffV(2),P2NDiffV(3),0,0,0])
Device.WAIT();
Device.SST('x y z',[0.1 0.1 0.1]);
% Set Bregma as new cordination
% Do after moving to bregma
%stoppreview(vid)
Device.KEN('zero')
BreNPos = Device.qPOS('x y z');
%Set Bregma as new cordination
Device.KSF('BREGMA')
Device.KEN('BREGMA')
start(vid);
vidimage = getsnapshot(vid);
save(strcat('C:\Users\RVIS\Documents\PIMikroMoveMacro\',filename_image,'\standard_bregma.mat'), 'vidimage');
imwrite(vidimage,strcat('C:\Users\RVIS\Documents\PIMikroMoveMacro\',filename_image,'\bregma.png'));
clear vidimage;
Device.MOV('x',-10);
Device.WAIT();
%% Return the pipette tip to the original position
%% Crop Image Data for SA-UNet
%start(vid);
vidimage = getsnapshot(vid);%getdata
save(strcat('C:\Users\RVIS\Documents\PIMikroMoveMacro\',filename_image,'\standard_for_needle.mat'), 'vidimage');
imwrite(vidimage,strcat('C:\Users\RVIS\Documents\PIMikroMoveMacro\',filename_image,'\needle.png'));
stop(vid)
clear vidimage;

% Check the bregma position 
load(strcat(filename_image,'/standard_for_needle.mat'));
imgs = cell(size(targetim_L,1)+1,1);
figure;
imshow(vidimage);
needle = vidimage;
load(strcat(filename_image,'/standard_bregma.mat'));
imgs = cell(size(targetim_L,1)+1,1);
imgs{1} = vidimage;

bregma = GetNeedleTip(needle);
figure;imshow(needle);hold on;
scatter(bregma(1),bregma(2));

figsize = params.figsize;
Unet_input = 512; % Constant
resize_scale = params.resize_scale;
cropsize_for_stitch = Unet_input*resize_scale;
load(strcat(filename_image,'/standard_bregma.mat'));
imwrite(imresize(imcrop(vidimage,[bregma(1)-cropsize_for_stitch/2,bregma(2)-cropsize_for_stitch/2,cropsize_for_stitch-1,cropsize_for_stitch-1]),[Unet_input,Unet_input]),strcat(filename_image,'\u_net\standard0.tif'));

for i = 2:size(targetim_L,1)+1
    load(strcat(filename_image,'/standard',num2str(i-1),'.mat'));
    imgs{i} = vidimage;
    imwrite(imresize(imcrop(vidimage,[bregma(1)-cropsize_for_stitch/2,bregma(2)-cropsize_for_stitch/2,cropsize_for_stitch-1,cropsize_for_stitch-1]),[Unet_input,Unet_input]),strcat(filename_image,'\u_net\standard',num2str(i-1),'.tif'));
end

num = size(imgs,1);

% local image に変換
imgc = zeros(figsize,figsize,num,'uint8');
imgcs = zeros(figsize,figsize,3,num,'uint8');%位置確認実験用

for i = 1:num
    imgc(:,:,i) = rgb2gray(imcrop(imgs{i},[bregma(1)-figsize/2,bregma(2)-figsize/2,figsize-1,figsize-1]));
    imgcs(:,:,:,i) = imcrop(imgs{i},[bregma(1)-figsize/2,bregma(2)-figsize/2,figsize-1,figsize-1]);%位置確認実験用
end

for i = 1:size(imgc,3)
    imwrite(squeeze(imgc(:,:,i)),strcat(filename_image,'/panoimgs/mask_',num2str(i),'.png'));
    imwrite(squeeze(imgcs(:,:,:,i)),strcat(filename_image,'/panoimgs/img_',num2str(i),'.png'));
end


function [c,ceq] = divcon(x)
c(1) = x(1)/x(2) - 1.5;
c(2) = 1/1.5 - x(1)/x(2);
ceq = [];
end