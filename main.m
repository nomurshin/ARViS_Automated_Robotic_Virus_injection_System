% Automated injection workflow - Preparation step
% In this step, a crystal pipette ordinarily used in electrophysiology or virus injection is prepared.
% Because the pipette is hand-made and disposable, its length may vary, causing positioning errors.
% Therefore, a calibration step is essential to account for this variability.
% The pipette is loaded with a solution containing virus or chemical for the injection.

%% Device Initialization and Data I/O Process
C887 = DeviceInit();
Device = DeviceWrapper(C887);
% Set Grid Position 
addpath(genpath('Functions'));
addpath(genpath('images'));
addpath(genpath('Data'));
params = importdata('Data/params_mouse.mat');
% params = importdata('Data/params_marmo.mat') % Parameters for marmoset
template = importdata(params.template);

v2dmodel = importdata('Data/v2dmodel.mat');
targetim_L = importdata(params.initgrids);
injdepth = params.injdepth;
v2d = @(x) v2dmodel(x);

if exist(strcat('images/',date),'file') ~= 7
    filename_image = strcat('images/',date);
    mkdir(filename_image);
    addpath(filename_image);
    filename_masks = strcat(filename_image,'/masks');
    mkdir(filename_masks);
    addpath(filename_masks);
    filename_unet = strcat(filename_image,'/u_net');
    mkdir(filename_unet);
    addpath(filename_unet);
    filename_panoimgs = strcat(filename_image,'/panoimgs');
    mkdir(filename_panoimgs);
    addpath(filename_panoimgs);
else 
    i = 2;
    while exist(strcat('images/',date,'_',num2str(i)),'file') == 7
        i = i+1;
    end
    filename_image = strcat('images/',date,'_',num2str(i));
    mkdir(filename_image);
    addpath(filename_image);
    filename_masks = strcat(filename_image,'/masks');
    mkdir(filename_masks);
    addpath(filename_masks);
    filename_unet = strcat(filename_image,'/u_net');
    mkdir(filename_unet);
    addpath(filename_unet);
    filename_panoimgs = strcat(filename_image,'/panoimgs');
    mkdir(filename_panoimgs);
    addpath(filename_panoimgs);
end
%% Needle detection and camera calibration. Run "TCPCalibration.m"
% Calibration step
% In this step, the pipette is calibrated to account for the variability in its length.
% The calibration process takes about 90 minutes but is performed automatically.
% The calibration is essential to reduce positioning errors during the injection.


%% Recognition step. Run "ScanSurface.m"
% In this step, the three-dimensional surface of the brain is scanned using an infrared sensor.
% The skull is removed for accurate scanning.
% Based on the surface data, the camera height is automatically adjusted for fine-focused surface images.
% The acquired images are used to segment blood vessels using a convolutional neural network (SA U-Net).
% The segmented vessel images are stitched together, resulting in a 3D
% surface frame and a 2D image frame.
% The two coordinate systems are transformed, and the vessels are mapped to the 3D surface.



%% Segmentation step. Run "SA-UNet"

%% Integration of 3D brain structure and 2D vessel segmentation. Run "IntegrateSurface.m"

%% Injection Planning Part A
% In this step, the system automatically selects injection sites where the risk of bleeding is minimized.
% The normal surface vectors are computed to insert the needle vertically to the surface.
% This orientation preserves the main vessels, as they are oriented perpendicular to the surface.

%%%%%%%%% Parameters for injection plannning %%%%%%%%%%%%%%

targetlen = 0.4; % Distance between injection sites(mm)
distlimit = 0.065; % Safe margin from the vessels(mm)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set the area where the Injection Site will be placed

global boundary
boundary = params.boundary;
figure;
imshow(mask_p);
roi_b = cell(size(boundary,1),1);
for i = 1:size(boundary,1)
    if i < 7
        roi_b{i} = images.roi.Point(gca,'Position',boundary(i,:),'Color','y');
    else
        roi_b{i} = images.roi.Point(gca,'Position',boundary(i,:),'Color','g');
    end
    roi_b{i}.Label = num2str(i);
    addlistener(roi_b{i},'ROIMoved',@getroiposEvent_boundary);
end
%% Injection Planning Part B
% Calculate the distance between vessels and non-vessels.
result = CulcVesselDistMat(mask_p,boundary,F2S);

tic
target = OptimLocate(result,boundary,targetlen,distlimit,F2S);
toc
targetonfig = S2F(target);
global targetonfig_new
targetonfig_new = targetonfig;
% Check the targetonfig_new 

target_new = F2S(targetonfig_new);

[yv,xv] = find(mask_p);
[k_v,~] = convhull(boundary);
[in_v,on_v] = inpolygon(xv,yv,boundary(k_v,1),boundary(k_v,2));
VesselsOnFig = [xv(in_v|on_v),yv(in_v|on_v)];
vesselsOnSurf = F2S(VesselsOnFig);

figure;
scatter3(surf_L_rmd(:,2),surf_L_rmd(:,3),surf_L_rmd(:,1),1,'y','filled');
hold on
scatter3(target_new(:,2),target_new(:,3),target_new(:,1),'b','filled');
hold on
scatter3(target(:,2),target(:,3),target(:,1),'g');
hold on
scatter3(vesselsOnSurf(:,2),vesselsOnSurf(:,3),vesselsOnSurf(:,1),0.1,'r');
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');
axis equal
% Interactively change the position of ROI
figure;
%imshow(pano);
imshow(mask_p);
roi = cell(size(targetonfig_new,1),1);
for i = 1:size(targetonfig_new,1)
    roi{i} = images.roi.Point(gca,'Position',targetonfig_new(i,:));
    roi{i}.Label = num2str(i);
    addlistener(roi{i},'ROIMoved',@getroiposEvent);
end

%% Injection Planning Part C
% Renumbering injection sites,and save estimated surface & injection sites.
targetonfig_new = Renum_from_center(targetonfig_new,BregmasOnPano(1,:),200); %Sort the order of target_new for saving time.

figure;
imshow(~mask_p)
hold on
scatter(targetonfig_new(:,1),targetonfig_new(:,2),25,'r','filled');
for i = 1:size(targetonfig_new,1)
    text(targetonfig_new(i,1)+5,targetonfig_new(i,2),cellstr(num2str(i)),'Color',[0.9 0.9 0.9],'FontSize',13.5,'FontWeight','bold');
    text(targetonfig_new(i,1)+5,targetonfig_new(i,2),cellstr(num2str(i)),'Color',[0 0.5 1],'FontSize',13,'FontWeight','bold');
end

h = x_fig(1);
theta = deg2rad(x_fig(2));
u = deg2rad(x_fig(3));
%w = deg2rad(x_fig(4));
hold off
saveas(gcf,strcat(filename_image,'/InjectionSites_',date,'.fig'));
% 3/8 Edited
target_new = F2S(targetonfig_new);


figure;
scatter3(surf_L_rmd(:,2),surf_L_rmd(:,3),surf_L_rmd(:,1),1,'g','filled');
hold on
scatter3(target_new(:,2),target_new(:,3),target_new(:,1),'b','filled');
hold on
scatter3(vesselsOnSurf(:,2),vesselsOnSurf(:,3),vesselsOnSurf(:,1),0.1,'r');

scatter3(0,0,0,40,'xr');
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse');
hold off
saveas(gcf,strcat(filename_image,'/Estimated_surface_',date,'.fig'));
Device.VLS(15);
Device.SST('x y z',[0.1,0.1,0.1]);
target_done = zeros(size(target_new));

% Specify angles from normal vectors
range = [[min(surf_L_rmd(:,2)),max(surf_L_rmd(:,2))];[min(surf_L_rmd(:,3)),max(surf_L_rmd(:,3))]];
[Y_m,Z_m] = meshgrid(range(1,1):0.2:range(1,2),range(2,1):0.2:range(2,2));
X_m = F(Y_m,Z_m);
points = pointCloud([reshape(X_m,[],1),reshape(Y_m,[],1),reshape(Z_m,[],1)]);
normals = pcnormals(points);
F_nx = scatteredInterpolant(reshape(Y_m,[],1),reshape(Z_m,[],1),normals(:,1));
F_ny = scatteredInterpolant(reshape(Y_m,[],1),reshape(Z_m,[],1),normals(:,2));
F_nz = scatteredInterpolant(reshape(Y_m,[],1),reshape(Z_m,[],1),normals(:,3));
targetNorm = [F_nx(target_new(:,2),target_new(:,3)),...
    F_ny(target_new(:,2),target_new(:,3))...
    F_nz(target_new(:,2),target_new(:,3))];
figure;
scatter3(target_new(:,2),target_new(:,3),target_new(:,1),25,1:length(target_new(:,2)),'filled');
hold on
quiver3(target_new(1:4:end,2),target_new(1:4:end,3),target_new(1:4:end,1),...
    targetNorm(1:4:end,2),targetNorm(1:4:end,3),targetNorm(1:4:end,1))
axis equal
hold on
scatter3(points.Location(:,2),points.Location(:,3),points.Location(:,1),5,'filled');
ax = gca;
ax.ZDir = 'reverse';

target_v = nan(length(target_new(:,1)),1);
target_w = nan(length(target_new(:,1)),1);
Device.KEN('BREGMA');
Device.NLM('v w',[-10,-20]);
Device.PLM('v w',[2,20]);
for i = 1:length(target_new(:,1))
    [target_v(i),target_w(i)] = FindAllowedDeg(targetNorm(i,:),Device,target_new(i,:));
end
%% Injection !!Change NANOJECT to INJECT mode!! 
% Execution step
% In this step, the parameters for the needle insertion are adjusted.
% The parameters include the speed and depth of insertion, which are defined through Nanoject III.
% The time required to change the position and angle of the injection is only about 20 seconds.

%%%%%Parameters for injection%%%%%%%
vol = 30;
rate = 5;
delay = 5;
repeat = 1;
startId = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
injtime = ceil(vol/rate);
endId = size(target_new,1);
answer = questdlg('Change NANOJECT to INJECT mode', ...
	'Caution', ...
	'Yes','No','No');
% Handle response
switch answer
    case 'Yes'
        disp('Start Injection');
    case 'No'
        disp('Please restart');
        return
end
preview(vid);
Device.VLS(20);
Device.KEN('BREGMA');
Device.MOV('x v w',[-3,0,0])
Device.WAIT();

fig = uifigure;
d = uiprogressdlg(fig,'Title','Injection Progresses...',...
        'Message','1','Cancelable','on');
drawnow
t = 0;
for i=startId:endId
    d.Value = i/endId;
    if i == startId
        d.Message = sprintf(strcat('Cycle : ',num2str(i),'. Estimated time : ?'));
    else
        t = toc;
        d.Message = sprintf(strcat('Cycle : ',num2str(i),'. Estimated time : ',datestr(datetime('now') + seconds(t*(endId-i)))));
    end
    tic
    % Check for Cancel button press
    if d.CancelRequested
        break
    end
    %slack_example(strcat('Cycle : ',num2str(i),'/',num2str(endId),'. Estimated time : ',datestr(datetime('now') + seconds(t*(endId-i)))),'channel','#log');
    current_target_pos = target_new(i,:);
    jv = 1;
    j = 0;
    while jv == 1 % Loop to confirm that the place to be punctured is not a blood vessel
        if j > 3
            %slack_example(strcat('Bleeding! <!channel>',datestr(datetime('now'))),'channel','#log');
            answer = questdlg('Move the needle manually', ...
                'Caution', ...
                'Done','No','No');
            % Handle response
            switch answer
                case 'Done'
                    disp('Resume the process');
                    current_target_pos(1,1) = Device.qPOS('x');
                    current_target_pos(1,3) = Device.qPOS('z');
                case 'No'
                    disp('Stop the process');
                    disp(strcat('Current Inj site : ',num2str(i)));
                    return
            end
        end
        Device.MOV('x y z',[current_target_pos(1,1)-1,current_target_pos(1,2),current_target_pos(1,3)]);
        % AAV Injection
        Device.WAIT()
        Device.VLS(20)
        Device.MOV('x',current_target_pos(1,1)-0.2)
        Device.WAIT()
        % Roughly judge whether current position is on vessels or not.
        start(vid);
        %stoppreview(vid);
        vidimage = getsnapshot(vid);
        %preview(vid)
        needlepoints = imcrop(vidimage,[bregma(1)-4,bregma(2)+10-4,7,7]);
        jv = TC.predictFcn(table(reshape(needlepoints(:,:,1),[],1),reshape(needlepoints(:,:,2),[],1),reshape(needlepoints(:,:,3),[],1)));
        jv = mean(jv) >= 0.5; % If more than half of the 8*8 pixels at the
        %needle tip are recognized as blood vessels: true
        %jv = 0;
        if jv == 1 
            xminus = imcrop(vidimage,[bregma(1)+30-4,bregma(2)+15-4,7,7]);
            xplus = imcrop(vidimage,[bregma(1)-30-4,bregma(2)+15-4,7,7]);
            zminus = imcrop(vidimage,[bregma(1)-4,bregma(2)+15+30-4,7,7]);
            jvxm = TC.predictFcn(table(reshape(xminus(:,:,1),[],1),reshape(xminus(:,:,2),[],1),reshape(xminus(:,:,3),[],1)));
            jvxp = TC.predictFcn(table(reshape(xplus(:,:,1),[],1),reshape(xplus(:,:,2),[],1),reshape(xplus(:,:,3),[],1)));
            jvzm = TC.predictFcn(table(reshape(zminus(:,:,1),[],1),reshape(zminus(:,:,2),[],1),reshape(zminus(:,:,3),[],1)));
            jvxm = mean(jvxm) >= 0.5;
            jvxp = mean(jvxp) >= 0.5;
            jvzm = mean(jvzm) >= 0.5;
            if jvxm == true && jvxp == true && jvzm == true
                Device.VLS(8)
                Device.WAIT()
                %slack_example(strcat('Bleeding! <!channel>',datestr(datetime('now'))),'channel','#log');
                answer = questdlg('Move the needle manually', ...
                'Caution', ...
                'Done','No','No');
                % Handle response
                switch answer
                    case 'Done'
                        disp('Resume the process');
                        current_target_pos(1,2) = Device.qPOS('y');
                        current_target_pos(1,3) = Device.qPOS('z');
                        current_target_pos(1,1) = F(current_target_pos(1,2),current_target_pos(1,3));
                    case 'No'
                        disp('Stop the process');
                        disp(strcat('Current Inj site : ',num2str(i)));
                        return
                end
            else
                % 1 : xminus, 2 : xplus, 3 : zminus
                direction = randsample(3,1,true,[~jvxm,~jvxp,~jvzm]./sum([~jvxm,~jvxp,~jvzm]));
                switch direction
                    case 1 % Move -0.1+-δ mm along x axis.
                        current_target_pos(1,2) = current_target_pos(1,2) - 0.1+(rand(1)-0.5)*0.1;
                        current_target_pos(1,3) = current_target_pos(1,3) + (rand(1)-0.5)*0.1;
                    case 2 % Move 0.1+-δ mm along x axis.
                        current_target_pos(1,2) = current_target_pos(1,2) + 0.1+(rand(1)-0.5)*0.1;
                        current_target_pos(1,3) = current_target_pos(1,3) + (rand(1)-0.5)*0.01;
                    case 3 % Move -0.1+-δ mm along z axis.
                        current_target_pos(1,3) = current_target_pos(1,3) - 0.1+(rand(1)-0.5)*0.1;
                        current_target_pos(1,2) = current_target_pos(1,2) + (rand(1)-0.5)*0.1;
                end
                current_target_pos(1,1) = F(current_target_pos(1,2),current_target_pos(1,3));
            end
        Device.VLS(20)
        Device.WAIT()
        Device.MOV('x', (current_target_pos(1,1) - 1));
        Device.WAIT()
        j = j + 1;
        clear vidimage;
        end
    end
    Device.MOV('x', (current_target_pos(1,1) - 0.2));
    Device.WAIT()
    % Check for Cancel button press
    if d.CancelRequested
        break
    end
    Device.VLS(5);
    Device.MVR('x y z v w',[-Xerrorfit(target_v(i),target_w(i)),-Yerrorfit(target_v(i),target_w(i)),...
        -Zerrorfit(target_v(i),target_w(i)),target_v(i),target_w(i)]);
    Device.WAIT()
    Device.MVR('x',0.2);
    Device.WAIT()
    Device.KSF('INJ');
    Device.KEN('INJ');
    
    % Injection
    Device.MOV('x',-0.3);
    Device.WAIT();
    Device.VLS(6);
    
    Device.MOV('x',0.85);
    Device.WAIT();
    Device.VLS(0.1)
    pause(2)
    Device.MOV('x',injdepth);
    Device.WAIT();
    pause(15)
    for j = 1:repeat
        Device.DIO(1,1);
        pause(injtime)
        Device.DIO(1,0);
        pause(delay)
    end
    % END

    % Marmo Injection
%     Device.VLS(0.1)
%     Device.MOV('x',0.7);
%     Device.WAIT();
%     Device.MOV('x',injdepth);
%     Device.WAIT();
%     pause(2.5)
%     for j = 1:repeat
%         Device.DIO(1,1);
%         pause(injtime)
%         Device.DIO(1,0);
%         pause(delay)
%     end
    % END
        
    Device.MOV('x',-0.2);
    Device.WAIT()
    Device.VLS(20)
    Device.MOV('x',-1);
    Device.WAIT()
    Device.KEN('BREGMA');
    Device.MOV('x v w',[-4,0,0]);
    Device.WAIT()
    % Check for Cancel button press
    if d.CancelRequested
        break
    end
end
close(d)
disp(strcat('Current cycle : ',num2str(i),' / ',num2str(endId)));
%% Save the data
if exist(strcat('history/',date),'file') ~= 7
    filename_history = strcat('history/',date);
    mkdir(filename_history);
    addpath(filename_history);
else
    i = 2;
    while exist(strcat('history/',date,'_',num2str(i)),'file') == 7
        i = i+1;
    end
    filename_history = strcat('history/',date,'_',num2str(i));
    mkdir(filename_history);
    addpath(filename_history);
end

history= struct();
history.x_fig = x_fig;
history.target = target;
history.target_new = target_new;
history.targetonfig = targetonfig;
history.standard = standard_N;
history.delay = delay;
history.repeat = repeat;
history.vol = vol;
history.rate = rate;
history.distlimit = distlimit;
history.targetlength = targetlen;
save(strcat(filename_history,'/history.mat'),'history');% save history
save(strcat(filename_history,'/workspace.mat'));% save workspace
save(strcat(filename_history,'/imgs.mat'),'imgs','-v7.3');% save history


%% Functions
function getroiposEvent(src,evt)
global targetonfig_new
evname = evt.EventName;
    switch(evname)
        case{'ROIMoved'}
            disp(['ROI moved Previous Position: ' mat2str(evt.PreviousPosition)]);
            disp(['ROI moved Current Position: ' mat2str(evt.CurrentPosition)]);
            disp(evt.Source)
            targetonfig_new(cast(str2double(evt.Source.Label),'uint8'),:) = evt.CurrentPosition;
    end
end

function getroiposEvent_boundary(src,evt)
global boundary
evname = evt.EventName;
    switch(evname)
        case{'ROIMoved'}
            disp(['ROI moved Previous Position: ' mat2str(evt.PreviousPosition)]);
            disp(['ROI moved Current Position: ' mat2str(evt.CurrentPosition)]);
            disp(evt.Source)
            boundary(cast(str2double(evt.Source.Label),'uint8'),:) = evt.CurrentPosition;
    end
end

