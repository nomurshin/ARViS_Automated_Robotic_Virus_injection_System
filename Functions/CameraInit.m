% CameraInit Camera calibration using the robot.
% Camera calibration should be performed when you use cameras for the 
% first time so that proper mapping between the absolute coordinate and
% one in the camera frame.
%% Device and Camera setup
% Execute this section to establish the connection to the device,
% create camera objects, and start capturing video to 
C887 = DeviceInit();
Device = DeviceWrapper(C887);

Device.KEN('zero');
Device.MOV('u v w',[0 0 0]);
Device.WAIT();
Device.VLS(20);

oldNpos = nan(2,1);
newNpos = nan(2,1);

PASSTHRESH = 0.015;

MaxTrialNum = 100;

% Create the webcam object.
cam = videoinput('gentl', 2, 'Mono8');
cam2 = videoinput('gentl',3,'Mono8');
% 高速化
triggerconfig(cam,'manual');
triggerconfig(cam2,'manual')

stop(cam);
start(cam)


% Capture one frame to get its size.
videoFrame = getsnapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
stop(cam2);
start(cam2)
% Capture one frame to get its size.
videoFrame = getsnapshot(cam2);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer2 = vision.VideoPlayer('Position', [100+frameSize(2)+50 100 [frameSize(2), frameSize(1)]+30]);
Device.KEN('zero');
Device.VLS(20);

% Here, move the camera to focus on the needle tip while two cameras get
% images continuously.

runLoop = true;
runLoop2 = true;
numPts = 0;
frameCount = 0;
while runLoop && runLoop2 
    % Get the next frame.
    videoFrame = getsnapshot(cam);
    videoFrame2 = getsnapshot(cam2);
    frameCount = frameCount + 1;
    
    [meanPoints, videoFrame] = updateNpos(videoFrame);
    [~,videoFrame2] = updateNpos(videoFrame2);
    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);
    step(videoPlayer2, videoFrame2);
    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
    runLoop2 = isOpen(videoPlayer2);
end
if runLoop == false
    videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
    step(videoPlayer, videoFrame);
end
stop(cam2);
if runLoop2 == false
    videoPlayer2 = vision.VideoPlayer('Position', [100+frameSize(2)+50 100 [frameSize(2), frameSize(1)]+30]);
    step(videoPlayer2, videoFrame2);
end
Opos = Device.qPOS('x y z');
stop(cam);
%% Move needle as grid pattern for camera calibration
% Change "cam" variable to "cam2" if you want to calibrate the other
% camera.
cam_for_calb = cam;

Device.KEN('zero')
Device.MOV('x y z u v w',[Opos;0;0;0]);
Device.WAIT();
[gridX,gridY] = meshgrid(0:0.2:1.4,0:0.2:1.4);
gridX = reshape(gridX,[],1);gridY = reshape(gridY,[],1);
[imgridX,imgridY] = meshgrid(-1.6:0.2:0.6,-2:0.2:0.8);
imgridX = reshape(imgridX,[],1);
imgridY = reshape(imgridY,[],1);
imagenum = length(imgridX);
imagePoints = nan(length(gridX),2,imagenum);
start(cam_for_calb);
for j = 1:imagenum
    if j == 1
        Device.MVR('x y',[imgridX(j),imgridY(j)]);
    else
        Device.MVR('x y',[-imgridX(j-1)+imgridX(j),-imgridY(j-1)+imgridY(j)]);
        Device.WAIT();
        degCad = [zeros(1,2),2*(rand(1,1)-0.5)*30];
        posCad = [(([imgridX(j),imgridY(j),0]+Opos')+[gridX,gridY,zeros(size(gridX))]),repmat(degCad,size(gridX))];
        while qCanMov(posCad,Device) == false
            degCad = [zeros(1,2),2*(rand(1,1)-0.5)*30];
            posCad = [(([imgridX(j),imgridY(j),0]+Opos')+[gridX,gridY,zeros(size(gridX))]),repmat(degCad,size(gridX))];
            disp(1);
        end
        Device.MOV('u v w',degCad)
        Device.WAIT();
        Device.KSF('GRID');
        Device.KEN('GRID');
    end
    Device.WAIT();
    for i = 1:length(gridX)
        if i == 1
            Device.MVR('x y',[gridX(i),gridY(i)]);
        else
            Device.MVR('x y',[-gridX(i-1)+gridX(i),-gridY(i-1)+gridY(i)]);
        end
        Device.WAIT();
        videoFrame = getsnapshot(cam_for_calb);
        [newNpos, videoFrame] = updateNpos(videoFrame);
        step(videoPlayer,videoFrame);
        imagePoints(i,:,j) = newNpos;
        if i == length(gridX)
            Device.MVR('x y',[-gridX(i),-gridY(i)]);
            Device.WAIT();
        end
    end
    Device.KEN('zero');
    Device.MOV('z u v w',[Opos(3),0,0,0]);
    Device.WAIT();

end
%
worldPoints = [gridX,gridY];
calib_struct = struct();
calib_struct.imagePoints = imagePoints;
calib_struct.worldPoints = worldPoints;

params = estimateCameraParameters(imagePoints,worldPoints,"NumRadialDistortionCoefficients",2, ...
                                  'WorldUnits','mm','ImageSize',size(videoFrame),'EstimateSkew',false,'EstimateTangentialDistortion',false);
calib_struct.params = params;
save(strcat(date,'_params.mat'),'calib_struct');
figure;
showReprojectionErrors(params);
figure;
showExtrinsics(params);

function flag = qCanMov(candpos,device)
    flag = true;
    for i = 1:length(candpos(:,1))
        flag = flag && device.qVMO({'x','y','z','u','v','w'},candpos(i,:));
    end
end


function [meanPoints, videoFrame] = updateNpos(videoFrame)
    videoFrame = imadjust(imtophat(videoFrame,strel('square',100)),[0.01,0.1]);
    y = find(max(videoFrame>50,[],2),5,'last');
    [~,x] = max(videoFrame(y,:),[],2,'omitnan');
    zyPoints = [x,y];
    meanPoints = [mean(y);mean(x)];
    videoFrame = insertMarker(videoFrame, zyPoints, 'o', 'Color', 'white','size',20);
    videoFrame = rgb2gray(videoFrame);
end