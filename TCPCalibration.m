%% Set up for cameras: Just execute this section.

%%%%%%%% Create the webcam object %%%%%%%%
% Please check if the parameters are correct
% for the camera in your environment.

cam = videoinput('gentl', 2, 'Mono8');
cam2 = videoinput('gentl',3,'Mono8');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

triggerconfig(cam,'manual');
triggerconfig(cam2,'manual')

Device.KEN('zero');
Device.MOV('u v w',[0 0 0]);
Device.WAIT();
Device.VLS(20);

%
currentSPI = Device.qSPI('r s t');
historySPI = currentSPI;
movlist = NaN(10,6); % x y z u v w
MaxFrameNum = zeros(10,1);
MaxFrameNum(1) = 400;
MaxFrameNum(2) = 2;
MaxFrameNum(3) = 10;
oldNpos = nan(2,1);
newNpos = nan(2,1);
save_data = struct();

PASSTHRESH = 0.015;

movlist(1,:) = [0,0,0,0.01,0,0];
MaxTrialNum = 100;
umovschedule = zeros(4,1);
wmovschedule = zeros(4,1);
a = linspace(1,5,2);
b = [diff(a)/2,1];
for i = 1:2
    umovschedule((i-1)*2+1) = a(i);
    umovschedule((i-1)*2+2) = -a(i)-b(i);
end

a = linspace(5,20,3);
b = [diff(a)/2,1];
for i = 1:2
    wmovschedule((i-1)*2+1) = a(i);
    wmovschedule((i-1)*2+2) = -a(i)-b(i);
end

umovhis = nan(MaxTrialNum,1);
wmovhis = nan(MaxTrialNum,1);

framelen = zeros(2,1); % 1mm distance on the image (px)
absNN = nan(400,1);
vecNN = nan(400,2);% (y z)
vecNP = nan(400,2);% (y z)


%% Focus manually here while capturing the videos in while loop
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
totalframe = 0;
camTheta1 = nan(10,1);
camTheta2 = nan(10,2);


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
%% Camera calibration
% Estimate the rotation and translation between the two cameras so that 
% we can Estimate camera coordinates {C} from the pixel coordinates of two images.

addpath(genpath('Data/videoCalibration'));
cameraParams1 = load("Data/videoCalibration/cameraparam1.mat").params;
cameraParams2 = load("Data/videoCalibration/cameraparam2.mat").params;
runLoop = true;
stop(cam2);
Device.KEN('zero')
frameCount = 0;
Device.MOV('x y z u v w',[Opos;0;0;0]);
Device.WAIT()
EvecNN = nan(10,2);
EvecNN2 = nan(10,2);
MovLog = nan(10,3);
start(cam);start(cam2);
while frameCount < 200
    if frameCount == 0
        Device.MOV('x y z',Opos);
    elseif frameCount < 70
        Device.MOV('x y z',Opos + 0.4*(rand(3,1)-0.5));
    elseif frameCount < 120
        Device.MOV('x y z',Opos + 0.8*(rand(3,1)-0.5));
    else
        Device.MOV('x y z',Opos + 1.6*(rand(3,1)-0.5));
    end
    Device.WAIT();
    % Get the next frame.
    videoFrame2 = getsnapshot(cam2);
    videoFrame2 = undistortImage(videoFrame2,cameraParams2);
    videoFrame = getsnapshot(cam);
    videoFrame = undistortImage(videoFrame,cameraParams1);
    frameCount = frameCount + 1;
    NewHexaPos = Device.qPOS('x y z u v w');
    MovLog(frameCount,:) = NewHexaPos(1:3)';
    [newNpos2, videoFrame2] = updateNpos(videoFrame2);
    step(videoPlayer2, videoFrame2);
    [newNpos, videoFrame] = updateNpos(videoFrame);
    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);
    EvecNN(frameCount,:) = newNpos;
    EvecNN2(frameCount,:) = newNpos2;% (y,x)
end
stop(cam);
if runLoop == false
    videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
end
if runLoop2 == false
    videoPlayer2 = vision.VideoPlayer('Position', [100+frameSize(2)+50 100 [frameSize(2), frameSize(1)]+30]);
    step(videoPlayer2, videoFrame2);
end
%
stop(cam2);
%
fMatrix = estimateEssentialMatrix(EvecNN, EvecNN2, cameraParams1,cameraParams2,'MaxDistance',1);

[R, t] = relativeCameraPose(fMatrix, cameraParams1,cameraParams2, EvecNN, EvecNN2);

camMatrix1 = cameraMatrix(cameraParams1, eye(3), [0 0 0]);
camMatrix2 = cameraMatrix(cameraParams2, R', -t*R');
%Estimate visual coordinates {V} from the pixel coordinates of two images.
points3D = triangulate(EvecNN, EvecNN2, camMatrix1, camMatrix2);

sampleSize = 9;
maxDistance = 0.015;
fitRBVFnc = @(id) MakeRBVMat(points3D,MovLog,id);
evalRBVFnc = @(model,id) culcRBVDist(points3D,MovLog,id,model);
id = (1:size(points3D,1))';
[RBV,inlierIdx] = ransac(id,fitRBVFnc,evalRBVFnc,sampleSize,maxDistance,'MaxSamplingAttempts',200,'MaxNumTrials',50000);
recontPnt = (points3D-points3D(1,:))*RBV+MovLog(1,:);
dist = vecnorm(recontPnt - MovLog,2,2);
figure;
histogram(dist);
figure;
scatter3(recontPnt(inlierIdx,1),recontPnt(inlierIdx,2),recontPnt(inlierIdx,3));
hold on
scatter3(recontPnt(~inlierIdx,1),recontPnt(~inlierIdx,2),recontPnt(~inlierIdx,3));
hold on
scatter3(MovLog(:,1),MovLog(:,2),MovLog(:,3),'filled');
figure;
scatter3(MovLog(2:end,1),MovLog(2:end,2),MovLog(2:end,3),25,100*vecnorm(recontPnt(2:end,:) - MovLog(2:end,:),2,2)./vecnorm(MovLog(2:end,:)-MovLog(1,:),2,2),'filled');
ONpoints = points3D(1,:);
Device.MOV('x y z',Opos);
Device.WAIT()
%% 
EvecNN = nan(10,2);
EvecNN2 = nan(10,2);
MovLog = nan(10,3);
start(cam);start(cam2);
frameCount = 0;
while frameCount < 60
    Device.MOV('x y z',Opos + 1*(rand(3,1)-0.5));
    Device.WAIT();
    % Get the next frame.
    videoFrame2 = getsnapshot(cam2);
    videoFrame2 = undistortImage(videoFrame2,cameraParams2);
    videoFrame = getsnapshot(cam);
    videoFrame = undistortImage(videoFrame,cameraParams1);
    frameCount = frameCount + 1;
    NewHexaPos = Device.qPOS('x y z u v w');
    MovLog(frameCount,:) = NewHexaPos(1:3)';
    [newNpos2, videoFrame2] = updateNpos(videoFrame2);
    step(videoPlayer2, videoFrame2);
    [newNpos, videoFrame] = updateNpos(videoFrame);
    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);
    EvecNN(frameCount,:) = newNpos;
    EvecNN2(frameCount,:) = newNpos2;% (y,x)
end
if runLoop == false
    videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
end
if runLoop2 == false
    videoPlayer2 = vision.VideoPlayer('Position', [100+frameSize(2)+50 100 [frameSize(2), frameSize(1)]+30]);
    step(videoPlayer2, videoFrame2);
end
points3D = triangulate(EvecNN, EvecNN2, camMatrix1, camMatrix2);
%
stop(cam)
recontPnt = (points3D-ONpoints)*RBV+Opos';
figure;
histogram(vecnorm(recontPnt-MovLog,2,2));
figure;
scatter3(recontPnt(:,1),recontPnt(:,2),recontPnt(:,3));
hold on
scatter3(MovLog(:,1),MovLog(:,2),MovLog(:,3),'filled');
stop(cam2);
%% TCP calibration
Device.MOV('x y z u v w',[Opos;0;0;0]);
Device.WAIT();
spischedule = 2*(rand(10,3)-0.5).*[20,20,20] + [0,0,200];
range = [2,2];
cyclenum = size(spischedule,1);
rhis = nan(cyclenum,1);
PBNeedle = nan(3,100,cyclenum);
DegLog_log = nan(cyclenum,40,3);
recontPnt_log = nan(cyclenum,40,3);
SPI_log = nan(cyclenum,40,3);
tic
for j = 1:cyclenum
    stop(cam);
    EvecNN = nan(40,2);
    EvecNN2 = nan(40,2);
    MovLog = nan(40,3);
    DegLog = nan(40,3);
    stop(cam2);
    Device.MOV('x y z u v w',[Opos;0;0;0]);
    Device.WAIT();
    Device.SPI('r s t',spischedule(j,:));
    start(cam);start(cam2);
    frameCount = 0;
    while frameCount < 40
        nextpos = Opos;
        nextdeg = [0;(rand(1,1)-0.5)*range(1);(rand(1,1)-0.5)*range(2)];
        if Device.qVMO({'x','y','z','u','v','w'},[nextpos;nextdeg])
            Device.MOV('x y z u v w',[nextpos;nextdeg]);
            Device.WAIT();
            % Get the next frame.
            videoFrame2 = getsnapshot(cam2);
            videoFrame2 = undistortImage(videoFrame2,cameraParams2);
            videoFrame = getsnapshot(cam);
            videoFrame = undistortImage(videoFrame,cameraParams1);
            frameCount = frameCount + 1;
            NewHexaPos = Device.qPOS('x y z u v w');
            MovLog(frameCount,:) = NewHexaPos(1:3)';
            DegLog(frameCount,:) = NewHexaPos(4:6)';
            [newNpos2, videoFrame2] = updateNpos(videoFrame2);
            step(videoPlayer2, videoFrame2);
            [newNpos, videoFrame] = updateNpos(videoFrame);
            % Display the annotated video frame using the video player object.
            step(videoPlayer, videoFrame);
            EvecNN(frameCount,:) = newNpos;
            EvecNN2(frameCount,:) = newNpos2;% (y,x)
        end
    end
    if runLoop == false
        videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
    end
    if runLoop2 == false
        videoPlayer2 = vision.VideoPlayer('Position', [100+frameSize(2)+50 100 [frameSize(2), frameSize(1)]+30]);
        step(videoPlayer2, videoFrame2);
    end
    points3D = triangulate(EvecNN, EvecNN2, camMatrix1, camMatrix2);
    recontPnt = (points3D-ONpoints)*RBV;%
    recontPnt_log(j,:,:) = recontPnt;
    DegLog_log(j,:,:) = DegLog;
    SPI_log(j,:,:) = repmat(reshape(spischedule(j,:),1,1,[]),[1,length(DegLog(:,1)),1]);
    % SVD Method
    Device.MOV('x y z u v w',[Opos;0;0;0]);
    Device.WAIT();
    RENeedle_cand = nan(3,100);
    for jj = 1:100
        RM = [];
        recontPnt_v = [];
        for i = randsample(1:length(DegLog(:,1)),round(length(DegLog(:,1))*0.8))
            RM = [RM;MakeRotMat(DegLog(i,:))-MakeRotMat([0,0,0])];
            recontPnt_v = [recontPnt_v;recontPnt(i,:)'];
        end
        RENeedle_cand(:,jj) = lsqminnorm(RM,recontPnt_v);
    end
    PBNeedle(:,:,j) = RENeedle_cand + spischedule(j,:)';
end
toc
currentSPI = mean(PBNeedle,[2,3]);
figure;
for i = 1:cyclenum
    scatter3(PBNeedle(1,:,i),PBNeedle(2,:,i),PBNeedle(3,:,i),5,'filled');
    hold on
end
scatter3(currentSPI(1),currentSPI(2),currentSPI(3),50,"*");
hold off
axis equal
Device.SPI('r s t',currentSPI)

% 3D Grid Search to find the point where the needle tip shift is minimized.
stop(cam2)
srange = 0.35;
numgrid = 4;
stop(cam);
cyclenum = ceil(log2(0.03/srange)/log2(2/3));%Set the number of times for the needle shift to be less than 0.07 mm
for k = 1:cyclenum
    tic
    maxlim = srange;
    minlim = -srange;
    
    srange = (maxlim-minlim)./(numgrid-1);
    sYveclog = [];
    sZveclog = [];
    sXveclog = [];
    errorVecPlog = [];
    errorVecNlog = [];
    
    [sX,sY,sZ] = meshgrid(linspace(minlim,maxlim,numgrid),linspace(minlim,maxlim,numgrid),linspace(minlim,maxlim,numgrid));
    sYvec = squeeze(reshape(sY,[],1));
    sZvec = squeeze(reshape(sZ,[],1));
    sXvec = squeeze(reshape(sX,[],1));
    X = sXvec+currentSPI(1);
    Y = sYvec+currentSPI(2);
    Z = sZvec + currentSPI(3);
    sYveclog = [sYveclog+historySPI(2,1);Y];
    sZveclog = [sZveclog+historySPI(3,1);Z];
    sXveclog = [sXveclog+historySPI(1,1);X];
    % grid search
    errorVecPU = nan(length(sZvec),2);
    errorVecNU = nan(length(sZvec),2);
    errorVecP2U = nan(length(sZvec),2);
    errorVecN2U = nan(length(sZvec),2);
    errorVecPW = nan(length(sZvec),2);
    errorVecNW = nan(length(sZvec),2);
    errorVecP2W = nan(length(sZvec),2);
    errorVecN2W = nan(length(sZvec),2);
    Device.KEN('zero');
    % Moving to the origin
    Device.MOV('u v w',[0,0,0]);
    Device.WAIT();
    % Get the next frame.
    start(cam)
    start(cam2)
    videoFrame = getsnapshot(cam);
    videoFrame = undistortImage(videoFrame,cameraParams1);
    NewHexaPos = Device.qPOS('x y z u v w');
    [OriNpos, videoFrame] = updateNpos(videoFrame);
    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);
    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);

    for i = 1:length(X)
        Device.MOV('v w',[0,0]);
        Device.WAIT();
        videoFrame = getsnapshot(cam);
        videoFrame = undistortImage(videoFrame,cameraParams1);
        [OriNpos, videoFrame] = updateNpos(videoFrame);
        % Display the annotated video frame using the video player object.
        step(videoPlayer, videoFrame);
        videoFrame2 = getsnapshot(cam2);
        videoFrame2 = undistortImage(videoFrame2,cameraParams2);
        [OriNpos2, videoFrame2] = updateNpos(videoFrame2);
        % Display the annotated video frame using the video player object.
        step(videoPlayer2, videoFrame2);
        Device.SPI('r s t',[X(i),Y(i),Z(i)]);


        Device.MOV('v',-7);
        Device.WAIT();
        % Get the next frame.
        videoFrame = getsnapshot(cam);
        videoFrame = undistortImage(videoFrame,cameraParams1);
        [errorVecNU(i,:), videoFrame] = updateNpos(videoFrame);
        step(videoPlayer, videoFrame);
        errorVecNU(i,:) = errorVecNU(i,:);
        % Get the next frame.
        videoFrame2 = getsnapshot(cam2);
        videoFrame2 = undistortImage(videoFrame2,cameraParams2);
        [errorVecN2U(i,:), videoFrame2] = updateNpos(videoFrame2);
        step(videoPlayer2, videoFrame2);
        errorVecN2U(i,:) = errorVecN2U(i,:);
      
        Device.MOV('v w',[0,-15]);
        Device.WAIT();
        % Get the next frame.
        videoFrame = getsnapshot(cam);
        videoFrame = undistortImage(videoFrame,cameraParams1);
        [errorVecNW(i,:), videoFrame] = updateNpos(videoFrame);
        step(videoPlayer, videoFrame);
        errorVecNW(i,:) = errorVecNW(i,:);
        % Get the next frame.
        videoFrame2 = getsnapshot(cam2);
        videoFrame2 = undistortImage(videoFrame2,cameraParams2);
        [errorVecN2W(i,:), videoFrame2] = updateNpos(videoFrame2);
        step(videoPlayer2, videoFrame2);
        errorVecN2W(i,:) = errorVecN2W(i,:);

    end

    Device.MOV('u v w',[0,0,0]);
    Device.WAIT();
    runLoop = isOpen(videoPlayer);
    stop(cam)

    errorVecN3DU = (triangulate(errorVecNU, errorVecN2U, camMatrix1, camMatrix2)-triangulate(OriNpos', OriNpos2', camMatrix1, camMatrix2))*RBV;
    errorVecN3DW = (triangulate(errorVecNW, errorVecN2W, camMatrix1, camMatrix2)-triangulate(OriNpos', OriNpos2', camMatrix1, camMatrix2))*RBV;
    stop(cam2)
%     
    figure;
    objectE = vecnorm(errorVecN3DU,2,2)...
        +vecnorm(errorVecN3DW,2,2);
    scatter3(X,Y,Z,25,objectE,'filled');
    title(strcat('range : ',num2str(srange)));

    [~,minId] = min(objectE);
    
    minSPI = [X(minId),Y(minId),Z(minId)]';
    Device.SPI('r s t',minSPI);
    currentSPI = Device.qSPI('r s t');
    historySPI = [historySPI,currentSPI];
    t = toc;
end
time1 = toc
save_data.finalSPI = currentSPI;

% Detects and compensates for machine errors
tic
numrep = 40;
meanrep = 2;
stop(cam)
[V,W] = meshgrid(linspace(-14,14,numrep),linspace(-23,23,numrep));
chechmovschedule = [reshape(V,[],1),reshape(W,[],1)];
passedId = false(size(chechmovschedule,1),meanrep);
EvecNN = nan(size(chechmovschedule,1),2,meanrep);
EvecNN2 = nan(size(chechmovschedule,1),2,meanrep);
frameCount = 0;
trialnum = 0;
% u = 0に戻す。
Device.MOV('x y z u v w',[Opos;0;0;0]);
Device.WAIT(); 
stop(cam2)
runLoop = isOpen(videoPlayer);
runLoop2 = isOpen(videoPlayer2);
if runLoop == false
    videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
    step(videoPlayer, videoFrame);
end
if runLoop2 == false
    videoPlayer2 = vision.VideoPlayer('Position', [100+frameSize(2)+50 100 [frameSize(2), frameSize(1)]+30]);
    step(videoPlayer2, videoFrame2);
end
start(cam)
start(cam2)
% Get the next frame.
videoFrame = getsnapshot(cam);
videoFrame2 = getsnapshot(cam2);

frameCount = frameCount + 1;
OriHexaPos = Device.qPOS('x y z u v w');
[OriNpos, videoFrame] = updateNpos(videoFrame);
% Display the annotated video frame using the video player object.
step(videoPlayer, videoFrame);
[OriNpos2, videoFrame2] = updateNpos(videoFrame2);
% Display the annotated video frame using the video player object.
step(videoPlayer2, videoFrame2);
% Check whether the video player window has been closed.
runLoop = isOpen(videoPlayer);
currentSPI = Device.qSPI('r s t');
for ii = 1:meanrep
    if ii >= 2
        passedId(:,ii) = chechmovschedule(:,1)<2 | abs(chechmovschedule(:,2))<5;
    end
    if ii >= 3
        passedId(:,ii) = chechmovschedule(:,1)<4 | abs(chechmovschedule(:,2))<10;
    end


Device.MOV('x y z u v w',[Opos;0;0;0]);
Device.WAIT(); 
for i = find(~passedId(:,ii))'
    trialnum = trialnum + 1;
    % + 方向の移動
    if Device.qVMO({'v','w'},[chechmovschedule(i,1), chechmovschedule(i,2)]) == 1
        Device.MOV('v w',[chechmovschedule(i,1), chechmovschedule(i,2)]);
        Device.WAIT();
        % Get the next frame.
        videoFrame = getsnapshot(cam);
        videoFrame = undistortImage(videoFrame,cameraParams1);
        videoFrame2 = getsnapshot(cam2);
        videoFrame2 = undistortImage(videoFrame2,cameraParams2);
        frameCount = frameCount + 1;
        %idx = idx + 1;
        NewHexaPos = Device.qPOS('x y z u v w');
        [newNpos, videoFrame] = updateNpos(videoFrame);
        step(videoPlayer, videoFrame);
        [newNpos2, videoFrame2] = updateNpos(videoFrame2);
        step(videoPlayer2, videoFrame2);
        EvecNN(i,:,ii) = newNpos;
        EvecNN2(i,:,ii) = newNpos2;
    else
        disp('passed');
        passedId(i,ii) = true;
    end
    runLoop = isOpen(videoPlayer);
    disp(i)
end
end
stop(cam);
%
EvecNN_m = mean(EvecNN,3,'omitnan');
EvecNN2_m = mean(EvecNN2,3,'omitnan');
nanflag_e = isnan(mean(EvecNN_m,2));
nanflag_e2 = isnan(mean(EvecNN2_m,2));
nanflag = nanflag_e | nanflag_e2;
error3D = (triangulate(EvecNN_m(~nanflag,:), EvecNN2_m(~nanflag,:), camMatrix1, camMatrix2)...
    -triangulate(OriNpos', OriNpos2', camMatrix1, camMatrix2))*RBV;

YdegHisV = chechmovschedule(~nanflag,1);
YdegHisW = chechmovschedule(~nanflag,2);
Xerror = error3D(:,1);
Yerror = error3D(:,2);
Zerror = error3D(:,3);

stop(cam2);
[Yerrorfit gof] = createFits_error(YdegHisV, YdegHisW, Yerror);
xlabel('V axis');
ylabel('W axis');
zlabel('Y error (mm)');
legend('data')
saveas(gcf,strcat('Y_error',date,'.fig'));
[Zerrorfit gof] = createFits_error(YdegHisV, YdegHisW,Zerror);
xlabel('V axis');
ylabel('W axis');
zlabel('Z error (mm)');
legend('data')
saveas(gcf,strcat('Z_error',date,'.fig'));
[Xerrorfit gof] = createFits_error(YdegHisV, YdegHisW,Xerror);
xlabel('V axis');
ylabel('W axis');
zlabel('X error (mm)');
legend('data')
saveas(gcf,strcat('X_error',date,'.fig'));

k = convhull(YdegHisV,YdegHisW);
convpoints = [YdegHisV(k),YdegHisW(k)];%Surveyed angle range specification

time2 = toc
%
save_data.Xerrorfit = Xerrorfit;
save_data.Yerrorfit = Yerrorfit;
save_data.Zerrorfit = Zerrorfit;
save_data.boundary = convpoints;
save(strcat('images/',date,'/',date,'save_params.mat'),'save_data');

% Accuracy verification With error correction
thredFlag = false;
Dy = [0,0]; % （+方向　ー方向）
Du = [0,0]; % （+方向　ー方向）
Dz = [0,0];
YLossHis = nan(10,1);
stop(cam)
Device.NLM('v w',[-10,-15]);
Device.PLM('v w',[4,20]);

numrep = 150;
chechmovschedule = [(rand(1,numrep)-0.5)*2*10;(rand(1,numrep)-0.5)*2*20]';%[v w]

[in,on] = inpolygon(chechmovschedule(:,1),chechmovschedule(:,2),convpoints(:,1)*0.95,convpoints(:,2)*0.95);
chechmovschedule = chechmovschedule(in,:);
passedId = false(size(chechmovschedule,1),1);
EvecNN_c = nan(size(chechmovschedule,1),2);
EvecNN2_c = nan(size(chechmovschedule,1),2);
frameCount = 0;
stop(cam2)
idx = 0;
trialnum = 0;
if runLoop == false
    videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
    step(videoPlayer, videoFrame);
end
if runLoop2 == false
    videoPlayer2 = vision.VideoPlayer('Position', [100+frameSize(2)+50 100 [frameSize(2), frameSize(1)]+30]);
    step(videoPlayer2, videoFrame2);
end
start(cam)
runLoop = true;
runLoop2 = true;
Device.MOV('x y z u v w',[Opos;0;0;0]);
Device.WAIT(); 
start(cam2)
for i = 1:size(chechmovschedule,1)
    trialnum = trialnum + 1;
    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
    runLoop2 = isOpen(videoPlayer2);

    if Device.qVMO({'x','y','z','v','w'},[-Xerrorfit(chechmovschedule(i,1),chechmovschedule(i,2))+Opos(1),-Yerrorfit(chechmovschedule(i,1),...
            chechmovschedule(i,2)) + Opos(2),-Zerrorfit(chechmovschedule(i,1),chechmovschedule(i,2))+Opos(3),chechmovschedule(i,1), chechmovschedule(i,2)]) == 1
        Device.MOV('x y z v w',[-Xerrorfit(chechmovschedule(i,1),chechmovschedule(i,2))+Opos(1),-Yerrorfit(chechmovschedule(i,1),...
            chechmovschedule(i,2)) + Opos(2),-Zerrorfit(chechmovschedule(i,1),chechmovschedule(i,2))+Opos(3),chechmovschedule(i,1), chechmovschedule(i,2)]);
        Device.WAIT();
    
        % Get the next frame.
        videoFrame = getsnapshot(cam);
        videoFrame = undistortImage(videoFrame,cameraParams1);
        videoFrame2 = getsnapshot(cam2);
        videoFrame2 = undistortImage(videoFrame2,cameraParams2);
        frameCount = frameCount + 1;
        idx = idx + 1;
        oldNpos = newNpos;
        oldNpos2 = newNpos2;
        OldHexaPos = NewHexaPos;
        NewHexaPos = Device.qPOS('x y z u v w');
        [newNpos, videoFrame] = updateNpos(videoFrame);
        [newNpos2, videoFrame2] = updateNpos(videoFrame2);
        step(videoPlayer, videoFrame);
        step(videoPlayer2, videoFrame2);
        EvecNN_c(idx,:) = newNpos;
        EvecNN2_c(idx,:) = newNpos2;
        runLoop = isOpen(videoPlayer);
        runLoop2 = isOpen(videoPlayer2);
    else
        disp("passed")
        passedId(i) = true;
    end
    disp(i)
end
if runLoop == false
    videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
end
stop(cam)

Device.MOV('x y z u v w',[Opos;0;0;0]);
Device.WAIT(); 
stop(cam2)
error3D_C = (triangulate(EvecNN_c(1:sum(~passedId),:), EvecNN2_c(1:sum(~passedId),:), camMatrix1, camMatrix2)-triangulate(OriNpos', OriNpos2', camMatrix1, camMatrix2))*RBV;
YvecNN = error3D_C(:,2);
ZvecNN = error3D_C(:,3);
XvecNN = error3D_C(:,1);

titlestr = strcat("誤差 X ",num2str(round(Opos(1),2)),"mm, Y ",num2str(round(Opos(2),2)),"mm, Z ",num2str(round(Opos(3),2)),"mm");

figure;
scatter3(chechmovschedule(~passedId,1),chechmovschedule(~passedId,2),YvecNN);
xlabel('U 軸');ylabel('W 軸');zlabel('Y 軸　誤差')
title(strcat("Y軸",titlestr));
saveas(gcf,strcat('images/',date,'/','Y_error_modified',date,'.fig'));
figure;
scatter3(chechmovschedule(~passedId,1),chechmovschedule(~passedId,2),ZvecNN);
xlabel('U 軸');ylabel('W 軸');zlabel('Z 軸　誤差')
title(strcat("Z軸",titlestr));
saveas(gcf,strcat('images/',date,'/','Z_error_modified',date,'.fig'));
figure;
scatter3(chechmovschedule(~passedId,1),chechmovschedule(~passedId,2),XvecNN);
xlabel('U 軸');ylabel('W 軸');zlabel('X 軸　誤差')
title(strcat("X軸",titlestr));
saveas(gcf,strcat('images/',date,'/','X_error_modified',date,'.fig'));
figure;
scatter3(chechmovschedule(~passedId,1),chechmovschedule(~passedId,2),vecnorm(error3D_C,2,2));
xlabel('U 軸');ylabel('W 軸');zlabel('誤差 (mm)')
title(titlestr);
saveas(gcf,strcat('images/',date,'/','Norm_error_modified',date,'.fig'));
error_data = struct();
error_data.error = error3D_C;
error_data.position = Opos;
toc

%% Clean up.
clear cam;
release(videoPlayer);

function RBV = MakeRBVMat(points3D,MovLog,id)
    RBV = lsqminnorm(points3D(id,:)-points3D(1,:),(MovLog(id,:)-MovLog(1,:)),1e-3);
end
function dist = culcRBVDist(points3D,MovLog,id,RBV)
    recontPnt = (points3D(id,:)-points3D(1,:))*RBV+MovLog(1,:);
    dist = vecnorm(recontPnt - MovLog(id,:),2,2);
end

function R = MakeRotMat(DegLog)
    u = deg2rad(DegLog(:,1));
    v = deg2rad(DegLog(:,2));
    w = deg2rad(DegLog(:,3));
    U = [[1,0,0];[0,cos(u),-sin(u)];[0,sin(u),cos(u)]];
    V = [[cos(v),0,sin(v)];[0,1,0];[-sin(v),0,cos(v)]];
    W = [[cos(w),-sin(w),0];[sin(w),cos(w),0];[0,0,1]];
    R = W*V*U;
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