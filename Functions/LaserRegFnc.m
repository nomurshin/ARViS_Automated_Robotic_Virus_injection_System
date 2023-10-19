function centerpos = LaserRegFnc(Device,v2d,Vel)
%LASERREGFNC レーザーをRegistlationする際に、針の中央を自動定位する関数。Z軸を走査したのち、X軸を走査する。
%   Device : Hexapodデバイス
tic
Device.KSF('LaserReg');
Device.KEN('LaserReg');
SR = 20; %Sampling Rate (Hz)
%Vel = 20; % Scan speed (mm/sec)
zwidth = 1;
ywidth = 1;
%stepw = 0.05;
%zsteps = -zwidth:stepw:zwidth;
%xsteps = -xwidth:stepw:xwidth;
Device.VLS(Vel);
Device.RTR(round(10000/SR)); 
centerpos_l = zeros(2,1);
Device.MOV('y z',[0,-zwidth]);
WAIT(Device);

Device.DRC(1,'x',2);
Device.DRC(2,'y',2);
Device.DRC(3,'z',2);
%Device.DRC(4,'1',150);
Device.DRC(4,'1',17);
Device.DRT ( 6, 1, '0' );
Device.DRT ( 6, 2, '0' );
Device.DRT ( 6, 3, '0' );
Device.DRT ( 6, 4, '0' );
% for i = 1:length(zsteps)
%         Device.MOV('z', zsteps(i));
%         WAIT(Device);
%         Device.MOV('x z',[0,zsteps(i)]);
%         WAIT(Device);
% end
Device.MOV('z',zwidth);
WAIT(Device);
laserdx = Device.qDRR(1,1,-1);
tlen = length(laserdx(:,1));
laserdy = Device.qDRR(2,1,tlen);
laserdz = Device.qDRR(3,1,tlen);
laserdd = Device.qDRR(4,1,tlen);
toc

X = laserdx(:,2);
D = v2d(laserdd(:,2));
Z = laserdz(:,2);
Y = laserdy(:,2);

D = D + (X- mean(X));
pos = [D,Y,Z];
[~,I] = sort(Z);
pos = pos(I,:);

minZ = find(pos(:,1)<4.5&pos(:,1)>-3,1,'first');
maxZ = find(pos(:,1)<4.5&pos(:,1)>-3,1,'last');
centerpos_l(2) = mean([pos(minZ,3),pos(maxZ,3)]);

figure;
subplot(1,2,1)
scatter(Z,D,2,'r','filled');
hold on
scatter(pos([minZ;maxZ],3),pos([minZ;maxZ],1),25,'b');
hold on
xline(centerpos_l(2));
Device.MOV('z',centerpos_l(2));
WAIT(Device);
Device.MOV('y z',[-ywidth,0]);
WAIT(Device);

Device.DRC(1,'x',2);
Device.DRC(2,'y',2);
Device.DRC(3,'z',2);
Device.DRC(4,'1',17);
Device.DRT ( 6, 1, '0' );
Device.DRT ( 6, 2, '0' );
Device.DRT ( 6, 3, '0' );
Device.DRT ( 6, 4, '0' );
% for i = 1:length(xsteps)
%         Device.MOV('x', xsteps(i));
%         WAIT(Device);
%         Device.MOV('x z',[xsteps(i),0]);
%         WAIT(Device);
% end
Device.MOV('y',ywidth);
WAIT(Device);

laserdx = Device.qDRR(1,1,-1);
tlen = length(laserdx(:,1));
laserdy = Device.qDRR(2,1,tlen);
laserdz = Device.qDRR(3,1,tlen);
laserdd = Device.qDRR(4,1,tlen);
toc

X = laserdx(:,2);
D = v2d(laserdd(:,2));
Z = laserdz(:,2);
Y = laserdy(:,2);

D = D + (X- mean(X));
pos = [D,Y,Z];
[~,I] = sort(Y);
pos = pos(I,:);

minY = find(pos(:,1)<4.5&pos(:,1)>-3,1,'first');
maxY = find(pos(:,1)<4.5&pos(:,1)>-3,1,'last');
centerpos_l(1) = mean([pos(minY,2),pos(maxY,2)]);
subplot(1,2,2)
scatter(Y,D,2,'r','filled');
hold on
scatter(pos([minY;maxY],2),pos([minY;maxY],1),25,'b');
xline(centerpos_l(1));
Device.MOV('y z',[centerpos_l(1),centerpos_l(2)]);
WAIT(Device);
Device.KEN('zero');
centerpos = Device.qPOS('x y z');
end

