function f = surfaceculfcn(x,target,BregmasOnPano)
%SURFACECULFCN �_x�̍��W���󂯎���Ċp�x���œK�����邱�Ƃɂ��^����ꂽ�_��Figure��ŗ\�������_�̋������ŏ�������B
%   h = x(1) : ��]����DP���̍���
%   theta = x(2) : X������̊p�x�@�ideg�j
%   u = x(3) : Y������̊p�x (deg)
%   standardp : Random sampling �ɂ���ē���ꂽ��_�̍��W(mm),target��index
%   targetonfig : ��_�̉摜��ł̍��W(px)
%ppm = 274.45; % pixel per mm;
h = x(1);

% BregmasOnPano(1,:) �͍ŏ���Autostitch�ɓ��ꂽ�摜�̈ړ���B�܂�ABregma.

% �g��k��
surfgrid = [target(:,1)-h,target(:,2),target(:,3)];

% �����Ɋւ��ĉ�]
theta = deg2rad(x(2));
u = deg2rad(x(3));
%w = deg2rad(x(4));
rotX = [1 0 0;0 cos(theta) sin(theta);0 -sin(theta) cos(theta)];
trans = [0, 0, 0];
% Y���Ɋւ��ĉ�] 1/20 �ǉ�
rotY = [cos(u) 0 sin(u);0 1 0;-sin(u) 0 cos(u)];
% Z���Ɋւ��ĉ�] 1/20 �ǉ� ��]��2���ŏ\���Ȃ̂ŁA�ꎩ�R�x�팸
%rotZ = [cos(w) -sin(w) 0;sin(w) cos(w) 0;0 0 1];
rotZ = eye(3);
tform = rigid3d(rotX*rotY*rotZ,trans);

ptCloudOut = pctransform(pointCloud(surfgrid),tform);
surfgrid = ptCloudOut.Location;
% % Bregma�̈ʒu����
% ptCloudOut2 = pctransform(pointCloud([0,-h,0]),tform);
% bregma = ptCloudOut2.Location;
% % Figure��ł̍��W(px)�ɕϊ�]
% surfgrid_for_standard = [bregma([1,3]);surfgrid(flags(:,3),[1,3])];
% 
% tform_fig = fitgeotrans(BregmasOnPano,surfgrid_for_standard,'lwm',12);%�p�����[�^�[���  �olwm 12},{polynomial 4}
% targetonfig = transformPointsInverse(tform_fig,surfgrid(:,[1,3]));

surfgrid_for_standard = surfgrid(:,[2,3]);
% surfgrid_for_standard_mirrored = [-surfgrid_for_standard(:,1),surfgrid_for_standard(:,2)];
% surfgrid_for_standard = [surfgrid_for_standard;surfgrid_for_standard_mirrored];
tform_fig = fitgeotrans(BregmasOnPano,surfgrid_for_standard,'lwm',12); %�p�����[�^�[���  �olwm 12},{polynomial 4}
targetonfig = transformPointsInverse(tform_fig,surfgrid(:,[2,3]));

%surfgrid = surfgrid * ppm;
%surfgrid = [-a*surfgrid(:,1),surfgrid(:,2),-b*surfgrid(:,3)];
%targetonfig = [surfgrid(:,1),surfgrid(:,3)]+[ones(size(surfgrid,1),1)*bregma(1),ones(size(surfgrid,1),1)*bregma(2)];
% Figure��ł̋������v�Z
%f = sum(diag(pdist2(targetonfig,BregmasOnPano)),'omitnan');
f = sum(vecnorm(targetonfig-BregmasOnPano,2,2),'omitnan');
end

