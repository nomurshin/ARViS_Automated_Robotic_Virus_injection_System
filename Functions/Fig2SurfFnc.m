function target = Fig2SurfFnc(x_fig,targetonfig,tform_fig_inv,F_camera)
%FIG2SURFFNC ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q
h = x_fig(1);

theta = deg2rad(-x_fig(2));
u = deg2rad(-x_fig(3));
%w = deg2rad(-x_fig(4));

% % Figure��ł̍��W(px)����J�������_��Hexapod���W(mm)�ɕϊ�
target = transformPointsInverse(tform_fig_inv,targetonfig);
target = [F_camera(target(:,1),target(:,2)),target(:,1),target(:,2)];

rotX = [1 0 0;0 cos(theta) sin(theta);0 -sin(theta) cos(theta)];
trans = [0, 0, 0];
% Y���Ɋւ��ĉ�] 1/20 �ǉ�
rotY = [cos(u) 0 sin(u);0 1 0;-sin(u) 0 cos(u)];
% Z���Ɋւ��ĉ�] 1/20 �ǉ�
%rotZ = [cos(w) -sin(w) 0;sin(w) cos(w) 0;0 0 1];
tform = rigid3d(rotY*rotX,trans);
ptCloudOut = pctransform(pointCloud(target),tform);
target = ptCloudOut.Location;

% �g��k��
target = [target(:,1)+h,target(:,2),target(:,3)];



end

