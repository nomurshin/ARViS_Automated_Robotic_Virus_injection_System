function [e_surf,x] = EstimateSurface(standard,template,nonlcon)
%ESTIMATESURFACE �v���_ standard����͂��A���肳�ꂽ�\�� e_surf���o�͂���
%   �ڍא����������ɋL�q
fun = @(x)distculfcn(x,standard,template);
options = optimoptions(@fmincon,'Algorithm','active-set','PlotFcn','optimplotfval','UseParallel',true,'MaxFunctionEvaluations',2.0e+03);
fvalmin = 1000;
starttime = tic;
x = 'dummy';
for j = 1:2
    x0 = [0.9*(rand-0.5)+1,0.9*(rand-0.5)+1,0.9*(rand-0.5)+1,(rand-0.5)*18,36*(rand-0.5),3.6*(rand-0.5),1.8*(rand-0.5),36*(rand-0.5),36*(rand-0.5)];
    [xi,fval] = fmincon(fun,x0,[],[],[],[],[0.5,0.5,0.2,-10,-20,-2,-2,-20,-20],[1.5,1.5,1.5,10,20,2,2,20,20],nonlcon,options);
    if fval < fvalmin
        fvalmin = fval;
        x = xi;
    end
end
endtime = toc(starttime);
disp(endtime);
% �����Ɋւ��ĉ�]
theta = deg2rad(x(5));
u = deg2rad(x(8));
w = deg2rad(x(9));
rotX = [1 0 0;0 cos(theta) sin(theta);0 -sin(theta) cos(theta)];
trans = [0, 0, 0];
% Y���Ɋւ��ĉ�] 1/20 �ǉ�
rotY = [cos(u) 0 sin(u);0 1 0;-sin(u) 0 cos(u)];
% Z���Ɋւ��ĉ�] 1/20 �ǉ�
rotZ = [cos(w) -sin(w) 0;sin(w) cos(w) 0;0 0 1];
tform = rigid3d(rotX*rotY*rotZ,trans);
ptCloudOut = pctransform(pointCloud(template),tform);
ptemplate = ptCloudOut.Location;

e_surf = [ptemplate(:,1)*x(1),x(3)*ptemplate(:,2),ptemplate(:,3)*x(2)];
e_surf = correctbregma(e_surf,x(6),x(7),x(4),1); % ���_�����f����ł�bregma����hexapod�n�ł̌��_�Ɉړ��B

end


