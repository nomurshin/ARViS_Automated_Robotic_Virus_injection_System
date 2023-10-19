function [U,V,W] = RotationAxisFnc(u_a,w_a,theta_a)
%ROTATIONAXISFNC 元のY軸周りの回転thetaを、既にu,w回転を済ませたX'Y'Z'座標系でのUVW回転に変換する関数
%   詳しくはRotationAxisFnc.mlxを参照
%ロドリゲスの回転公式Rを成分表示する
u_a = deg2rad(u_a);
w_a = deg2rad(w_a);
theta_a = deg2rad(theta_a);
syms n1 n2 n3 theta
R = cos(theta)*eye(3,3)+(1-cos(theta))*[n1,n2,n3]'*[n1,n2,n3]...
    +sin(theta)*[[0,-n3,n2];[n3,0,-n1];[-n2,n1,0]];
g(n1,n2,n3,theta) = R;
%同様にしてU->V->W回転する回転行列Dを成分表示する
syms su cu sv cv sw cw
D = [[cw,-sw,0];[sw,cw,0];[0,0,1]]*[[cv,0,sv];[0,1,0];[-sv,0,cv]]...
    *[[1,0,0];[0,cu,-su];[0,su,cu]];
f(su,cu,sv,cv,sw,cw) = D;
%Y軸の単位ベクトル(1,0,0)のu = u_a,w = w_aだけ回転させた座標系X'Y'Z'における座標を求める。座標系の変換では回転運動の行列における角度の符号を反転させることに注意。
n = f(sin(-u_a),cos(-u_a),sin(0),cos(0),sin(-w_a),cos(-w_a))*[0,1,0]';
%元のY軸周りにtheta_a度回転させるのと同じ座標へ移動するUVW回転を求める。
eqns = g(n(1),n(2),n(3),theta_a) == D;

eqns2 = [eqns(1,3);eqns(2,1);eqns(3,2);cu^2+su^2==1;cv^2+sv^2==1;cw^2+sw^2==1];
S  = solve(eqns2,[su,cu,sv,cv,sw,cw]);
%一部の方程式しか用いていないため、ほかの方程式が成立しているかどうか確認する。
summatrix = nan(length(S.su),1);
for i = 1:length(S.su)
    summatrix(i) =  sum(vpa(g(n(1),n(2),n(3),theta_a)-f(S.su(i),S.cu(i),S.sv(i),S.cv(i),S.sw(i),S.cw(i))),'all');
end
%すべての要素について0に近い解について、角度u,v,wを求める。
idx = find(summatrix <0.00001);
u_a = double([atan(S.su(idx)./S.cu(idx)),asin(S.su(idx))]);
v_a = double([atan(S.sv(idx)./S.cv(idx)),asin(S.sv(idx))]);
w_a = double([atan(S.sw(idx)./S.cw(idx)),asin(S.sw(idx))]);
%acosとasinという二つの求め方が等しくなる値を求める
idx2 = find((u_a(:,1) == u_a(:,2)) & (v_a(:,1) == v_a(:,2)) & (w_a(:,1) == w_a(:,2)));
U = rad2deg(u_a(idx2(1),1));
V = rad2deg(v_a(idx2(1),1));
W = rad2deg(w_a(idx2(1),1));

end