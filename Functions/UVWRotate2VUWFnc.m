function [u_h,v_h,w_h] = UVWRotate2VUWFnc(u_a,v_a,w_a)
%UVWROTATE2VUWFNC Hexapod系での回転計算であるUVWを、VUWに変更する関数
syms u v w
U = [[1,0,0];[0,cos(u),-sin(u)];[0,sin(u),cos(u)]];
V = [[cos(v),0,sin(v)];[0,1,0];[-sin(v),0,cos(v)]];
W = [[cos(w),-sin(w),0];[sin(w),cos(w),0];[0,0,1]];
%それぞれの軸周りの回転行列を定義した。
%y軸周りにv_a度だけ回転させた後、U軸でu_a度、W軸でw_a度回転させたいとする。
%radに変換
v_a = deg2rad(v_a);u_a = deg2rad(u_a);w_a = deg2rad(w_a);
%Hexapod内部処理はUVWの順であり、これを関数f(u,v,w)で表現する。
syms f(u,v,w)
f(u,v,w) = W*V*U;
%実際に動かしたいのはVUWの順で、これを関数g(u,v,w)で表現する。
syms g(u,v,w)
g(u,v,w) = W*U*V;
%VUW順で角度が指定された時、UVW順での角度に変換するには以下の方程式を解けば良い。
equals = f(u,v,w) == g(u_a,v_a,w_a);
%数値的に解を求める。この時、変数は3つなので、適切な方程式を3*3行列から３つ選び出す。
equal_chosen = [equals(1,2);equals(2,3);equals(3,1)];
answer = vpasolve(equal_chosen);
%degreeに直して表示
u_h = rad2deg(double(answer.u));
v_h = rad2deg(double(answer.v));
w_h = rad2deg(double(answer.w));
end

