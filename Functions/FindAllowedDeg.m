function [v,w] = FindAllowedDeg(normals,device,targetpos)
%UNTITLED2 法線ベクトルを受け、移動可能な角度ｖ、ｗを二分探索する

[v,w] = Grad2VW(normals);
if device.qVMO({'x','y','z','u','v','w'},[targetpos,0,v,w]) == true && ...
        device.qVMO({'x','y','z','u','v','w'},EstimateINJCordinate([targetpos,0,v,w],[2,0,0])) == true && ...
        device.qVMO({'x','y','z','u','v','w'},EstimateINJCordinate([targetpos,0,v,w],[-2.5,0,0])) == true &&...
        device.qVMO({'x','y','z','u','v','w'},[targetpos-[1,0,0],0,v,w]) == true && ...
        checkaround([targetpos,0,v,w],device) == true
    return
else
    bannedNorm = normals;
    allowedNorm = [1,0,0];
    for i = 1:3
        candNorm = (bannedNorm+allowedNorm)/2;
        candNorm = candNorm/vecnorm(candNorm,2,2);
        [v,w] = Grad2VW(candNorm);
        if device.qVMO({'x','y','z','u','v','w'},[targetpos,0,v,w]) == true && ...
                device.qVMO({'x','y','z','u','v','w'},EstimateINJCordinate([targetpos,0,v,w],[2,0,0])) == true && ...
                device.qVMO({'x','y','z','u','v','w'},EstimateINJCordinate([targetpos,0,v,w],[-2.5,0,0])) == true && ...
                device.qVMO({'x','y','z','u','v','w'},[targetpos-[1,0,0],0,v,w]) == true && ...
                checkaround([targetpos,0,v,w],device) == true
            allowedNorm = candNorm;
        else
            bannedNorm = candNorm;
        end
    end
[v,w] = Grad2VW(allowedNorm);
end
end
function id = checkaround(pos,device)
id = true;
[X,Y,Z] = sphere(3);
X = reshape(X*0.5,[],1);Y = reshape(Y*0.5,[],1);Z = reshape(Z*0.5,[],1);
for i = 1:length(X)
    id = device.qVMO({'x','y','z','u','v','w'},[pos(1:3)+[X(i),Y(i),Z(i)],pos(4:end)]) && id;
end
end
function [v,w] = Grad2VW(n)
%UNTITLED 勾配ベクトルn #*(x,y,z)を入力し、そのベクトルと一致するv,w回転を与える関数
% 座標系はHexapod系と一致し、(x,y,z) = (1,0,0)を回転の初期位置としている。
Nxy = [n(:,1:2),zeros(length(n(:,1)),1)];
Nxy = Nxy./vecnorm(Nxy,2,2);
w = acosd(Nxy*[1,0,0]');
w = Nxy(:,2)./abs(Nxy(:,2)).*w;
Nxz = [n(:,1),zeros(length(n(:,1)),1),n(:,3)];
Nxz = Nxz ./ vecnorm(Nxz,2,2);
v = acosd(Nxz*[1,0,0]');
v = -Nxz(:,3)./abs(Nxz(:,3)).*v;
end

function [pos_est] = EstimateINJCordinate(pos,INJxyz)
x = pos(1);y = pos(2);z = pos(3); u = deg2rad(pos(4));v = deg2rad(pos(5)); w = deg2rad(pos(6));
U = [[1,0,0];[0,cos(u),-sin(u)];[0,sin(u),cos(u)]];
V = [[cos(v),0,sin(v)];[0,1,0];[-sin(v),0,cos(v)]];
W = [[cos(w),-sin(w),0];[sin(w),cos(w),0];[0,0,1]];
R = W*V*U;
pos_est = [[R*INJxyz' + [x;y;z]];0;v;w];
end
