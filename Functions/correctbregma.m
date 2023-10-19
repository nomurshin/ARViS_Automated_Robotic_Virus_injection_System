function points_modified = correctbregma(points,m,a,d,inv)
%CORRECTBREGMA standardの座標と、bregma座標の真の位置からのずれを受けて、ずれを修正したstandardの座標を返す。
%   standard : 基準座標。hexapod座標系における(x,y,z)
%   m : ML軸のbregma座標のずれ
%   a : AP軸
%   d : DP軸
% ML軸のずれを修正する。
if ~exist('inv','var')
    inv = 0;
end
if inv == 0 
    points(:,1) = points(:,1)-m;
    points(:,3) = points(:,3)-a;
    points(:,2) = points(:,2)-d;
    points_modified = points;
else
    points(:,3) = points(:,3)+a;
    points(:,2) = points(:,2)+d;
    points(:,1) = points(:,1)+m;
    points_modified = points;
end
end

