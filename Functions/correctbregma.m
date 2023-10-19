function points_modified = correctbregma(points,m,a,d,inv)
%CORRECTBREGMA standard�̍��W�ƁAbregma���W�̐^�̈ʒu����̂�����󂯂āA������C������standard�̍��W��Ԃ��B
%   standard : ����W�Bhexapod���W�n�ɂ�����(x,y,z)
%   m : ML����bregma���W�̂���
%   a : AP��
%   d : DP��
% ML���̂�����C������B
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

