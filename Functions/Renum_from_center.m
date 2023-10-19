function points_new = Renum_from_center(points,center,width)
%RENUM_FROM_CENTER 
%   詳細説明をここに記述
D = pdist2(center,points);
figure;
histogram(D);
N = ceil(max(D/width));
points_new = zeros(size(points));

%id = D<width;
%a = SolvingTSP(points(id,:));
id = D<width;
b = points(id,:);
a = SolvingTSP(b);
points_new(1:sum(id),:) = b(a,:);
crrt = sum(id);
for i = 2:N
    id = D>=(i-1)*width & D<i*width;
    
    if sum(id) < 2
        points_new(crrt+1,:) = points(id,:);
        crrt = crrt+1;
    else
        b = points(id,:);
        a = SolvingSTP([points_new(crrt,:);b]);
        points_new(crrt+1:crrt+sum(id),:) = b(a(2:end)-1,:);
        crrt = crrt + sum(id);
    end
end


figure;
scatter(points_new(:,1),points_new(:,2),25,1:size(points_new,1),'filled');
hold on
for i = 1:size(points_new,1)
    text(points_new(i,1),points_new(i,2),cellstr(num2str(i)),'Color','r');
end
end

