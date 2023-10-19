function target_new = OptimLocate(results,boundary,targetlen,distlimit,F2S)
% This algorithm is a variation of Lloyd's algorithm
% 

% % setting variables
boundary_l = boundary(1:6,:);
boundary_r = boundary(7:end,:);
distw_l = results.distw_l;
distw_r = results.distw_r;
NonVesselOnSurf_l = results.NonVesselOnSurf_l;
NonVesselOnSurf_r = results.NonVesselOnSurf_r;

% 血管からの距離が0.05以下であるpixelは候補から除外
NonVesselOnSurf_l_rmd = NonVesselOnSurf_l(distw_l>distlimit,:);
NonVesselOnSurf_r_rmd = NonVesselOnSurf_r(distw_r>distlimit,:);
distw_l_rmd = distw_l(distw_l>distlimit);
distw_r_rmd = distw_r(distw_r>distlimit);

% 指定された領域の面積を計算
boundary_l_surf = F2S(boundary_l);
[k_l,av_l] = convhull(boundary_l_surf(:,[2,3]));
boundary_r_surf = F2S(boundary_r);
[k_r,av_r] = convhull(boundary_r_surf(:,[2,3]));
% targetとなる距離をもとにInjection siteの個数を決定
app = sqrt(3) * targetlen^2 / 2;
num_l = round(av_l / app)+10;
if num_l > 200
    num_l = 200;
end
num_r = round(av_r / app)+10;
if num_r > 200
    num_r = 200;
end
% kgreedy + クラスタ中心
clustArea_L_log = nan(200,50);
clustArea_R_log = nan(200,50);
S_l = [];
flag = true;
% We use "VoronoiLimit" function.
% Jakob Sievers (2022). VoronoiLimit(varargin) 
% (https://www.mathworks.com/matlabcentral/fileexchange/34428-voronoilimit-varargin), 
% MATLAB Central File Exchange. Retrieved April 5, 2022.
cnt = 1;
while num_l > 0 && flag && ~isempty(NonVesselOnSurf_l_rmd)
    [S_l,d_max_l] = cluster_center_k_greedy_Fnc(num_l,NonVesselOnSurf_l_rmd,S_l,false);
    % 各Injection siteの責任領域の面積を計算する
    clustArea = nan(length(S_l),1);
    % injection siteをクエリ点としたボロノイ線図を作成する
    pos = NonVesselOnSurf_l_rmd(S_l,[2,3]);
    bnd_pnts =  boundary_l_surf(k_l,[2,3]);
    [V,C,~]=VoronoiLimit(pos(:,1),pos(:,2),'bs_ext',bnd_pnts,'figure','off');
    for i = 1:size(pos,1)
        polyV = intersect(polyshape(V(C{i},1),V(C{i},2)),polyshape(bnd_pnts(:,1),bnd_pnts(:,2)));
        %plot(polyV)
        in = inpolygon(pos(:,1),pos(:,2),polyV.Vertices(:,1),polyV.Vertices(:,2));
        clustArea(in) = area(polyV);
        hold on;
    end
%     plot(bnd_pnts(:,1),bnd_pnts(:,2),'-');
%     hold on;
%     plot(pos(:,1),pos(:,2),'Marker','o','MarkerFaceColor',[0 .75 .75],'MarkerEdgeColor','k','LineStyle','none');
%     axis('equal')
%     hold off;
    clustArea_L_log(1:length(clustArea),cnt) = clustArea;
    % 面積が理想的な面積appを上回るまで、最も面積の小さいクラスタのセントロイドを取り除く
    if mean(clustArea) <= app
        [~,minid] = sort(clustArea,1,'ascend');
        S_l(minid(1:2)) = [];
        num_l = num_l-2;
        num_l = num_l+1;
    else
        flag = false;
    end
    cnt = cnt + 1;
end
figure;
for i = 1:size(pos,1)
    polyV = intersect(polyshape(V(C{i},1),V(C{i},2)),polyshape(bnd_pnts(:,1),bnd_pnts(:,2)));
    plot(polyV)
    hold on;
end
plot(bnd_pnts(:,1),bnd_pnts(:,2),'-');
hold on;
plot(pos(:,1),pos(:,2),'Marker','o','MarkerFaceColor',[0 .75 .75],'MarkerEdgeColor','k','LineStyle','none');
axis('equal')
title('left hemisphere')
S_r = [];
flag = true;
cnt = 1;
while num_r > 0 && flag && ~isempty(NonVesselOnSurf_r_rmd)
    [S_r,d_max_r] = cluster_center_k_greedy_Fnc(num_r,NonVesselOnSurf_r_rmd,S_r,false);
    % 各Injection siteの責任領域の面積を計算する
    clustArea = nan(length(S_r),1);
    % boundaryで囲まれたinjection siteのボロノイ線図を作成する
    pos = NonVesselOnSurf_r_rmd(S_r,[2,3]);
    bnd_pnts =  boundary_r_surf(k_r,[2,3]);
    [V,C,~]=VoronoiLimit(pos(:,1),pos(:,2),'bs_ext',bnd_pnts,'figure','off');
    for i = 1:size(pos,1)
        polyV = intersect(polyshape(V(C{i},1),V(C{i},2)),polyshape(bnd_pnts(:,1),bnd_pnts(:,2)));
        %plot(polyV)
        in = inpolygon(pos(:,1),pos(:,2),polyV.Vertices(:,1),polyV.Vertices(:,2));
        clustArea(in) = area(polyV);
        hold on;
    end
%     plot(bnd_pnts(:,1),bnd_pnts(:,2),'-');
%     hold on;
%     plot(pos(:,1),pos(:,2),'Marker','o','MarkerFaceColor',[0 .75 .75],'MarkerEdgeColor','k','LineStyle','none');
%     axis('equal')
%     hold off;
    clustArea_R_log(1:length(clustArea),cnt) = clustArea;
    % 面積が理想的な面積appを上回るまで、最も面積の小さいクラスタのセントロイドを取り除く
    if mean(clustArea) <= app
        [~,minid] = sort(clustArea,1,'ascend');
        S_r(minid(1:2)) = [];
        num_r = num_r-2;
        num_r = num_r+1; % "cluster_center_k_greedy_Fnc"によって現存する点
        %から最も離れたpixelを新たなinjection siteとして追加する事になる。
    else
        flag = false;
    end
    cnt = cnt + 1;
end

figure;
for i = 1:size(pos,1)
    polyV = intersect(polyshape(V(C{i},1),V(C{i},2)),polyshape(bnd_pnts(:,1),bnd_pnts(:,2)));
    plot(polyV)
    hold on;
end
plot(bnd_pnts(:,1),bnd_pnts(:,2),'-');
hold on;
plot(pos(:,1),pos(:,2),'Marker','o','MarkerFaceColor',[0 .75 .75],'MarkerEdgeColor','k','LineStyle','none');
axis('equal')
title('right hemisphere')
figure;
histogram(clustArea_R_log(:,1));
hold on
histogram(clustArea_R_log(:,find(~isnan(clustArea_R_log(1,:)),1,'last')));
legend('before','after')
figure;
histogram(clustArea_L_log(:,1));
hold on
histogram(clustArea_L_log(:,find(~isnan(clustArea_L_log(1,:)),1,'last')));
legend('before','after')
figure;
plot(mean(clustArea_L_log,1,'omitnan'));
hold on
plot(mean(clustArea_R_log,1,'omitnan'));
target_new = [NonVesselOnSurf_l_rmd(S_l,:);NonVesselOnSurf_r_rmd(S_r,:)];
end