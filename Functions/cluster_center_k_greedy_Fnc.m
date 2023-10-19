function [S,d_max] = cluster_center_k_greedy_Fnc(K,P,S,figFlag,MAXREPNUM)
%CLUSTER_CENTER_K_GREEDY k-greedy法でのクラスタを作り、その中心に近い点を選ぶ
% Lloyd's algorithmの移動場所が離散的になっているバージョン
%   P : data points. #points * #dim matrix
%   W : weight of each points. #points * 1 array.
%   S : index of selected points
%   S_w : total weight of selected points.
%   d_max : radius of longest distance from cluster center.
arguments
    K (1,1) {mustBeInteger(K),mustBePositive(K)} = 10
    P = rand(2000,2)
    S = []
    figFlag = true
    MAXREPNUM = 10;
end
% variables for record
d_max_his = nan(MAXREPNUM+1,1);
S_w_his = nan(MAXREPNUM+1,1);
if isempty(S)
    S = nan(K,1);
    num = size(P,1);
    S(1) = randperm(num,1);
    for i = 2:K
        [~,d] = dsearchn(P(S(1:i-1),:),P);
        idx = randsample(length(d),1,true,d); %新たな点を追加する際にランダム性を持たせた
        %[~,idx] = max(d);
        S(i) = idx;
    end
end
if size(S,1) < K
    for i = size(S,1)+1:K
        [~,d] = dsearchn(P(S(1:i-1),:),P);
        [~,idx] = max(d);
        S(i) = idx;
    end
end
[a,d] = dsearchn(P(S,:),P);
d_max = max(d);
d_max_g = d_max;

d_max_his(1) = d_max_g;
j = 1;
plateauF = false;
while j < MAXREPNUM && plateauF == false
    for i = 1:K
        idx_A2C = find(a == i);
        C = P(idx_A2C,:);
        centroid = mean(C,1);
        centroid_idx = dsearchn(C,centroid);
        S(i) = idx_A2C(centroid_idx);
    end
    [a,d] = dsearchn(P(S,:),P);
    d_max_his(j+1) = max(d);
    if j > 1
        if S_w_his(j-1) == S_w_his(j)
            plateauF = true;
        end
    end
    j = j+1;
end

if figFlag == true
    figure;
    plot(d_max_his);
    yline(d_max_g);
    title('History of d_max')
end
end

