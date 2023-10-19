function [renumId] = SolvingTSP(target_new)
%SOLVINGTSP target_new‚ðxz•½–Ê‚É‚¨‚¢‚Ä‹ß‚¢‡‚É”Ô†‚ðU‚è‚È‚¨‚·ŠÖ”
%   Ú×à–¾‚ð‚±‚±‚É‹Lq
nStops = size(target_new,1);
stopsLon = target_new(:,1);
stopsLat = target_new(:,2);
idxs = nchoosek(1:nStops,2);

dist = hypot(stopsLat(idxs(:,1)) - stopsLat(idxs(:,2)), ...
             stopsLon(idxs(:,1)) - stopsLon(idxs(:,2)));
lendist = length(dist);

G = graph(idxs(:,1),idxs(:,2));
figure
hGraph = plot(G,'XData',stopsLon,'YData',stopsLat,'LineStyle','none','NodeLabel',{});
hold on

Aeq = spalloc(nStops,length(idxs),nStops*(nStops-1)); % Allocate a sparse matrix
for ii = 1:nStops
    whichIdxs = (idxs == ii); % Find the trips that include stop ii
    whichIdxs = sparse(sum(whichIdxs,2)); % Include trips where ii is at either end
    Aeq(ii,:) = whichIdxs'; % Include in the constraint matrix
end
beq = 2*ones(nStops,1);

intcon = 1:lendist;
lb = zeros(lendist,1);
ub = ones(lendist,1);

opts = optimoptions('intlinprog','Display','off');
[x_tsp,costopt,exitflag,output] = intlinprog(dist,intcon,[],[],Aeq,beq,lb,ub,opts);

x_tsp = logical(round(x_tsp));
Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2));

% figure;
% scatter(stopsLon,stopsLat,'filled');
% hold on
% highlight(hGraph,Gsol,'LineStyle','-');
% title('Solution with Subtours');

tourIdxs = conncomp(Gsol);
numtours = max(tourIdxs); % number of subtours
fprintf('# of subtours: %d\n',numtours);

A = spalloc(0,lendist,0); % Allocate a sparse linear inequality constraint matrix
b = [];
while numtours > 1 % Repeat until there is just one subtour
    % Add the subtour constraints
    b = [b;zeros(numtours,1)]; % allocate b
    A = [A;spalloc(numtours,lendist,nStops)]; % A guess at how many nonzeros to allocate
    for ii = 1:numtours
        rowIdx = size(A,1) + 1; % Counter for indexing
        subTourIdx = find(tourIdxs == ii); % Extract the current subtour
%         The next lines find all of the variables associated with the
%         particular subtour, then add an inequality constraint to prohibit
%         that subtour and all subtours that use those stops.
        variations = nchoosek(1:length(subTourIdx),2);
        for jj = 1:length(variations)
            whichVar = (sum(idxs==subTourIdx(variations(jj,1)),2)) & ...
                       (sum(idxs==subTourIdx(variations(jj,2)),2));
            A(rowIdx,whichVar) = 1;
        end
        b(rowIdx) = length(subTourIdx) - 1; % One less trip than subtour stops
    end

    % Try to optimize again
    [x_tsp,costopt,exitflag,output] = intlinprog(dist,intcon,A,b,Aeq,beq,lb,ub,opts);
    x_tsp = logical(round(x_tsp));
    Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2));
    
    % Visualize result
    hold on
    hGraph.LineStyle = 'none'; % Remove the previous highlighted path
    highlight(hGraph,Gsol,'LineStyle','-')
    drawnow
    
    % How many subtours this time?
    tourIdxs = conncomp(Gsol);
    numtours = max(tourIdxs); % number of subtours
    fprintf('# of subtours: %d\n',numtours)
end

title('Solution with Subtours Eliminated');
hold off

graphid = idxs(x_tsp,:);
numnode = size(graphid,1);
renumId = zeros(numnode,1);
renumId(1) = 1;
a = graphid(graphid(:,1) == 1,2);
b = graphid(graphid(:,2) == 1,1);
a = reshape([a,b],[],1);
renumId(2) = a(1);
for i = 2:numnode
    a = graphid(graphid(:,1) == renumId(i),2);
    b = graphid(graphid(:,2) == renumId(i),1);
    a = reshape([a,b],[],1);
    if a(1) ~= renumId(i-1)
        renumId(i+1) = a(1);
    else
        renumId(i+1) = a(2);
    end
end
renumId = renumId(1:end-1);

hold on
for i = 1:size(target_new,1)
    text(target_new(renumId(i),1),target_new(renumId(i),2),cellstr(num2str(i)),'Color','r');
end
end

