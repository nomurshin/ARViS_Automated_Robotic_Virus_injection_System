function results = CulcVesselDistMat(mask_p,boundary,F2S)

% setting variables
boundary_l = boundary(1:6,:);
boundary_r = boundary(7:end,:);
[k_l,~] = convhull(boundary_l);
[k_r,~] = convhull(boundary_r);

% Detect object perimeter in binary images
mask_bound = bwperim(mask_p,8);
figure;
imshowpair(mask_p,mask_bound,'montage');
[y,x,~] = find(mask_bound);
VesselOnFig = [x,y];
VesselOnFig_l = VesselOnFig(inpolygon(VesselOnFig(:,1)...
    ,VesselOnFig(:,2),(boundary_l(k_l,1)-mean(boundary_l(k_l,1)))*1.2+mean(boundary_l(k_l,1))...
    ,(boundary_l(k_l,2)-mean(boundary_l(k_l,2)))*1.2+mean(boundary_l(k_l,2))),:);
VesselOnFig_r = VesselOnFig(inpolygon(VesselOnFig(:,1)...
    ,VesselOnFig(:,2),(boundary_r(k_r,1)-mean(boundary_r(k_r,1)))*1.2+mean(boundary_r(k_r,1)),...
    (boundary_r(k_r,2)-mean(boundary_r(k_r,2)))*1.2+mean(boundary_r(k_r,2))),:);
VesselOnSurf_l = F2S(VesselOnFig_l);
VesselOnSurf_r = F2S(VesselOnFig_r);
% Extract non-vascular areas that are included in the INJECTION area
[y,x,~] = find(~mask_p);
NonVesselOnFig = [x,y];
NonVesselOnFig_l = NonVesselOnFig(inpolygon(NonVesselOnFig(:,1)...
    ,NonVesselOnFig(:,2),boundary_l(k_l,1),boundary_l(k_l,2)),:);
NonVesselOnFig_r = NonVesselOnFig(inpolygon(NonVesselOnFig(:,1),...
    NonVesselOnFig(:,2),boundary_r(k_r,1),boundary_r(k_r,2)),:);
% Converted to Hexapod coordinate system
NonVesselOnSurf_l = F2S(NonVesselOnFig_l);
NonVesselOnSurf_r = F2S(NonVesselOnFig_r);
% Search for the nearest neighbor between the outer edge of a blood vessel and a pixel of a non-vascular region. Save the distance as distw.
VesselOnSurf_l_gpu = gpuArray(VesselOnSurf_l);
VesselOnSurf_r_gpu = gpuArray(VesselOnSurf_r);
NonVesselOnSurf_r_gpu = gpuArray(NonVesselOnSurf_r);
NonVesselOnSurf_l_gpu = gpuArray(NonVesselOnSurf_l);

[~,distw_l_gpu] = dsearchn(VesselOnSurf_l_gpu,NonVesselOnSurf_l_gpu);
[~,distw_r_gpu] = dsearchn(VesselOnSurf_r_gpu,NonVesselOnSurf_r_gpu);
distw_l = gather(distw_l_gpu);
distw_r = gather(distw_r_gpu);
results = struct();
results.distw_l = distw_l;
results.distw_r = distw_r;
results.NonVesselOnSurf_l = NonVesselOnSurf_l;
results.NonVesselOnSurf_r = NonVesselOnSurf_r;
end