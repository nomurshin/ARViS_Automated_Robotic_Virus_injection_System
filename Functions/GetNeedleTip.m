function [col,row] = GetNeedleTip(im)
    im = rgb2gray(im);
    I = imadjust(imtophat(im,strel('rectangle',[8,20])),[0.01,0.1]);
    BW = bwareaopen(I>100,500,4);
    CC = bwconncomp(BW);
    stats = regionprops(CC, 'BoundingBox');
    aspectRatios = zeros(length(stats),1);
    for i = 1:length(stats)
        aspectRatios(i) = stats(i).BoundingBox(4)/stats(i).BoundingBox(3);  % height/width
    end
    [~,idx] = max(aspectRatios);

    stats = regionprops(CC, 'PixelIdxList');  % get pixel indices of each object
    pixelList = stats(idx).PixelIdxList;  % get pixel indices of the object with largest aspect ratio
    [rows, cols] = ind2sub(size(BW), pixelList);  % convert linear indices to subscripts
    [row, bottomIdx] = maxk(rows,10);  % find the bottom-most pixel
    col = cols(bottomIdx);  % get the column of the bottom-most pixel
    row = mean(row);
    col = mean(col); 
end
