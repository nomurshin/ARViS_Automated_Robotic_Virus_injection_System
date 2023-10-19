function mask_bold = BoldIsolate(mask,params)
%BoldIsolate 太い主要血管のみをmaskから取り出した画像を返す。

erodePx = params(1);
dilatePx = params(2);
N = size(mask,3);
mask_bold = zeros(size(mask));
for i = 1:N
    A = squeeze(mask(:,:,i));
    B = imerode(A,strel('disk',erodePx));
    C = imdilate(B,strel('disk',dilatePx));
    if i == 1
        figure;
        subplot(1,2,1)
        imshow(B)
        subplot(1,2,2);
        imshow(imfuse(A,C));
    end
    mask_bold(:,:,i) = C>80;
end
mask_bold = logical(mask_bold);
end

