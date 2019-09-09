function im_proc = imObjectFocus(image, seg, reg_size, scale)
% imageObjectFocus de-emphasizes the background of an object identified in 
%   the image by a segmentation contour.
%
%   image: image to process, assummes size of 224 x 224.
%   seg: (struct) segmentation data with a property "contour" that defines 
%   the segmentation contour of the object.
%   reg_size: (int) size in pixels of region around segmented object that 
%   remains unprocessed.
%   scale: [0 1] the intensity multipler for the background pixels.
%   im_proc: processed image.

    image = im2double(image);

    if isempty(seg)
        mask = scale*ones(224);
    else
        cont = double(squeeze(seg.contour) + 1)'; %correct for python index
        se = strel('square',2*reg_size+1);
        mask = imdilate(poly2mask(cont(1,:),cont(2,:),224,224),se);
        mask = scale*imcomplement(mask) + mask;
    end
    im_proc = image.*mask;
        
end