function im_proc_batch = imObjectFocusBatch(cnn_data_file, reg_size, scale)
% imageObjectFocusBatch processes the images a 3DMTS data capture file to 
%   de-emphasize the background of an object identified by a segmentation
%   contour.
%
%   cnn_data_file: file of data captured from a 3DMTS data collection run, 
%   which contains the images and segmentation contours.
%   reg_size: (int) size in pixels of region around segmented object that 
%   remains unprocessed.
%   scale: [0 1] the intensity multipler for the background pixels.
%   im_proc_batch: batch of processed images.

    load(cnn_data_file);

    im_proc_batch = zeros(size(images));
    seq_size = size(images,2);

    for ii = 1:seq_size
        array_images = squeeze(images(:,ii,:,:,:));
        array_segs = objects(:,ii);
        for jj = 1:9    % assumes 9 cameras in array
            image = im2double(squeeze(array_images(jj,:,:,:)));
            seg = array_segs{jj};
            im_proc_batch(jj,ii,:,:,:) = ...
                imObjectFocus(image,seg,reg_size,scale);
        end
    end

end

        
        
        