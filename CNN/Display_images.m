%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Display collected images from VREP Sim
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%image_count = count(1,end);
image_count = 1;

% images: (step, pixel i, pixel j, color BGR)

for i = 1:image_count
    img = flip(flip(reshape(images(i,:,:,:),256,256,3),1),3);
    figure;
    imagesc(img);
end
