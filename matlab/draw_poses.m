clf
close all
clear all

figure;
hold on;

names_struct = dir('C:\Users\chris\Dropbox\Work\Agricultural Robotics\move_to_see\results\experiment_1\*_exp_1.mat');
names = {names_struct.name};

stride = 4;

for n = 1:9:length(names)

    load(strcat('C:\Users\chris\Dropbox\Work\Agricultural Robotics\move_to_see\results\experiment_1\',names{:,n}))
    
    m = length(1:stride:length(ee_pose));

    Tf = zeros(4,4,m);
    trans = zeros(3,m);

    count = 0;
    for i = 1:stride:size(ee_pose,1)
        count = count + 1;
    %tform = homogeneous transform

        T = trvec2tform(ee_pose(i,1:3));
        %eul_angles = [ee_pose(i,6),ee_pose(i,5),ee_pose(i,4)]
        R = my_eul2tform(ee_pose(i,4:6),'XYZ');

        Tf(:,:,count) = T*R;

        trans(:,count) = ee_pose(i,1:3);

    end
    %the corresponding inverse conversions exist
    %from Peter's toolbox:
    %Plotting the coordinate frame at a specified pose:


    %e for a sequence of n poses size(Hgt) =  (4x4xn)

    tLength = 0.025;
    th = 0.3;

    trans = ee_pose';

    %object_position = [0.5,0.59,0.69];

    for i = 1:count
        %trplot(squeeze(Tf(:,:,i)),'arrow','width',0.2,'rgb','text',false,'color','blue','length',tLength,'thick',th,...
        %    'text_opts', {'FontSize', 0.1});
        trplot(squeeze(Tf(:,:,i)),'arrow','width',0.2,'rgb','color','blue','length',tLength,'thick',th);
        plot3(ee_pose(:,1),ee_pose(:,2),ee_pose(:,3),'color',[0.43,0.67,0.27],'LineWidth',2.0)
        hold on
    end
    
    
    plot3(capsicum_position(1),capsicum_position(2),capsicum_position(3),'r.','markersize',50);
    plot3(capsicum_position(1),occlusion_y,capsicum_position(3),'g.','markersize',50);

    
end

hold off;
%axis equal 
grid on

xlabel('X','FontSize',20);
ylabel('Y','FontSize',20);
zlabel('Z','FontSize',20);

text(capsicum_position(1),capsicum_position(2),capsicum_position(3)-0.01,['Targets'],'HorizontalAlignment','center','FontSize',20);

%%
xlim([0.0,0.45]);
ylim([0.5,1.0]);
zlim([0.5,0.7]);
