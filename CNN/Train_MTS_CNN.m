%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Train MTS CNN
% =============
% 
% Train the Move To See CNN using a pre-trained CNN and data from VREP MTS
% simulations.
%
% Paul Zapotezny-Anderson (n4337948)
% 26 May 2019
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear

% load MTS data for training and validation of MTS CNN
load('MTSdataset.mat');
targetSize = size(Ytrain,2);        %direction gradient + orientation updates
% targetSize = size(Ytrain,2)-2;      %direction gradients only
Xtrain = rescale(Xtrain);
Xval = rescale(Xval);
augmenter = imageDataAugmenter( ...
    'RandXTranslation', [-3,3],...
    'RandYTranslation', [-3,3]);
augmentedTrainData = augmentedImageDatastore([224,224], Xtrain, Ytrain(:,1:targetSize), ...
    'DataAugmentation',augmenter);

% load and modify pre-trained CNN for regression of MTS target values.
net = resnet18();
% net = resnet50();
netDepth = length(net.Layers);
inputSize = net.Layers(1).InputSize;
lgraph = layerGraph(net);

newFClayer = fullyConnectedLayer(targetSize,...
    'Name','fc5',...
    'WeightLearnRateFactor',10,...
    'BiasLearnRateFactor',10);

newRegressionLayer = regressionLayer('Name','regression');
    
lgraph = replaceLayer(lgraph,'fc1000',newFClayer);

lgraph = removeLayers(lgraph,'prob');                              %resnet18
% lgraph = removeLayers(lgraph,'fc1000_softmax');                    %resnet50
lgraph = removeLayers(lgraph,'ClassificationLayer_predictions');   %resnet18
% lgraph = removeLayers(lgraph,'ClassificationLayer_fc1000');        %resnet50
lgraph = addLayers(lgraph,newRegressionLayer);
lgraph = connectLayers(lgraph,'fc5','regression');

% set training options and train CNN
options = trainingOptions('sgdm',...
    'InitialLearnRate',0.0001,...
    'MiniBatchSize', 64,...
    'MaxEpochs', 50, ...
    'Shuffle','every-epoch',...
    'ValidationData',{Xval,Yval(:,1:targetSize)},...
    'Plots','training-progress',...
    'LearnRateSchedule','piecewise',...
    'LearnRateDropPeriod',20,...
    'LearnRateDropFactor',0.1,...
    'ExecutionEnvironment','gpu');

%MTSnet = trainNetwork(Xtrain,Ytrain,lgraph,options);
MTSnet = trainNetwork(augmentedTrainData,lgraph,options);



    
    

