function datasetStruct = load_data(select_dataset)

datasetStruct = {};

cd '/home/isorrentino/dev/Datasets/YarpRobotLogger';
[file,path] = uigetfile('*.mat','Select dataset for identification','MultiSelect', 'on');

if ~iscell(file)
    matToLoad = [path, file];
    datasetStruct{1} = load(matToLoad);
else
    for dataset_j = 1 : size(file,2)
      % Load dataset to process
      matToLoad = [path, file{dataset_j}];
      datasetStruct{dataset_j} = load(matToLoad);
    end
end

cd '/home/isorrentino/dev/iCub-torque-control/matlab';

end