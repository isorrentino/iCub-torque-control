function datasetStruct = load_data(select_dataset)
i = 1;
datasetStruct = {};

while (select_dataset)
  % Load dataset to process
  [file,path] = uigetfile('*.mat','Select dataset for identification');
  load([path file]);
  matToLoad = [path, file];
  datasetStruct{i} = load(matToLoad);
  answer = questdlg('Would you like to choose an additional dataset?', ...
	'Menu', ...
	'Yes please','No thank you','No thank you');
  if strcmp(answer,'No thank you')
    select_dataset = false;
  else
    i = i + 1;
  end
end
end

