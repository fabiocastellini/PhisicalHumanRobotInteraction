
function [time, M_pos, M_vel, M_volt] = readTxtFile(filePath)

    fileID = fopen(filePath);

    formatSpec = '%s';
    numCols = 8;   
    textFromFile = textscan(fileID,formatSpec,numCols);
    dataFromFile = textscan(fileID,'%f %f %f %f %f %f %f %f %f %f');
    
    time  = dataFromFile{1}; 
    M_pos = dataFromFile{2}; M_vel = dataFromFile{3}; M_volt = dataFromFile{4}; 

%     time(1:292) = []; %cut samples until something happens (0.3s)
%     M_pos(1:292) = []; M_vel(1:292) = []; M_volt(1:292) = [];
    time(1:500) = []; %cut samples until something happens (0.3s)
    M_pos(1:500) = []; M_vel(1:500) = []; M_volt(1:500) = [];
    
    
end