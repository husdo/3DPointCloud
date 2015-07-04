function [IMUData, LidarData] = AndroidReader(Filename,savefile)
%ANDROIDREADER Summary of this function goes here
%   Detailed explanation goes here

if(nargin<2)
	savefile = 0;
end

angle = importdata([Filename, '.txt']);
lidar = importdata([Filename, '.csv']);

LidarData.Ranges = lidar.data(:,2:6:end)*0.001;

LidarData.TimeStamp = lidar.data(:,1) - lidar.data(1,1);
dataNum = size(LidarData.Ranges,2);
totalDegree = (dataNum-1)*0.0043633;
LidarData.AngleMax = totalDegree/2;
LidarData.AngleMin = -1*totalDegree/2;
LidarData.AngleIncrement = deg2rad(0.25);
LidarData.RangeMin = 0.023000000044703;
LidarData.RangeMax = 60;

IMUData.TimeStamp = angle.data(:,1) - angle.data(1,1);
IMUData.Angle = angle.data(:,2:end)+ pi/2;
if(savefile)
	save(Filename,'LidarData','IMUData');
end
end

