function [IMUData, LidarData] = bagRead(fileName, saveFile)

if(nargin<2)
	saveFile = 0;
end

bag = rosbag(fileName);

bagselectIMU = select(bag, 'Topic', '/mavros/imu/data');
bagselectLidar = select(bag, 'Topic', '/most_intense');

IMUCell = readMessages(bagselectIMU);
LidarCell = readMessages(bagselectLidar);

LidarData = [];
LidarData.Ranges = zeros(numel(LidarCell),numel(LidarCell{1}.Ranges));

for LidarIterator = 1:numel(LidarCell)
	LidarData.Ranges(LidarIterator,:) = LidarCell{LidarIterator}.Ranges';
	LidarData.Intensities(LidarIterator,:) = LidarCell{LidarIterator}.Intensities';
end
LidarData.TimeStamp = bagselectLidar.MessageList.Time;
LidarData.AngleMax = LidarCell{1}.AngleMax;
LidarData.AngleMin = LidarCell{1}.AngleMin;
LidarData.AngleIncrement = LidarCell{1}.AngleIncrement;
LidarData.RangeMin = LidarCell{1}.RangeMin;
LidarData.RangeMax = LidarCell{1}.RangeMax;
	
for IMUIterator = 1:numel(IMUCell)
	IMUData.Orientation(IMUIterator,:) = [IMUCell{IMUIterator}.Orientation.X IMUCell{IMUIterator}.Orientation.Y IMUCell{IMUIterator}.Orientation.Z IMUCell{IMUIterator}.Orientation.W];
	IMUData.LinearAcceleration(IMUIterator,:) = [IMUCell{IMUIterator}.LinearAcceleration.X IMUCell{IMUIterator}.LinearAcceleration.Y IMUCell{IMUIterator}.LinearAcceleration.Z];
	IMUData.AngularVelocity(IMUIterator,:) = [IMUCell{IMUIterator}.AngularVelocity.X IMUCell{IMUIterator}.AngularVelocity.Y IMUCell{IMUIterator}.AngularVelocity.Z];
end
IMUData.TimeStamp = bagselectIMU.MessageList.Time;

if(saveFile)
	save(fileName(1:end-4),'IMUData','LidarData');
end
