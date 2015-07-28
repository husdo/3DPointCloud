angle = importdata('labtest2.txt');
lidar = importdata('lab_test_2.csv');
%angle = importdata('forum2.txt');
%lidar = importdata('forum2.csv');
% angle = importdata('humantest1.txt');
% lidar = importdata('human_test_1.csv');

lidarData = lidar.data(:,2:6:end);
androidPitch = angle.data(:,2) + pi/2;

lidarZeroPoint = 88;
lidarTimeStamp = lidar.data(:,1);
lidarTimeStamp = lidarTimeStamp(lidarZeroPoint:end) - lidarTimeStamp(lidarZeroPoint);
lidarData = lidarData(lidarZeroPoint:end,:);

lidarData = lidarData(1:min(end,600),1:1:end);
%getting rid of not valid measurements
lidarData(lidarData>12000) = 0;
lidarData(lidarData<400) = 0;

%furthest point
maxDistance = max(max((lidarData)));
androidZeroPoint = 144;
androidTimeStamp = angle.data(:,1);
androidTimeStamp = androidTimeStamp(androidZeroPoint:end) - androidTimeStamp(androidZeroPoint);
androidPitch = -1*androidPitch(androidZeroPoint:end);

%yaw = linspace(-120,120,size(lidarData,2));
 yaw = linspace(-90,90,size(lidarData,2));
yaw = yaw/180*pi;

x = [];
y = [];
z = [];
Colours = [];
figure(1);
for i = 1:1:size(lidarData,1)
	i/size(lidarData,1) *100
	[~, idx] = min(abs(androidTimeStamp - lidarTimeStamp(i)));
	pitch = androidPitch(idx);
	d = lidarData(i,:);
	%redness
	red  = (d/maxDistance)';
	blue = 1 - red;
	Colours_tmp = [red, zeros(size(red)), blue];
	y_tmp = d.*cos(yaw)*sin(pitch);
	x_tmp = d.*sin(yaw);
	z_tmp = d.*cos(yaw)*cos(pitch) + 1650;
		
	%scatter3(x_tmp,y_tmp,z_tmp,1, Colours);
	%plot3(x_tmp,y_tmp,z_tmp,'.');
	%axis equal
	%hold on;
 	x = [x x_tmp];
 	y = [y y_tmp];
 	z = [z z_tmp];
	Colours = [Colours; z_tmp'];
end
	
L = [x' y' z'];
plot3k(L, 'ColorData', Colours(:,1));
axis equal