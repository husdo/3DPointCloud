function Lidar3DMap_RosBag(fileName)

 load(fileName);

HorizontalStep = 1;
VerticalStep = 1;
MinimalDistance = 0.4;
MaximalDistance = 3;

%Horizontal selection (in degree)
StartAngleHor = -40;
FinishAngleHor = 40;
	
%vertical selection 0 z axis (in degree)
StartAngleVer = 0;
FinishAngleVer = 180;

LidarData.Ranges(LidarData.Ranges>MaximalDistance) = 0;
LidarData.Ranges(LidarData.Ranges<MinimalDistance) = 0;

%furthest point
maxDistance = max(max((LidarData.Ranges)));
yaw = LidarData.AngleMin:LidarData.AngleIncrement*HorizontalStep:LidarData.AngleMax;
HorizontalSelection = yaw>(StartAngleHor/180*pi) & yaw<(FinishAngleHor/180*pi);
yaw = yaw (HorizontalSelection);
LidarData.Ranges = LidarData.Ranges(:,1:HorizontalStep:end);
LidarData.Ranges = LidarData.Ranges(:,HorizontalSelection);

x = [];
y = [];
z = [];
Colours = [];

figure(1);
for i = 1:VerticalStep:size(LidarData.Ranges,1)
	i/size(LidarData.Ranges,1) *100
	[~, idx] = min(abs(IMUData.TimeStamp - LidarData.TimeStamp(i)));
% 	scalar = [-1 .25 0]...
% 		*[IMUData.Orientation(idx,1) IMUData.Orientation(idx, 2) IMUData.Orientation(idx, 3)]'; %dot product with x y z to get the angle between the vectors
% 	norm_base = norm([-1 .25 0]);
% 	norm_current = norm([IMUData.Orientation(idx,1) IMUData.Orientation(idx,2) IMUData.Orientation(idx,3)]);
% 	pitch = acos(scalar/(norm_base*norm_current));

	a(i) = pitch
	
	
	[hullam, pitch, hullam] = SpinCalc('QtoEA321', IMUData.Orientation(idx,:), 1, 1);

%	pitch = -1*(IMUData.Orientation(idx).Y + pi/2);
	distance = LidarData.Ranges(i,:);
	if(pitch>FinishAngleVer/180*pi || pitch < StartAngleVer/180*pi)
		distance(:) = 0;
	end
	d(i,:) = distance;
	%redness
	red  = (distance/maxDistance)';
	blue = 1 - red;
	Colours_tmp = [red, zeros(size(red)), blue];
	x_tmp = distance.*cos(yaw)*sin(pitch);
	y_tmp = distance.*sin(yaw)*sin(pitch);
	z_tmp = 2*distance.*cos(yaw)*cos(pitch); %TODO
	%Colours_tmp = z_tmp';
	%z_tmp = distance.*cos(pitch); %TODO
	select = x_tmp~=0 | y_tmp~=0 | z_tmp~=0;
	x_tmp = x_tmp(select);	
	y_tmp = y_tmp(select);
	z_tmp = z_tmp(select);
	Colours_tmp = Colours_tmp(select,:);
 	x = [x x_tmp];
 	y = [y y_tmp];
 	z = [z z_tmp];
	Colours = [Colours; Colours_tmp];
	
end
plot(a)
L = [x' y' z'];
plot3k(L, 'ColorData', Colours(:,1));
axis equal