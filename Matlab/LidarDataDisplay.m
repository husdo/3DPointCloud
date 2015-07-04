function LidarDataDisplay(DataFile,Params)

if (nargin<2)
	Params.HorizontalStep = 1;
	Params.VerticalStep = 1;
	Params.MinimalDistance = 0.4;
	Params.MaximalDistance = 25;

	%Horizontal selection (in degree)
	Params.StartAngleHor = -120;
	Params.FinishAngleHor = 120;

	%vertical selection 0 z axis (in degree)
	Params.StartAngleVer = 0;
	Params.FinishAngleVer = 180;
end

if(strcmp(DataFile(end-3:end),'bag'))
	[IMUData, LidarData] = bagRead(DataFile);
else
	load(DataFile);
end

LidarData.Ranges(LidarData.Ranges>Params.MaximalDistance) = 0;
LidarData.Ranges(LidarData.Ranges<Params.MinimalDistance) = 0;

%furthest point
maxDistance = max(max((LidarData.Ranges)));
yaw = LidarData.AngleMin:LidarData.AngleIncrement*Params.HorizontalStep:LidarData.AngleMax;
HorizontalSelection = yaw>(Params.StartAngleHor/180*pi) & yaw<(Params.FinishAngleHor/180*pi);
yaw = yaw (HorizontalSelection);
LidarData.Ranges = LidarData.Ranges(:,1:Params.HorizontalStep:end);
LidarData.Ranges = LidarData.Ranges(:,HorizontalSelection);

x = [];
y = [];
z = [];
Colours = [];

figure(1);

zeroVector = [10 0 0];
orientationMatrix = IMUData.LinearAcceleration;
orientationTimeStamp = IMUData.TimeStamp;

for i = 1:Params.VerticalStep:size(LidarData.Ranges,1)
	i/size(LidarData.Ranges,1) *100
	[~, idx] = min(abs(orientationTimeStamp - LidarData.TimeStamp(i)));
	scalar = [10 0 0]...
		*[orientationMatrix(idx,1) orientationMatrix(idx,2) orientationMatrix(idx,3)]';
	norm_base = norm([10 0 0]);
	norm_current = norm([orientationMatrix(idx,1) orientationMatrix(idx,2) orientationMatrix(idx,3)]);
	pitch = acos(scalar/(norm_base*norm_current));
	a(i) = pitch;
	
%	pitch = -1*(IMUData.Orientation(idx).Y + pi/2);
	distance = LidarData.Ranges(i,:);
	if(pitch>Params.FinishAngleVer/180*pi || pitch < Params.StartAngleVer/180*pi)
		distance(:) = 0;
	end
	d(i,:) = distance;
	%redness
	red  = (distance/maxDistance)';
	blue = 1 - red;
	Colours_tmp = [red, zeros(size(red)), blue];
	x_tmp = distance.*cos(yaw)*sin(pitch);
	y_tmp = distance.*sin(yaw)*sin(pitch);
	z_tmp = distance.*cos(yaw)*cos(pitch);
	Colours_tmp = z_tmp';
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
	
L = [x' y' z'];
plot3k(L, 'ColorData', Colours(:,1));
axis equal