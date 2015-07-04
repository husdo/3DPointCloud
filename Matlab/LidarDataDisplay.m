function Pitch = LidarDataDisplay(DataFile,Params)

if (nargin<2)
	Params.HorizontalStep = 1;
	Params.VerticalStep = 1;
	Params.MinimalDistance = 0.4;
	Params.MaximalDistance = 25;
	Params.DistanceFromAxis = 0.075;
	%Horizontal selection (in degree)
	Params.StartAngleHor = -20;
	Params.FinishAngleHor = 20;

	%vertical selection 0 z axis (in degree)
	Params.StartAngleVer = 0;
	Params.FinishAngleVer = 180;
end

if(strcmp(DataFile(end-2:end),'bag'))
	[IMUData, LidarData] = bagRead(DataFile);
elseif(strcmp(DataFile(end-2:end),'mat'))
	load(DataFile);
else
	[IMUData, LidarData] = AndroidReader(DataFile);
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
orientationTimeStamp = IMUData.TimeStamp;

if(isfield(IMUData,'Orientation'));
	orientationMatrix = IMUData.LinearAcceleration;
	orientationMatrix = IMUData.Orientation;
	orientation = 1;
else
	orientation = 0;
end

for i = 1:Params.VerticalStep:size(LidarData.Ranges,1)
	i/size(LidarData.Ranges,1) *100
	[~, idx] = min(abs(orientationTimeStamp - LidarData.TimeStamp(i)));
	
	if(orientation)
		scalar = [10 0 0]...
			*[orientationMatrix(idx,1) orientationMatrix(idx,2) orientationMatrix(idx,3)]';
		norm_base = norm([10 0 0]);
		norm_current = norm([orientationMatrix(idx,1) orientationMatrix(idx,2) orientationMatrix(idx,3)]);
		pitch = acos(scalar/(norm_base*norm_current));
		%pitch = atan2(orientationMatrix(idx,3),orientationMatrix(idx,1));
	else
		pitch = IMUData.Angle(idx,1);
	end
	
	Pitch(i) = pitch;
	
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
	x_tmp = distance.*cos(yaw)*sin(pitch) + Params.DistanceFromAxis *sin(pitch);
	y_tmp = distance.*sin(yaw);%*sin(pitch);
	z_tmp = distance.*cos(yaw)*cos(pitch)+ Params.DistanceFromAxis *cos(pitch);
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