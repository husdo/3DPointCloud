function Pitch = LidarDataDisplay(DataFile,Params)

if (nargin<2)
	Params.HorizontalStep = 1;
	Params.VerticalStep = 1;
	Params.MinimalDistance = 0.5;
	Params.MaximalDistance = 20;
	Params.DistanceFromAxis = 0.075;
	%Horizontal selection (in degree)
	Params.StartAngleHor = -70;
	Params.FinishAngleHor = 70;

	%vertical selection 0 z axis (in degree)
	Params.StartAngleVer = 30;
	Params.FinishAngleVer = 180;
end
CameraData = [];
if(strcmp(DataFile(end-2:end),'bag'))
	[IMUData, LidarData, CameraData] = bagRead(DataFile);
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
% if(numel(CameraData)>0);
% 	figure(2);
% 	imshow(CameraData.MiddlePicture);
% end

PointCloudHandle = figure(1);

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
	[~, ImuIdx] = min(abs(orientationTimeStamp - LidarData.TimeStamp(i)));
	
% 	if(numel(CameraData)>0);
% 		[~, CameraIdx] = min(abs(CameraData.TimeStamp - LidarData.TimeStamp(i)));
% 		figure(3);
% 		img = zeros(1,640,3);
% 		img(:,:,:) = CameraData.Slices(CameraIdx,:,:,:);
% 		imshow(img);
% 	end
	
	
	if(orientation)
		EulerAngles = SpinCalc('QtoEA321',orientationMatrix(ImuIdx,:),1,1);
		pitch = deg2rad(EulerAngles(2)+90);
		if(pitch>2*pi)
			pitch = pitch - 2*pi;
		end
	else
		pitch = IMUData.Angle(ImuIdx,1);
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
	%Colours_tmp = z_tmp';
	select = x_tmp~=0 | y_tmp~=0 | z_tmp~=0;
	x_tmp = x_tmp(select);	
	y_tmp = y_tmp(select);
	z_tmp = z_tmp(select);
	d = d(select);
	Colours_tmp = Colours_tmp(select,:);
 	x = [x x_tmp];
 	y = [y y_tmp];
 	z = [z z_tmp];
	Colours = [Colours; Colours_tmp];
	
	refreshRate = 5;
	
	if(mod(i,refreshRate) == 0)
		showPointCloud([x(:),y(:),z(:),],Colours);
		
		xlim([-3 15])
		ylim([-15 15])
		zlim([-2 8])
		if(i<700)
			view(-30,5);
		elseif (i<900)
			view(-1*((70-30)/200*(i-700)+30),5);
		else
			view(-70,5)
		end
		if (i == refreshRate)
			pause;
		end

		drawnow;
		colormap(PointCloudHandle,'jet');
		%saveas(PointCloudHandle,['video',num2str(i),'.jpg']);
	end
	
end
	
L = [x' y' z'];
showPointCloud([x(:),y(:),z(:)]);
%plot3k(L, 'ColorData', Colours(:,1));
axis equal