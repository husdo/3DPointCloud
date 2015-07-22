function LidarLocalization(filename)

load(filename);

if (nargin<2)
	Params.HorizontalStep = 1;
	Params.VerticalStep = 2;
	Params.MinimalDistance = 0.3;
	Params.MaximalDistance = 5;
	Params.DistanceFromAxis = 0.075;
	%Horizontal selection (in degree)
	Params.StartAngleHor = -130;
	Params.FinishAngleHor = 130;

	%vertical selection 0 z axis (in degree)
	Params.StartAngleVer = 70;
	Params.FinishAngleVer = 180;
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

orientationTimeStamp = IMUData.TimeStamp;
orientationMatrix = IMUData.Orientation;

Angle = 0;
WallAnglesPrev = [];
quadrants = 1;
AbsDiffPrev = 1;
AbsAnglePrev = 0;
AbsDiff = [];

FullFigure = figure(1);

for i = 1:Params.VerticalStep:size(LidarData.Ranges,1)
% 	i/size(LidarData.Ranges,1) *100
	[~, ImuIdx] = min(abs(orientationTimeStamp - LidarData.TimeStamp(i)));

	EulerAngles = SpinCalc('QtoEA321',orientationMatrix(ImuIdx,:),1,1);
	pitch = deg2rad(EulerAngles(2)+90);
	global_yaw = deg2rad(EulerAngles(1));
	global_yaw = global_yaw - deg2rad(240);
	if(pitch>2*pi)
		pitch = pitch - 2*pi;
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
	x_tmp = distance.*cos(yaw)*sin(pitch); %+ Params.DistanceFromAxis *sin(pitch);
	y_tmp = distance.*sin(yaw);%*sin(pitch);
	z_tmp = distance.*cos(yaw)*cos(pitch)+ Params.DistanceFromAxis *cos(pitch);
	

	
	%Colours_tmp = z_tmp';
	select = distance ~= 0;
	x_tmp = x_tmp(select);	
	y_tmp = y_tmp(select);
	z_tmp = z_tmp(select);
% 	d = d(select);
	Colours_tmp = Colours_tmp(select,:);
 	x = [x x_tmp];
 	y = [y y_tmp];
 	z = [z z_tmp];
	Colours = [Colours; Colours_tmp];
	
	% rotate
	rot_x_tmp = x_tmp*cos(global_yaw) - y_tmp*sin(global_yaw);
	rot_y_tmp = x_tmp*sin(global_yaw) + y_tmp*cos(global_yaw);
	
	subplot_row = 2;
	subplot_column = 3;
	
	figure(1);
	subplot(subplot_row,subplot_column,1)
	hold off;
	plot(x_tmp,y_tmp,'k.');
	hold on;
	plot(0,0,'k.','MarkerSize',15);
	circleSegment(deg2rad(-135),deg2rad(135),'k');
	title('Raw recording');
	xlim([-5,5]);
	ylim([-5,5]);
	axis equal
	
	
	subplot(subplot_row,subplot_column,2)
	hold off;
	plot(rot_x_tmp,rot_y_tmp,'b.');
	hold on;
	plot(0,0,'k.','MarkerSize',15);
	circleSegment(global_yaw-deg2rad(135),global_yaw+deg2rad(135),'b');
	title('Rotated recording (based on compass)');
	xlim([-5,5]);
	ylim([-5,5]);
	axis equal

	res_rate = 100;
	x_img = x_tmp*res_rate;
	y_img = y_tmp*res_rate;
	
	image = zeros(round(max(max(x_img-min(x_img))+1,max(y_img-min(y_img))+1)));
	linearInd = sub2ind(size(image), round(y_img-min(y_img))+1, round(x_img-min(x_img))+1);
	image(linearInd) = 1;
	
	se1 = strel('line',3,90);
	se2 = strel('line',3,0);
	image = imdilate(image,[se1,se2],'full');
	
	subplot(subplot_row,subplot_column,3);
	image = flipdim(image,1);
	imshow(image);
	title('Dilated binary image');
	
% 	subplot(subplot_row,subplot_column,4)
	[H,T,R] = hough(image,'Theta',-90:0.1:89.5);
% 	imshow(imadjust(mat2gray(H)),'XData',T,'YData',R,...
%    'InitialMagnification','fit');
% 	axis on, axis normal;
	
	P = houghpeaks(H,7,'threshold',ceil(0.2*max(H(:))));
	
	subplot(subplot_row,subplot_column,6)
	
	hold off, imshow(image), hold on
	title('Lines detected by Hough trasform');
	lines = houghlines(image,T,R,P,'FillGap',5,'MinLength',17);
	for lineIterator=1:length(lines)
		lines(lineIterator).len = norm(lines(lineIterator).point1 - lines(lineIterator).point2);
	end
	
	%lines = hough_line_elimination(lines);
	
	max_len = 0;
	for k = 1:length(lines)
	   xy = [lines(k).point1; lines(k).point2];
	   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

	   % Plot beginnings and ends of lines
	   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
	   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

	   % Determine the endpoints of the longest line segment
	   len = norm(lines(k).point1 - lines(k).point2);
	   lines_length(k) = len;
% 	   lines(k,:,:) = xy;
	   if ( len > max_len)
		  max_len = len;
		  xy_long = xy;
	   end
	end
	
	plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','red');
	

	
% 	subplot(subplot_row,subplot_column,5)
% 	hist(rot_x_tmp,-5:.1:5);
% 	title('Rotated image histogram by x');
% 	xlim([-5,5]);
% 	subplot(subplot_row,subplot_column,6)
% 	hist(rot_y_tmp,-5:.1:5);
% 	title('Rotated image histogram by y');
% 	xlim([-5,5]);
	
	WallAngles = findWallAngles([lines.theta]);
	if(numel(WallAnglesPrev)~=0)
		AbsDiff = WallAngles(1) - WallAnglesFirst(1);
		AbsDiff(AbsDiff<0) = 90 + AbsDiff(AbsDiff<0);
		AbsDiff = mean(AbsDiff);
		if(AbsDiffPrev - AbsDiff<-50)
			quadrants = quadrants - 1;
			if(quadrants == 0)
				quadrants = 4;
			end
		elseif(AbsDiffPrev - AbsDiff>50)
			quadrants = quadrants + 1;
			if(quadrants == 5)
				quadrants = 1;
			end
		end
		
		
		AbsAngle = (quadrants-1)*90 + AbsDiff;
		angles = [AbsAnglePrev AbsAngle]
		if(AbsAngle-AbsAnglePrev>180)
			AbsAngle = AbsAnglePrev + (AbsAngle-AbsAnglePrev-360)/2;
		elseif(AbsAngle-AbsAnglePrev<-180)
			AbsAngle = AbsAnglePrev + (AbsAngle-AbsAnglePrev+360)/2;
		else
			AbsAngle = AbsAnglePrev + (AbsAngle-AbsAnglePrev)/2;
		end
		
		
		AbsAnglePrev = AbsAngle;
		AbsDiffPrev = AbsDiff;
		AngleDiff = WallAngles - WallAnglesPrev;
		AngleDiff = mean(AngleDiff);
		if(AngleDiff>90)
			AngleDiff = mod(AngleDiff,90);
		end
		if(AngleDiff>45)
			AngleDiff = 90-AngleDiff;
		end  
		if(AngleDiff<-45)
			AngleDiff = -90-AngleDiff;
		end
		if(abs(AngleDiff)>30)
			AngleDiff = 0;
		end
		
		
		AbsAngle;
		rot_x_tmp = x_tmp*cos(deg2rad(AbsAngle)) - y_tmp*sin(deg2rad(AbsAngle));
		rot_y_tmp = x_tmp*sin(deg2rad(AbsAngle)) + y_tmp*cos(deg2rad(AbsAngle));
		
		subplot(subplot_row,subplot_column,4)
		hold off;
		plot(rot_x_tmp,rot_y_tmp,'r.');
		circleSegment(deg2rad(AbsAngle-135),deg2rad(AbsAngle+135),'r');
		hold on;
		plot(0,0,'k.','MarkerSize',15);
		title('Rotated recording (based on global difference)');
		xlim([-5 5]);
		ylim([-5 5]);
		axis equal;
		
		Angle = Angle + AngleDiff
		rot_x_tmp = x_tmp*cos(deg2rad(Angle)) - y_tmp*sin(deg2rad(Angle));
		rot_y_tmp = x_tmp*sin(deg2rad(Angle)) + y_tmp*cos(deg2rad(Angle));
		
		subplot(subplot_row,subplot_column,5)
		hold off;
		plot(rot_x_tmp,rot_y_tmp,'g.');
		hold on;
		plot(0,0,'k.','MarkerSize',15);
		circleSegment(deg2rad(Angle-135),deg2rad(Angle+135),'g');
		title('Rotated recording (based on local difference)');
		xlim([-5 5]);
		ylim([-5 5]);
		axis equal;
	else
		WallAnglesFirst = WallAngles;
	end
	
	
 	WallAnglesPrev = WallAngles;
	tmp = getframe(gcf);
	imwrite(tmp.cdata, strcat('frame', num2str(i),'.jpg'));
	
	saveas(FullFigure,['video',num2str(i),'.jpg']);
	pause(.001);	
	
	
end

function WallAngles = findWallAngles(Theta)
i=1;
WallAngles = [];
Theta = sort(Theta);
while i<numel(Theta)
	diffTheta = abs(Theta - Theta(i));
	WallAngles(end+1) = mean([Theta(diffTheta<30) Theta(diffTheta>150)-180]);
	if(numel([Theta(diffTheta<30) Theta(diffTheta<150)-180])>1)
		i = 1;
	else
		i = i+1;
	end
	Theta(diffTheta<30 | diffTheta>150) = [];
end
WallAngles = unique(WallAngles);


function lines = hough_line_elimination(lines)

i=1;
while i<numel(lines)
	diffTheta = abs([lines(:).theta] - lines(i).theta);
	idxThe = find(diffTheta<30);
	diffRho = abs([lines(idxThe).rho] - lines(i).rho);
	idxRho = find(diffRho<20);
	idx = idxThe(idxRho);
	if(numel(idx)>1)
		i = 1;
	else
		i = i+1;
	end
	[~,maxIdx] = max([lines(idx).len]);
	idx(maxIdx) = [];
	lines(idx) = [];
end


function h = circleSegment(startAngle,finishAngle,color)
	nsegments = 50;
	hold on
	r = 5;
	th = startAngle:2*pi/nsegments:finishAngle;
	xunit = r * cos(th) + 0;
	yunit = r * sin(th) + 0;
	xunit = [0 xunit 0];
	yunit = [0 yunit 0];
	plot(xunit, yunit,color);
	hold off

	
	