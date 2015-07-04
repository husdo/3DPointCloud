clc
clear all
close all
%% Please select the LIDAR FOV and resolution --> These values must be known from the author of the test
LIDAR_resolution=0.25; % [degree]
Max_STEP=90; % [degree]
Min_STEP=-90; % [degree]

Angleswrtboresight=(Min_STEP:LIDAR_resolution:Max_STEP)';  % [degree]

%% Please select the filesto import
% CSV range and intensity file
dataR_I=importdata('lab_test_2.csv',',',1); % the first row of the file must not be imported
LIDAR_timestamp=dataR_I.data(:,1); % [ms]
Range=dataR_I.data(:,2:6:end)./1000; % Range in [m]
Intensity=dataR_I.data(:,3:6:end); % Intensity of the backscattered radiation

% Attitude file
Euler_angles=importdata('lab_test_2.txt',' ',1); % [radians]; the first row of the file must not be imported
Attitude_timestamp=Euler_angles.data(:,1); % [ms]
Pitch=Euler_angles.data(:,2); % [radians]
Roll=Euler_angles.data(:,3).*0; % [radians]
Yaw=Euler_angles.data(:,4).*0; % [radians]


%% Timestamp Syncronization
% LIDAR_timestampN=LIDAR_timestamp-LIDAR_timestamp(1);
% Attitude_timestampN=Attitude_timestamp-Attitude_timestamp(1);

% % check LIDAR_timestamp
% iL=find(diff(LIDAR_timestamp)>200);
RTS=1; % [m]; Threshold for the range syncronization
ATS=1.5; % [degree]; Threshold for the Attitude syncronization


ideltaRange=find((diff(Range(:,Angleswrtboresight==0)))>RTS);
ideltaAtt=find(abs(diff(Pitch.*180/pi))>ATS);
ideltaRange=ideltaRange(1);
ideltaAtt=ideltaAtt(1);
LIDAR_timestampN=LIDAR_timestamp(ideltaRange+1:end)-LIDAR_timestamp(ideltaRange+1);
Attitude_timestampN=Attitude_timestamp(ideltaAtt+1:end)-Attitude_timestamp(ideltaAtt+1);

RangeN=Range(ideltaRange+1:end,:);
PitchN=Pitch(ideltaAtt+1:end);

PitchNN=zeros(size(RangeN,1),1);
for i=1:length(PitchNN)
i
    ind=find(Attitude_timestampN==LIDAR_timestampN(i));
    if size(ind,1)==0
        indup=find(Attitude_timestampN>LIDAR_timestampN(i));
        inddown=find(Attitude_timestampN<LIDAR_timestampN(i));
        if size(indup,1)==0
           PitchNN(i)=PitchNN(end);
        else
            indup=indup(1);
            inddown=inddown(end);            
            PitchNN(i)=interp1([Attitude_timestampN(inddown) Attitude_timestampN(indup)],PitchN([inddown indup]),LIDAR_timestampN(i));
        end
    else if size(ind,1)==1       
        PitchNN(i)=PitchN(ind);
        else
            PitchNN(i)=mean(PitchN(ind));            
        end
    end
end

% PC_SRF=zeros(size(RangeN,1).*length(Angleswrtboresight),3);
PC_NED=zeros(0,3);
for i=1:5:length(LIDAR_timestampN)
    i
    PC_appo=PCNEDcomp(RangeN(i,:),Angleswrtboresight,PitchNN(i),[0 0 0],0,0,0);
    PC_NED=[PC_NED;PC_appo];
    
end
size(PC_NED)
RangePC=sqrt(PC_NED(:,1).^2+PC_NED(:,2).^2+PC_NED(:,3).^2);
PC_NED((RangePC>10 | RangePC<0.4),:)=[];
size(PC_NED)


figure
set(gcf,'Color','white')
plot3(PC_NED(:,1),PC_NED(:,2),PC_NED(:,3),'.r')
set(gca,'Zdir','Reverse')
set(gca,'Ydir','Reverse')
xlabel('N (m)','Fontsize',12)
ylabel('E (m)','Fontsize',12)
zlabel('D (m)','Fontsize',12)
axis equal


