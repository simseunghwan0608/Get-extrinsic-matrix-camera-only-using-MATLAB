% % STEP 1

% % call intrinsic
data = load("/home/simseunghwan/last_intrinsic_camera1_2_6.mat")
intrinsic = data.cameraParams;
% %  
% % capture 1 image 
% camList = webcamlist;
% cam =  webcam(1);
% preview(cam);
% img_captured = snapshot(cam);
% imwrite(img_captured,'images/calibration_baselink_camera_img_1.png');
% clear cam;
% 
% % STEP 2
% 
img = imread('images/calibration_baselink_camera_img_1.png');
[imagePoints, boardSize] = detectCheckerboardPoints(img);

figure
imshow(img);
hold on; 

plot(imagePoints(:,1), imagePoints(:,2), 'ro', 'MarkerSize',5);

squareSize = 0.025; % Square size in meters (our teams default is 0.025);
worldPoints = generateCheckerboardPoints(boardSize,squareSize);
patternOriginHeight = 0.0175; % Pattern is on ground size in meter
[pitch, yaw, roll, height] = estimateMonoCameraParameters(intrinsic, ...
                            imagePoints, worldPoints, patternOriginHeight);

% STEP 3
% check out the pitch, yaw, roll, height if height is correct and
%  yaw and roll will be close at 0 and pitch will be positive, check out
%  it is degrees or radians.

% STEP 4

% Find Extrinsic matrix 

R = eul2rotm(deg2rad([yaw, pitch, roll]));

t = [0; 
     0;
     height];

T = [R,t;
     0, 0, 0, 1];

% T means pose of the camera by baselink (z is not front yet!) 

I = eye(size(T));
T_inv1 = T \ I ;

% R_test = T(1:3,1:3);
% t_test = T(1:3,4);
% 
% T_inv2 = eye(4);
% T_inv2(1:3,1:3) = R.';
% T_inv2(1:3,4) = -R.' * t;


Ry = [ 0 0 1 0;
       0 1 0 0;
       -1 0 0 0;
       0 0 0 1 ];

Rx = [ 1 0 0 0;
       0 0 1 0;
       0 -1 0 0;
       0 0 0 1];

R_conv_pose =  Rx * Ry;
R_conv_extrinsic = transpose(R_conv_pose); %computing inverse

Ext = R_conv_extrinsic * T_inv1;

