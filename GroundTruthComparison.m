% Setting up ROS connection
rosshutdown;
rosinit;

% Subscribing to the topics
odomSub = rossubscriber('/odom');
amclSub = rossubscriber('/amcl_pose');

% Create figure for plotting
figureHandle = figure;

% Pre-define the plot handles
subplot(2, 1, 1);
h_odom = plot(NaN, NaN, 'r', 'DisplayName', 'odom'); % initialize an empty plot for odom
hold on;
h_amcl = plot(NaN, NaN, 'b', 'DisplayName', 'amcl'); % initialize an empty plot for amcl
xlabel('X');
ylabel('Y');
title('XY Positions from /odom/pose/ and /amcl_pose');
legend('-DynamicLegend'); % Automatically update legend with 'DisplayName' properties
xlim([-10, 10]); % adjust as per your expected range
ylim([-10, 10]); % adjust as per your expected range

subplot(2, 1, 2);
h_disp = plot(NaN, NaN, 'g'); % initialize an empty plot for displacement
xlabel('Time (s)');
ylabel('Displacement');
title('Displacement between /odom/pose and /amcl_pose over time');

% Setting up variables for real-time plotting
odomXY = [];
amclXY = [];
timeArray = [];
displacementArray = [];
startTime = datetime('now');

% Real-time plotting
for i = 1:100 % Continue as long as the figure is open
    % Ensure that the figure is still valid
    if ~isvalid(figureHandle)
        break;
    end
    % Fetch the latest data
    odomData = odomSub.LatestMessage;
    amclData = amclSub.LatestMessage;
   
    % Extract X, Y data
    odomX = odomData.Pose.Pose.Position.X;
    odomY = odomData.Pose.Pose.Position.Y;
    amclX = amclData.Pose.Pose.Position.X; % Assuming the amcl message structure has a similar Pose structure
    amclY = amclData.Pose.Pose.Position.Y; % Adjust if not
    
    % Compute displacement
    disp = sqrt((odomX - amclX)^2 + (odomY - amclY)^2);
    
    % Updating the arrays for plotting
    odomXY = [odomX, odomY];
    amclXY = [amclX, amclY];
    currentTime = datetime('now');
    elapsedTime = seconds(currentTime - startTime);
    timeArray = [timeArray; elapsedTime];
    displacementArray = [displacementArray; disp];
    
    % Update the plot data
    subplot(2, 1, 1);
    set(h_odom, 'XData', [get(h_odom, 'XData'), odomX], 'YData', [get(h_odom, 'YData'), odomY]);
    set(h_amcl, 'XData', [get(h_amcl, 'XData'), amclX], 'YData', [get(h_amcl, 'YData'), amclY]);

    subplot(2, 1, 2);
    set(h_disp, 'XData', timeArray, 'YData', displacementArray);

    pause(1); % Assuming 1 second interval, adjust as needed

end
