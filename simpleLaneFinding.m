clc; 
clear; 
close all; 

videoName = 'My Movie 7.mp4';
videoReader = VideoReader(videoName);
figure;

while hasFrame(videoReader)
    I = readFrame(videoReader);
    I = imresize(I, [480, 640]);
    setpoint = [350 350];
    position = SimpleLaneDetection(I);
    error = setpoint - position; 
    display(error);
    imshow(I);
    
    hold on;
    plot(position(1), position(2), 'X', 'LineWidth', 10, 'Color', 'white');
    hold off;

    pause(0.1);
end

function position = SimpleLaneDetection(I)
    leftLaneBoundary = [];
    rightLaneBoundary = [];
    imshow(I);
    hold on; 
    Ig = rgb2gray(I);
    Min = ordfilt2(Ig, 1, ones(2));
    Max = ordfilt2(Ig, 4, ones(2));
    Edge = Max - Min; 
    BW = imbinarize(Edge);
    Height = size(Ig,1);
    BW(1:Height/2,:) = 0;
    [H,T,R] = hough(BW);
    P = houghpeaks(H,20,'threshold',1);
    lines = houghlines(BW,T,R,P,'FillGap',200,'MinLength',150);
    
    for i = 1:length(lines)
        if (lines(i).theta >= 30 && lines(i).theta <= 70)
            xy = [lines(i).point1; lines(i).point2];
            plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
            text(xy(1,1),xy(1,2),num2str(lines(i).theta),'Color','Red','FontSize',28);
            leftLaneBoundary = [leftLaneBoundary lines(i)];
        end
        if (lines(i).theta >= -70 && lines(i).theta <= -30)
            xy = [lines(i).point1; lines(i).point2];
            plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','blue');
            text(xy(1,1),xy(1,2),num2str(lines(i).theta),'Color','Red','FontSize',28);
            rightLaneBoundary = [rightLaneBoundary lines(i)];
        end
    end
    
    % Check if rightLaneBoundary is not empty before accessing its elements
    if ~isempty(rightLaneBoundary)
        right = rightLaneBoundary(1);
        midLineRight = (right.point1+right.point2)/2;
    else
        midLineRight = [0, 0];
    end
    
    % This logic wrongly assumes you will always have a right lane boundary
    % and a left lane boundary 
    left = leftLaneBoundary(1);
    midLineLeft = (left.point1+left.point2)/2;
    midLane = (midLineLeft+midLineRight)/2;
    position = midLane; 
    plot(midLane(1), midLane(2),'X','LineWidth',10,'Color','white');
end
