function [startTimes,stopTimes] = getStartStopTimes(time,logicalIdxs)
%% getStartStopTimes gets start/stop times of continuous "true" segments from a vector
%
% Syntax:
%   [startTimes,stopTimes] = getStartStopTimes(time,logicalIdxs)
%
% Inputs:
%   time - Vector of time values
%   logicalIdxs - Vector of logical indices - must be the same length as
%       time
%
% Outputs:
%   startTimes - Start time of each continuous "true" segment
%   stopTimes  - End time of each continuous "true" segment.
%       This is the time of the last "true" value.
%

inputTimeSize = size(time);
if ~any(inputTimeSize == 1)
    error('Input time must be a nx1 or 1xn vector. time is size %dx%d',inputTimeSize(1),inputTimeSize(2));
end

inputLogicalSize = size(logicalIdxs);
if ~any(inputLogicalSize == 1)
    error('Input logicalIdxs must be a nx1 or 1xn vector. logicalIdxs is size %dx%d',inputLogicalSize(1),inputLogicalSize(2));
end

timeLength = length(time);
logicalLength = length(logicalIdxs);
if timeLength ~= logicalLength
    error('Inputs time and logicalIdxs must be the same length. time is length %d and logicalIdxs is length %d',...
        timeLength,logicalLength);
end

% Enforce input to be a logical column vector
logicalIdxs = logical(logicalIdxs(:));

% Get the start/stop indices
[startIdxs,stopIdxs] = getStartStopIdxsFromLogical(logicalIdxs);

% Get the start/stop times
startTimes = time(startIdxs);
stopTimes = time(stopIdxs);