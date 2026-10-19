function [startIdxs,stopIdxs] = getStartStopIdxsFromLogical(logicalIdxs)
%% getStartStopIdxsFromLogical gets start/stop indices of continuous "true" segments from a vector
%
% Syntax:
%   [startIdxs,stopIdxs] = getStartStopIdxsFromLogical(logicalIdxs)
%
% Inputs:
%   logicalIdxs - Vector of logical indices
%
% Outputs:
%   startIdxs - Absolute start index of each continuous "true" segment
%   stopIdxs  - Absolute ending index of each continuous "true" segment.
%       This is the index of the last "true" value before going to "false".
%

inputSize = size(logicalIdxs);
if ~any(inputSize == 1)
    error('Input must be a nx1 or 1xn vector. Input is size %dx%d',inputSize(1),inputSize(2));
end

% Enforce input to be a logical column vector
logicalIdxs = logical(logicalIdxs(:));

% Get the diff of the logicals
idxDiff = diff(logicalIdxs);

% Compute the start indices
startIdxs = find(idxDiff == 1)+1;
if logicalIdxs(1)
    % Account for the first entry being true
    startIdxs = [1; startIdxs];
end

% Compute the end indices
stopIdxs = find(idxDiff == -1);
if logicalIdxs(end)
    % Account for the last entry being true
    stopIdxs = [stopIdxs; length(logicalIdxs)];
end