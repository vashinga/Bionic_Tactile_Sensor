function [cleanedData] = RmMultiObs(data,Th)

% Data should be Binary [0 or 1] and a column vector
% If there are multiple 1s within the Th datapoints, we only keep the first

onesIdx = find(data == 1);

if ~isempty(onesIdx)
    % Create groups: a new group starts when the gap is >= 10.
    % (If diff < Th, the ones belong to the same group.)
    groups = cumsum([1; diff(onesIdx) >= Th]);
    
    % For each group, pick the first (smallest) index.
    keepIdx = accumarray(groups, onesIdx, [], @min);
    
    % Build a cleaned data vector (all zeros except at the kept ones).
    cleanedData = zeros(size(data));
    cleanedData(keepIdx) = 1;
else
    cleanedData = data;
end
