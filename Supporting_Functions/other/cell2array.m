function array = cell2array(cell)
    array = zeros(size(cell{1},1), size(cell{1},2), numel(cell));
    for k = 1:numel(cell)
        array(:,:,k) = cell{k};
    end
end