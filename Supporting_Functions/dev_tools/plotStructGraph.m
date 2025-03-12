function plotStructGraph(s)
    % Initialize variables
    nodeNames = {};
    edges = [];

    % Recursive function to traverse the struct and record nodes and edges
    function traverseStruct(s, parentName)
        fields = fieldnames(s);
        for i = 1:numel(fields)
            currentField = fields{i};
            currentName = strcat(parentName, '.', currentField);
            
            % Add current node to the nodeNames list
            nodeNames{end+1} = currentName;
            if ~isempty(parentName)
                edges = [edges; {parentName, currentName}];
            end

            % Recurse into the struct if the field is a struct itself
            if isstruct(s.(currentField))
                traverseStruct(s.(currentField), currentName);
            end
        end
    end

    % Start traversal with the top-level struct
    traverseStruct(s, 'root');

    % Remove the 'root.' prefix from the node names for clarity
    nodeNames = cellfun(@(x) strrep(x, 'root.', 's.'), nodeNames, 'UniformOutput', false);

    % Remove the 'root.' prefix from the node names for clarity
    edges = cellfun(@(x) strrep(x, 'root', 's'), edges, 'UniformOutput', false);

    % Create a digraph from the edges
    G = digraph(edges(:,1), edges(:,2));

    % Plot the graph
    figure;
    plot(G);
    %     plot(G, 'NodeLabel', nodeNames, 'Layout', 'layered', 'Interpreter', 'none');
    title('Struct Graph');
end
