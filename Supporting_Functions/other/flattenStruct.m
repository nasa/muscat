function out = flattenStruct(in)
    % Suppress the "Calling STRUCT on an object" warning locally
    warnState = warning('off', 'MATLAB:structOnObject');
    cleanup = onCleanup(@() warning(warnState)); % Restores warning state when function ends

    % 1. Handle Basic Types (Direct Return)
    if isnumeric(in) || ischar(in) || isstring(in) || islogical(in) || ...
       isdatetime(in) || isduration(in) || isenum(in)
        out = in;
        return;
    end

    % 2. Handle Cell Arrays
    if iscell(in)
        out = cell(size(in));
        for i = 1:numel(in)
            out{i} = flattenStruct(in{i});
        end
        return;
    end

    % 3. Handle Objects/Handles (The Core Problem)
    if isobject(in) && ~isstruct(in)
        try
            % Try converting to struct to get all public properties
            out = struct(in);
        catch
            % If the object prohibits struct conversion, we try to 
            % manually extract properties via metaclass
            mc = metaclass(in);
            propNames = {mc.PropertyList.Name};
            out = struct();
            for p = 1:numel(propNames)
                try
                    out.(propNames{p}) = in.(propNames{p});
                catch
                    % Skip properties that are inaccessible
                end
            end
        end
        % Recursively flatten the resulting struct in case properties are also objects
        out = flattenStruct(out);
        return;
    end

    % 4. Handle Structs (and Struct Arrays)
    if isstruct(in)
        fields = fieldnames(in);
        if isempty(in)
            out = in;
            return;
        end
        % Pre-allocate
        out(size(in,1), size(in,2)) = struct(); 
        for i = 1:numel(in)
            for f = 1:numel(fields)
                out(i).(fields{f}) = flattenStruct(in(i).(fields{f}));
            end
        end
    else
        out = in;
    end
end