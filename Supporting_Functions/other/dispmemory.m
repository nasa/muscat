function dispmemory()
    % Get the current date and time
    currentTime = datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss');

    % Get process ID
    pid = feature('getpid');
    
    % Get the operating system
    if ispc
        % Windows
        [~, cmdout] = system(sprintf('tasklist /FI "PID eq %d" /FO LIST | find "Mem Usage"', pid));
        % Extract the number and convert from KB to GB
        memStr = regexp(cmdout, '\d+', 'match');
        if ~isempty(memStr)
            matlabMemUsedGB = str2double(memStr{1}) / (1024^2); % Convert KB to GB
        else
            matlabMemUsedGB = 0;
        end
    else
        % macOS or Linux
        if ismac
            % macOS using ps command
            [~, cmdout] = system(sprintf('ps -o rss= -p %d', pid));
        else
            % Linux using ps command
            [~, cmdout] = system(sprintf('ps -o rss= -p %d', pid));
        end
        % Convert KB to GB
        matlabMemUsedGB = str2double(strtrim(cmdout)) / (1024^2);
    end

    % Get the Java heap memory (as additional info)
    runtime = java.lang.Runtime.getRuntime;
    javaMemUsedBytes = runtime.totalMemory() - runtime.freeMemory();
    javaMemUsedGB = javaMemUsedBytes / (1024^3);

    % Format the output string
    memStr = sprintf('%.2f GB (Process) + %.2f GB (Java Heap) - %s', ...
        matlabMemUsedGB, javaMemUsedGB, char(currentTime));
    
    % Display the memory usage
    disp(memStr);
end
