classdef GravityField < handle
    
    %
    % Example for small bodies:
    %
    %   4
    %   6378.1363
    %   3.986004415E5
    %   2	0	-4.8416938905481E-04	 0.0000000000000E+00
    %   2	1	-2.0458338184745E-10	 1.3968195379551E-09
    %   2	2	 2.4393233001191E-06	-1.4002662003867E-06
    %   3	0	 9.5718508415439E-07	 0.0000000000000E+00
    %   3	1	 2.0304752656064E-06	 2.4817416903031E-07
    %   3	2	 9.0480066975068E-07	-6.1900441427103E-07
    %   3	3	 7.2128924247650E-07	 1.4143556434052E-06
    %   4	0	 5.3999143526074E-07	 0.0000000000000E+00
    %   4	1	-5.3617583789434E-07	-4.7356802287476E-07
    %   4	2	 3.5051159931087E-07	 6.6243944849186E-07
    %   4	3	 9.9085503541734E-07	-2.0097529442342E-07
    %   4	4	-1.8846750474516E-07	 3.0882228278756E-07
    %
    % Example for planetary bodies:
    %
    %   6378.1363,398600.4415,200,200
    %   0,0,0.e+00,0.e+00
    %   1,0,0.e+00,0.e+00
    %   1,1,0.e+00,0.e+00
    %   2,0,-4.8416938905481e-04,0.e+00
    
    properties (SetAccess='immutable')
        Size        % Size (i.e. maximum degree) of SH expansion
    end
    
    
    properties
        Subset      % Subset of SH coefficients eventually used for
        %  the computation (i.e. degree
        
        GM          % Gravitational Parameter [km^3/s^2]
        R           % Reference Radius [km]
        
        J           % Un-normalized Jn  harmonic coefficients
        C           % Un-normalized Cnm harmonic coefficients
        S           % Un-normalized Snm harmonic coefficients
    end
    
    
    
    
    methods
        
        
        function obj = GravityField (FileName)
            
            % If no file is provided, create a default empty object
            if isempty(FileName)
                obj.Size   = 0;
                obj.R      = 0;
                obj.GM     = 0;
                obj.Subset = 0;
                obj.J = [];
                obj.C = [];
                obj.S = [];
                return
            end
            
            %data = readtable(FileName,'Delimiter',{'tab', '+'}, MultipleDelimsAsOne, true)
            
            % Read the first line and check if it is 4 comma separated values or one value
            
            fid = fopen(FileName);
            first_line = strsplit(fgetl(fid), ',');
            if length(first_line) == 1
                % First 3 lines (small body)
                obj.Size = str2double(first_line{1});
                obj.Subset = obj.Size;
                obj.R    = str2double(fgetl(fid));
                obj.GM   = str2double(fgetl(fid));
                
                n_header_lines = 3;
            else
                % First line (planetary body)
                obj.R    = str2double(first_line{1});
                obj.GM   = str2double(first_line{2});
                obj.Size = min(str2double(first_line{3}),str2double(first_line{4}));
                obj.Subset = obj.Size;
                
                n_header_lines = (1 + 1) * 2; % (header + 0,0 coefficient) * empty line
            end
            fclose(fid);
            
            Table = readtable(FileName, 'HeaderLines', n_header_lines);
            Table.Properties.VariableNames = {'n', 'm', 'Cnorm', 'Snorm'};
            
            % Allocate J, C and S coefficient arrays
            obj.J = zeros(obj.Size, 1);
            obj.C = zeros(obj.Size, obj.Size);
            obj.S = zeros(obj.Size, obj.Size);
            
            % Store un-normalized coefficients
            for i = 1:size(Table,1)
                
                n     = Table.n(i);
                m     = Table.m(i);
                Cnorm = Table.Cnorm(i);
                Snorm = Table.Snorm(i);
                
                if m == 0
                    obj.J(n) = - GravityField.Unnormalize(Cnorm, n, 0);
                else
                    obj.C(n,m) = GravityField.Unnormalize(Cnorm, n, m);
                    obj.S(n,m) = GravityField.Unnormalize(Snorm, n, m);
                end
                
            end
            
        end
        
        
        function obj = setSubset (obj, value)
            
            if  all(value >= 0) && all(value <= obj.Size)
                obj.Subset = value;
            else
                if  value > obj.Size
                    obj.Subset = obj.Size;
                else
                    obj.Subset = 0;
                end
                warning( ['Subset must be a positive integer value no larger than the gravity field size (', num2str(obj.Size), ')'] )
            end
            
        end
        
        
        % Externally declared methods
        Load (obj, FileName)
        
        [a, V, W, dadr] = Acceleration (obj, Position, varargin)
        
    end
    
    
    
    
    methods (Access='private')
        
        % Externally declared methods
        [a, V, W] = Cunningham (obj, Position, nstart, varargin)

        [a, V, W, dadr] = Cunningham_Derivatives (obj, Position, nstart, varargin)
        
    end
    
    
    
    
    methods (Static, Access='private')
        
        
        function C = Unnormalize(Cnorm, n, m)
            
            if m==0
                k = 1;
            else
                k = 2;
            end
            
            % C = sqrt( k * (2*n+1) * factorial(n-m) / factorial(n+m) ) * Cnorm;
            C = sqrt( k * (2*n+1) * (1 / prod((n-m+1):(n+m)))) * Cnorm;
            
        end
        
    end
    
    
end

