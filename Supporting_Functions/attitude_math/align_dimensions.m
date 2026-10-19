function [A_out, B_out, L] = align_dimensions(A, B, dim, nc_A, nc_B)
% ALIGN_DIMENSIONS Permutes two 2D arrays to align their common dimension.
%
%   [A_out, B_out, L] = ALIGN_DIMENSIONS(A, B) determines which dimension length
%   is shared between arrays A and B. It then permutes (transposes) the arrays
%   so that the common length sits along the first dimension (rows). Returns
%   the common length L.
%
%   [A_out, B_out, L] = ALIGN_DIMENSIONS(A, B, DIM, NC_A, NC_B) allows explicit 
%   specification of the dimensions NOT to align (Non-Common dimensions).
%   
%   INPUTS:
%       A     - First 2D numeric or cell array.
%       B     - Second 2D numeric or cell array.
%       dim   - (Optional) The target dimension for the common length (1 or 2).
%               Default is 1.
%       nc_A  - (Optional) The size of the dimension in A *not* to use for
%               alignment. Used to disambiguate or force alignment on the 
%               other dimension.
%       nc_B  - (Optional) The size of the dimension in B *not* to use for
%               alignment.
%
%   OUTPUTS:
%       A_out - The first array, potentially transposed.
%       B_out - The second array, potentially transposed.
%       L     - The length of the common dimension used for alignment.
%
%   EXAMPLE:
%       A = zeros(3, 10); B = zeros(10, 3);
%       % Align matching dimension (3) along rows (dim 1).
%       % We specify that 10 is the non-common dimension for both.
%       [A_new, B_new, len] = align_dimensions(A, B, 1, 10, 10);
%       % A_new is 3x10. B_new is 3x10. len is 3.

    % 1. Input Handling
    if nargin < 3 || isempty(dim)
        dim = 1; % Default to aligning along rows
    end
    
    if isscalar(A) && isscalar(B)
        A_out = A;
        B_out = B;
        L = 1; % Common dimension for two scalars is conceptually 1
        return;
    end

    % 2. Determine the Target Common Length (L)
    use_explicit_nc = (nargin >= 5);
    
    if use_explicit_nc
        % -- Logic with explicit non-common dimension constraints --
        
        % Determine L for A (if A is a matrix)
        if ~isscalar(A)
            sA = size(A);
            idx = find(sA == nc_A, 1);
            if isempty(idx)
                error('Specified non-common dimension nc_A not found in A.');
            end
            % The "other" dimension is the target L
            sA(idx) = []; 
            L_A = sA(1); 
        end
        
        % Determine L for B (if B is a matrix)
        if ~isscalar(B)
            sB = size(B);
            idx = find(sB == nc_B, 1);
            if isempty(idx)
                error('Specified non-common dimension nc_B not found in B.');
            end
            sB(idx) = [];
            L_B = sB(1);
        end
        
        % Validation and Unification
        if isscalar(A)
            L = L_B;
        elseif isscalar(B)
            L = L_A;
        else
            if L_A ~= L_B
                error('The derived common dimensions from nc_A and nc_B do not match (%d vs %d).', L_A, L_B);
            end
            L = L_A;
        end
        
    else
        % -- Logic with automatic detection (Intersection) --
        
        if isscalar(A) || isscalar(B)
            % Mixed Scalar/Matrix case: Default to largest dimension of the matrix
            if isscalar(A)
                L = max(size(B));
            else
                L = max(size(A));
            end
        else
            % Matrix/Matrix case
            sA = size(A);
            sB = size(B);
            common_lens = intersect(sA, sB);
            
            if isempty(common_lens)
                error('The input arrays do not share a common dimension length.');
            end
            
            % Prioritize largest match
            L = max(common_lens);
        end
    end

    % 3. Align Array A
    if isscalar(A)
        A_out = A;
    else
        if size(A, dim) == L
            A_out = A;
        elseif size(A, 3-dim) == L % Check the other dimension
            A_out = A.';
        else
            % L exists in A (verified above), but if A is square or symmetric logic fails?
            % If we are here, neither dim matches L? 
            % This implies L wasn't actually a dimension of A, which shouldn't happen 
            % given the derivation steps unless user error in optional args logic 
            % or complex edge cases.
            % However, if A is 10x10 and L=10, the first check passes.
            % If A is 3x4 and L=5, we error.
            error('Target dimension length %d not found in A.', L);
        end
    end

    % 4. Align Array B
    if isscalar(B)
        B_out = B;
    else
        if size(B, dim) == L
            B_out = B;
        elseif size(B, 3-dim) == L
            B_out = B.';
        else
            error('Target dimension length %d not found in B.', L);
        end
    end

end