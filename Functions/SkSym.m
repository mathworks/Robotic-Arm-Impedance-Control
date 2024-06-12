function Asx = SkSym(X)
%#codegen
% skew-symmetric matrix
%
% Copyright 2024 The MathWorks, Inc.

    Asx = 0.5*(X - transpose(X));
end