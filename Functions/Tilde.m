function Tx = Tilde(X)
%#codegen
%
% Copyright 2024 The MathWorks, Inc.

    Tx = [ 0    -X(3)  X(2);
          X(3)   0    -X(1);
         -X(2)  X(1)   0];
end