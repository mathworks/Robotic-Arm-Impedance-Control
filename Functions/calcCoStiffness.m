function Gx = calcCoStiffness (Kx)
%#codegen
% Co-stiffness, denoted as Gx, is a concept used in impedance control to
% incorporate additional damping into the control scheme based on the
% stiffness matrix Kx of the robotic arm. This term helps improve the stability and
% performance of the control system, especially in scenarios where high
% stiffness values alone may lead to instability or undesirable behavior.
% 
% The equation used for G is : Gx = 0.5*trace(Kx)*eye(3) - Kx; 
% Let's break down this equation: 
% - trace(Kx) represents the sum of the diagonal elements of the stiffness matrix Kx. 
% It gives us a measure of the overall stiffness of the system across all degrees of freedom. 
% - eye(3) is the identity matrix of size 3x3. 
% Multiplying by trace(Kx) effectively scales the identity matrix by the 
% overall stiffness of the system. 
% - 0.5*trace(Kx)*eye(3) computes a scaled version of the identity matrix based on the stiffness.
% Subtracting Kx from this scaled identity matrix effectively reduces the
% stiffness along each degree of freedom, introducing damping effects
% proportional to the stiffness.
% 
% In essence, Gx modifies the stiffness matrix Kx to introduce damping
% effects that are proportional to the overall stiffness of the system.
% This helps to balance the stiffness-damping trade-off and improve the
% stability and performance of the impedance control system. In practical
% terms, incorporating co-stiffness into impedance control can enhance the
% behavior of robotic arms, allowing for more precise and stable
% interaction with the environment while mitigating the risk of
% oscillations or instability that may arise from high stiffness values
% alone.
%
% Copyright 2024 The MathWorks, Inc.

    Gx = 0.5*trace(Kx)*eye(3) - Kx;
end