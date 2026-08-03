function [reference, controlEnabled] = fcn(t)
%#codegen

reference = zeros(4,1);
controlEnabled = false;

if t >= 2.0
    reference(1) = 0.0;
    reference(2) = 0.0;
    reference(3) = 1.0;
    reference(4) = 0.0;
    controlEnabled = true;
end
end