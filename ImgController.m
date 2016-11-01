classdef ImgController
    properties
        ControllerName
        Environment;
    end
    methods (Abstract)
        % [u] = Control Input
        % [I_d] = Desired Position in Img Space (u1,v1,u2)_d
        % [I] = Current Position in Img Space (u1,v1,u2)
        u = GetU(I_d, I);
    end
end