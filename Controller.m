classdef Controller
    properties
        ControllerName
        Environment;
    end
    methods (Abstract)
        u = GetU(X_d, X);
    end
end