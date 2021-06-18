classdef BaseController < handle
    properties  (Access = protected)
        st      % Tempo di campionamento
        umax    % Saturazione azione di controllo
    end
    methods
        function obj=BaseController(st)
            obj.st=st;
        end
        function inizialize(obj)
        end
        function setUMax(obj,umax)
            obj.umax=umax;
        end
        function u=computeControlAction(obj,reference,y)
            u=reference-y;
        end
        function st=getSamplingPeriod(obj)
            st=obj.st;
        end
    end
end