classdef ControlledSystem < handle
    properties  (Access = protected)
        model
        controller
        time
        st
        controlled_output_index
        goal_output=1;
        IAE_baseline=0.55;
        settling_time_baseline=0.8;
        CE_baseline=0.72;
        OV_baseline=2;
    end
    methods  (Access = public)
        function obj=ControlledSystem(model)
            obj.model=model;
            obj.controller=[];
            obj.time=0;
            obj.st=model.getSamplingPeriod;
        end
        
        function setController(obj,controller,controlled_output_index)
            if nargin<3
                obj.controlled_output_index=1:obj.model.getOutputNumber;
            else
                obj.controlled_output_index=controlled_output_index;
            end
            obj.controller=controller;
            assert(obj.controller.getSamplingPeriod==obj.model.getSamplingPeriod,'Controller sampling period is wrong');
            obj.controller.setUMax(obj.model.getUMax)
            
        end
        
        function initialize(obj)
            rng shuffle
            obj.model.initialize;
            if ~isempty(obj.controller)
                obj.controller.inizialize;
            end
            obj.time=0;
        end
        
        function [y,t]=openloop(obj,control_action)
            obj.model.setScenario(1);
            t=obj.time;
            obj.time=obj.time+obj.st;
            y=obj.model.computeOutput;
            obj.model.updateState(control_action,t);
        end
        
        function [y,u,t]=step(obj,reference)
            t=obj.time;
            obj.time=obj.time+obj.st;
            y=obj.model.computeOutput;
            assert(~isempty(obj.controller),'Controller is not set');
            u=obj.controller.computeControlAction(reference,y(obj.controlled_output_index));
            obj.model.updateState(u,t);
        end
        
        function st=getSamplingPeriod(obj)
            st=obj.st;
        end
        
        function [score,results]=evalution(obj)
            
            figure(100)
            subplot(2,1,1)
            hold off
            xlabel('Time');
            ylabel('Output');
            grid on
            subplot(2,1,2)
            hold off
            xlabel('Time');
            ylabel('Torque');
            grid on
            for is=1:20
                [IAE(is),OV(is),CE(is),settling_time(is),results(is)]=simulation(obj,is+1);
                subplot(2,1,1)
                plot(results(is).t,results(is).y(:,obj.goal_output),'k');
                hold on
                subplot(2,1,2)
                plot(results(is).t,results(is).u,'k');
                hold on
            end
            subplot(2,1,1)
            hold off
            subplot(2,1,2)
            hold off
            IAE_max=max(IAE);
            OV_max=max(OV);
            CE_max=max(CE);
            settling_time_max=max(settling_time);
            score=obj.computeScore(IAE_max,OV_max,CE_max,settling_time_max);
        end
        
        function [IAE,OV,CE,settling_time,result]=simulation(obj,scenario)
            obj.initialize
            obj.model.setScenario(scenario);
            
            
            t=(0:obj.st:20)';
            R1=3;
            R2=1;
            t2=10;
            
            reference=R1+(R2-R1)*(t>t2);
            y=zeros(length(t),obj.model.getOutputNumber);
            u=zeros(length(t),obj.model.getInputNumber);
            for idx=1:length(t)
                [y(idx,:),u(idx,1),t(idx,1)]=obj.step(reference(idx));
            end
            
            y_goal=y(:,obj.goal_output);
            errore=reference-y_goal;
            
            idxs_IAE=find((t>obj.settling_time_baseline).*(t<t2)+(t>obj.settling_time_baseline+t2));
            IAE=sum(abs(errore(idxs_IAE))*obj.st);
            
            OV1=max(max(y_goal(t<3)-R1)/R1,0)*100;
            delta_y2=R1-y_goal(find((t>t2).*(t<t2+3)));
            OV2=max(max(delta_y2/abs(R2-R1))-1,0)*100;
            OV=max(OV1,OV2);
            CE=sum(abs(diff(u))*obj.st);
            
            settling_time1=t(find(abs(errore(t<3))>0.02*R1,1','last'));
            settling_time2=t(find(abs(errore(find((t>t2).*(t<(t2+3)))))>0.02*abs(R1-R2),1','last'));
            if isempty(settling_time1)
                settling_time1=inf;
            end
            if isempty(settling_time2)
                settling_time2=inf;
            end
            settling_time=max(settling_time1,settling_time2);
            
            result.t=t;
            result.y=y;
            result.u=u;
            result.reference=reference;
            result.CE=CE;
            result.OV=OV;
            result.IAE=IAE;
            result.settling_time=settling_time;

        end
    end
    
    methods  (Access = protected)
        function score=computeScore(obj,IAE,OV,CE,settling_time)
            penality_IAE=(IAE/obj.IAE_baseline-1);
            penality_CE=(CE/obj.CE_baseline-1);
            score=30-5*min(penality_IAE,3)-10*min(penality_CE,3);
            if (OV>obj.OV_baseline)
                score=score-0.5*(OV-obj.OV_baseline);
            end
            if (settling_time>obj.settling_time_baseline)
                score=score-5*min(settling_time/obj.settling_time_baseline-1,3);
            end
        end
    end
        
end