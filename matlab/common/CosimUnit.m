% Definition of class CosimUnit
% Sub-system standard interface for co-simulation
% This class is abstract. It cannot be instantiated.
% It is meant to be used as base class for those classes that describe
% actual implementations of co-simulation units.

% The class derives from handle (so we can modify its contents) and from
% matlab.mixin.Heterogeneous, so that it is possible to have an array of
% CosimUnits that represent actually different models

classdef (Abstract) CosimUnit < handle & matlab.mixin.Heterogeneous
    
    % __________________________________________________________ PROPERTIES
    properties
        name;           % Unit 
        storeEvery;     % Save results every X simulation steps
        tEnd;           % Final time
    end
    
    properties (Abstract)
        Y_;             % Exchanged variables at the interface
    end
    
    % _____________________________________________________________ METHODS
    methods
        
        % _____________________________________________ Default constructor
        function obj = CosimUnit(name)
            obj.name        = name;
    
            % Store data every x steps - Default: store every point
            obj.storeEvery  = 1;
        end
        
        % _______________________________________________________ Accessors
        
        % Establishes the final simulation time
        function setEndTime(obj,t); obj.tEnd = t; end

        % Sets the number of step-sizes before a store event
        function setStoreEvery(b, dt); b.storeEvery = dt; end
    end
    
    % ___________________________________________________ METHODS, ABSTRACT
    %
    % These methods are user-defined, must be implemented in the classes
    % that inherit from CosimUnit.
    %
    methods (Abstract)
        
        % Co-simulation procedures
        doStep(obj, finalT);      
        initialize(obj, t);       
        terminate(obj, t);        
        y   = readOutputs(obj, init);
        setInputs(obj, u);
        
    end    
end