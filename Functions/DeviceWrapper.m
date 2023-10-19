% DeviceWrapper This class is a wrapper for the robot device, providing a set of methods to interact with the device and control its behavior.
%
% The purpose of this wrapper is to abstract away the details of the robot device and
% provide a simplified interface for users. By using this wrapper, users
% can easily switch between different devices with minimal changes to their
% code. This wrapper class encapsulates the robot device instance, which is 
% defined in "DeviceInit.m", and exposes methods for common tasks such as 
% moving the device, setting limits, querying positions, and waiting 
% for motion to stop.
%
% Usage:
%   device = DeviceWrapper(deviceInstance);
%   device.MOV('x y', [2,3]);
%   position = device.WAIT();

classdef DeviceWrapper
    properties
        device
    end
    
    methods
        function obj = DeviceWrapper(deviceInstance)
            obj.device = deviceInstance;
        end
        
        function MOV(obj, axes, values)
            % Sets an absolute target position for the given axis.
            % - axes: One axis of the controller.
            % - values: The absolute target position in physical units.
            % Response: None.
            obj.device.MOV(axes, values);
        end
        
        function MVR(obj, axes, values)
            % Moves the given axis relative to the last commanded target position.
            % - axes: One axis of the controller.
            % - values: The distance that the axis is to move.
            % Response: None.
            obj.device.MVR(axes, values);
        end
        
        function SST(obj, axes, values)
            % Sets the distance ("step size") for motions of the given axis triggered by a manual control unit.
            % - axes: One axis of the controller.
            % - values: The distance, format: float.
            % Response: None.
            obj.device.SST(axes, values);
        end
        
        function PLM(obj, axes, values)
            % Limits the high end of the axis travel range in closed-loop operation ("soft limit").
            % - axes: One axis of the controller.
            % - values: The limit position for the high end of the travel range, in physical units.
            % Response: None.
            obj.device.PLM(axes, values);
        end
        
        function NLM(obj, axes, values)
            % Limits the low end of the axis travel range in closed-loop operation ("soft limit").
            % - axes: One axis of the controller.
            % - values: The limit position for the low end of the travel range, in physical units.
            % Response: None.
            obj.device.NLM(axes, values);
        end
        
        function result = qPOS(obj, axes)
            % Gets the current axis position.
            % - axes: One axis of the controller.
            % Response: The current axis position in physical units.
            result = obj.device.qPOS(axes);
        end
        
        function DRC(obj, number, axes, value)
            % Determines the data source to be used and the type of data to be recorded for the data recorder table given.
            % - number: Data recorder table of the controller.
            % - axes: ID of the data source, for example, an axis or channel of the controller.
            % - value: Type of data to be recorded (record option).
            obj.device.DRC(number, axes, value);
        end
        
        function result = qDRR(obj, number1, number2, value)
            % Gets the last recorded data.
            % - number1: First point to be read from the data recorder table.
            % - number2: Number of points to be read per table.
            % - value: Data recorder table of the controller.
            % Response: Recorded data in GCS array format.
            result = obj.device.qDRR(number1, number2, value);
        end
        
        function KEN(obj, name)
            % Activates the specified coordinate system.
            % - name: Name of the coordinate system to be activated.
            obj.device.KEN(name);
        end
        
        function KSF(obj, name)
            % Defines a KSF type operating coordinate system at the current position of the hexapod's motion platform.
            % - name: Name of the coordinate system to be defined.
            obj.device.KSF(name);
        end
        
        function VLS(obj, value)
            % Sets the velocity for the motion platform of the hexapod.
            % - value: The velocity value in physical units (mm/s).
            % Response: None.
            obj.device.VLS(value);
        end
        
        function RTR(obj, value)
            % Sets the record table rate for data recording operations.
            % - value: The record table rate to be used for recording operations (unit: number of cycles).
            % Response: None.
            obj.device.RTR(value);
        end
        
        function DIO(obj, number1, number2)
            % Switches the specified digital output line(s) to specified state(s).
            % - number1: Digital output line of the controller.
            % - number2: State of the digital output line.
            % Response: None.
            obj.device.DIO(number1, number2);
        end
        
        function SPI(obj, PPCoordinate, position)
            % Moves the pivot point out of the origin of the coordinate system.
            % - PPCoordinate: Pivot point coordinate.
            % - position: Value of the pivot point coordinate.
            % Response: None.
            obj.device.SPI(PPCoordinate, position);
        end
        
        function result = qSPI(obj, PPCoordinate)
            % Gets the pivot point coordinates.
            % - PPCoordinate: Pivot point coordinate.
            % Response: Value of the pivot point coordinate in physical units.
            result = obj.device.qSPI(PPCoordinate);
        end
        
        function result = qVMO(obj, AxisID, position)
            % Checks whether the motion platform of the hexapod can approach a specified position.
            % - AxisID: Axis of the controller.
            % - position: Target position value to be checked.
            % Response: 0 if position cannot be approached, 1 if it can be approached.
            result = obj.device.qVMO(AxisID, position);
        end
        
        function DEL(obj, delay)
            % Delays <uint> milliseconds.
            % DEL should only be used in macros.
            % <uint> is the delay value in milliseconds.
            obj.device.DEL(delay);
        end
        
        function stepSize = qSST(obj, axis)
            % Gets the distance ("step size") for motions of the given axis
            % that are triggered by a manual control unit.
            % <AxisID> is one axis of the controller.
            % <StepSize> is the distance in physical units, see SST.
            stepSize = obj.device.qSST(axis);
        end
        
        function position = WAIT(obj)
            % WAIT function should be placed right after MOV function so
            % that device movement is properly finished.
            % Wait for motion to stop
            while(0 ~= obj.device.IsMoving('x') || 0 ~= obj.device.IsMoving('y') || 0 ~= obj.device.IsMoving('z') || 0 ~= obj.device.IsMoving('u') || 0 ~= obj.device.IsMoving('v') || 0 ~= obj.device.IsMoving('w'))
                pause(0.1);
            end

            % Wait for axes to be on target
            while(1 ~= obj.device.qONT('x') || 1 ~= obj.device.qONT('y') || 1 ~= obj.device.qONT('z') || 1 ~= obj.device.qONT('u') || 1 ~= obj.device.qONT('v') || 1 ~= obj.device.qONT('w'))
                pause(0.1);
            end
            position = obj.device.qPOS('x y z u v w');
        end
    end
end
