function position = WAIT(device)
    % Wait for motion to stop
    while(0 ~= device.IsMoving('x') || 0 ~= device.IsMoving('y') || 0 ~= device.IsMoving('z') || 0 ~= device.IsMoving('u') || 0 ~= device.IsMoving('v') || 0 ~= device.IsMoving('w'))
        pause(0.1);
    end

    % Wait for axes to be on target
    while(1 ~= device.qONT('x') || 1 ~= device.qONT('y') || 1 ~= device.qONT('z') || 1 ~= device.qONT('u') || 1 ~= device.qONT('v') || 1 ~= device.qONT('w'))
        pause(0.1);
    end
    position = device.qPOS('x y z u v w');
end