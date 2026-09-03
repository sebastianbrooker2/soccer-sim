function [fovX, fovY, arcX, arcY] = computeFOVGeometry(player)
%COMPUTEFOVGEOMETRY Points describing a player's field-of-view wedge.
%   [FOVX, FOVY, ARCX, ARCY] = COMPUTEFOVGEOMETRY(PLAYER) returns the
%   closed wedge polygon (FOVX, FOVY) and the arc points (ARCX, ARCY)
%   spanning PLAYER's FOV at PLAYER.Range, centred on the look direction
%   (Heading + GazeOffset) rather than the walking heading, so the cone
%   sweeps independently of the direction of travel.

    if isfield(player, 'GazeOffset')
        lookHeading = player.Heading + player.GazeOffset;
    else
        lookHeading = player.Heading;
    end

    halfFOV = player.FOV / 2;
    nArc = 40;
    arcAngles = linspace(lookHeading - halfFOV, lookHeading + halfFOV, nArc);
    arcX = player.Position(1) + player.Range * cos(arcAngles);
    arcY = player.Position(2) + player.Range * sin(arcAngles);

    fovX = [player.Position(1), arcX, player.Position(1)];
    fovY = [player.Position(2), arcY, player.Position(2)];

end
