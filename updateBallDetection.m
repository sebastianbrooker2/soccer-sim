function det = updateBallDetection(h, player, ball)
%UPDATEBALLDETECTION Refresh a ball's detection readout for the current
%   player state.
%   DET = UPDATEBALLDETECTION(H, PLAYER, BALL) recomputes the detection
%   (see COMPUTEBALLDETECTION) and shows/hides the ring and confidence
%   label in H (from DRAWBALL) to match, so the readout only appears
%   while the ball is actually detected.

    det = computeBallDetection(player, ball);

    if det.Detected
        set(h.DetectionRing, 'Visible', 'on');
        set(h.Label, 'Visible', 'on', 'String', sprintf('ball %.0f%%', 100*det.Confidence));
    else
        set(h.DetectionRing, 'Visible', 'off');
        set(h.Label, 'Visible', 'off');
    end

end
