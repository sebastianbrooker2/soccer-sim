function cfg = fieldConfig()
%FIELDCONFIG Field dimensions for the RoboCup 5v5 field preset.
%   Values taken from the NUbots_K1 FieldDescription.yaml "5v5" preset.

    cfg.FieldLength         = 22.0;  % goal line to goal line (x)
    cfg.FieldWidth           = 14.0;  % touchline to touchline (y)
    cfg.BorderStripMinWidth  = 1.0;
    cfg.LineWidth            = 0.06;

    cfg.GoalWidth            = 2.6;
    cfg.GoalDepth            = 0.6;
    cfg.GoalpostWidth        = 0.10;

    cfg.GoalAreaLength       = 2.0;
    cfg.GoalAreaWidth        = 5.0;

    cfg.PenaltyAreaLength    = 5.0;
    cfg.PenaltyAreaWidth     = 8.0;
    cfg.PenaltyMarkDistance  = 3.5;

    cfg.CenterCircleDiameter = 4.0;

    cfg.BallRadius           = 0.07;

end
