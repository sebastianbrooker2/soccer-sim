function ev = makeEvent(type, t, name, trackLabel, detLabel, d2S, d2R, n, flag, note, life)
%MAKEEVENT One row of the debug log. Fields are reused by type:
%   assoc : detection fused into track NAME. trackLabel/detLabel are the
%           REAL objects (ground truth, for diagnosis only); flag = they
%           match; d2S/d2R = Mahalanobis distance (full S / R only); n =
%           fused detections the track has now.
%   spawn : new track NAME. d2S/d2R = distance to the closest still-live
%           track of the same real object (inf if none), n = how many
%           such live tracks, note = first | respawn | duplicate.
%   delete: track NAME removed after maxMisses expected-but-unseen scans;
%           n = detections it had fused, life = seconds it lived, flag =
%           it was the last live track for that real object.
%   merge : track NAME dropped as a duplicate of the track named in NOTE
%           (detLabel = that track's real object), d2S = merge distance,
%           d2R = metres between the two estimates, flag = same real
%           object (a correct merge).
%   spawn extras: life = position std (m) of the new track after its
%           first fusion, aux = its position error (m) vs the truth.
%   fusefail: an update that did not converge was skipped (name = track).
ev = struct('type', type, 't', t, 'name', name, 'trackLabel', trackLabel, 'detLabel', detLabel, ...
    'd2S', d2S, 'd2R', d2R, 'n', n, 'flag', flag, 'note', note, 'life', life, 'aux', 0);
end
