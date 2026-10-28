function occluded = checkOcclusion(cameraPos, targetPos, obstacles, cylinderRadius, cylinderHeight)
%CHECKOCCLUSION True if the segment CAMERAPOS->TARGETPOS is blocked.
%   OCCLUDED = CHECKOCCLUSION(CAMERAPOS, TARGETPOS, OBSTACLES, ...
%   CYLINDERRADIUS, CYLINDERHEIGHT) models each obstacle (e.g. another
%   robot) as a vertical cylinder of radius CYLINDERRADIUS and height
%   CYLINDERHEIGHT standing on the ground (z = 0), and tests whether the
%   3D segment from CAMERAPOS to TARGETPOS (each 3x1) passes through any
%   of them. OBSTACLES is an Mx2 list of [x y] cylinder-centre positions
%   (the caller is responsible for excluding the observing robot itself).

    occluded = false;
    if isempty(obstacles)
        return
    end

    p0 = cameraPos(:);
    d  = targetPos(:) - p0;

    for m = 1:size(obstacles, 1)
        cx = obstacles(m, 1);
        cy = obstacles(m, 2);

        ox = p0(1) - cx;
        oy = p0(2) - cy;
        dx = d(1);
        dy = d(2);

        a = dx^2 + dy^2;
        if a < eps
            continue % segment has no horizontal extent, can't cross a vertical cylinder wall
        end

        b  = 2*(ox*dx + oy*dy);
        cc = ox^2 + oy^2 - cylinderRadius^2;

        disc = b^2 - 4*a*cc;
        if disc < 0
            continue % horizontal projection of the segment never reaches the cylinder
        end

        sqrtDisc = sqrt(disc);
        for s = [(-b - sqrtDisc)/(2*a), (-b + sqrtDisc)/(2*a)]
            if s >= 0 && s <= 1
                zHit = p0(3) + s*d(3);
                if zHit >= 0 && zHit <= cylinderHeight
                    occluded = true;
                    return
                end
            end
        end
    end

end
