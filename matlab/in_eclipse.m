function e = in_eclipse(r, s, p)
% True if the satellite at r (ECI) is in the Earth's shadow.
% Cylindrical shadow model (no penumbra).
%   s : unit vector to the sun

along = r' * s;
if along > 0
    e = false;           % on the sun side of the Earth
    return
end
perp = r - along*s;
e = norm(perp) < p.R_earth;
end
