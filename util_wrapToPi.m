function ang = util_wrapToPi(ang)
ang = mod(ang + pi, 2*pi) - pi;
end