function body_vector = TransformFromInertialToBody(inertial_vecotor, EulerAngles)
    
    phi   = EulerAngles(1);    theta = EulerAngles(2);  psi  = EulerAngles(3);
    
    cphi = cos(phi);   sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);
    
    % Direction cosine matrix (3-2-1 body->inertial)
    R321 = [ cphi*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
             cphi*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
               -sth,                  sphi*cth,                 cphi*cth ];

    body_vector = R321*inertial_vecotor;
end