%Authors: Alex Godbout, Brady Hormuth
%Date Modified: 11/11/2025
%Purpose: Implement the Non-Linear Aircraft Equations of Motion
%Returns: The derivative of the statevector at the given initial state
function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial,aircraft_parameters)
    
    % var = [x y z phi theta psi uE vE wE p q r]^T
    % I   = [Ix Iy Iz]^T
    % motor_forces = [F1 F2 F3 F4]^T  (N)
    % Returns var_dot with the same ordering.
    
    % angles, translational velocities, body rates
    phi   = var(4);    theta = var(5);   psi  = var(6);
    uE    = var(7);    vE    = var(8);   wE   = var(9);
    p = var(10); q = var(11); r = var(12);
    
    v_inertial = [uE; vE; wE];
    omega_b = [p; q; r];



    % Trig
    cphi = cos(phi);   sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);
    ttheta  = tan(theta);  sctheta = sec(theta);
    
    % Direction cosine matrix (3-2-1 body->inertial)
    R321 = [ cphi*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
             cphi*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
               -sth,                  sphi*cth,                 cphi*cth ];
    
    % Euler-rate mapping
    ER = [1,      sphi*ttheta,  cphi*ttheta;
          0,      cphi,         -sphi;
          0,      sphi*sctheta,  cphi*sctheta];
    
    % Kinematics
    inertialPos_dot = R321 * v_inertial;  % [xdot ydot zdot]^T
    eulerAng_dot    = ER * omega_b;       % [phidot thetadot psidot]^T
    


    %% Forces and Moments
    [aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);
    
    
    % Translational dynamics (gravity resolved in inertial frame)
    uE_dot =  r*vE - q*wE - g*sth          + aero_forces(1)/m;
    vE_dot =  p*wE - r*uE + g*cth*sphi     + aero_forces(2)/m;
    wE_dot =  q*uE - p*vE + g*cth*cphi +  aero_forces(3)/m;
    inertialVel_dot = [uE_dot; vE_dot; wE_dot];
    
    % Inertias
    G = aircraft_parameters.Ix*aircraft_parameters.Iz - aircraft_parameters.Ixz^2;
    G1 = aircraft_parameters.Ixz*(aircraft_parameters.Ix-aircraft_parameters.Iy+aircraft_parameters.Iz)/G;
    G2 = (aircraft_parameters.Iz*(aircraft_parameters.Iz-aircraft_parameters.Iy)+aircraft_parameters.Ixz^2)/G;
    G3 = aircraft_parameters.Iz/G;
    G4 = aircraft_parameters.Ixz/G;
    G5 = (aircraft_parameters.Iz-aircraft_parameters.Ix)/aircraft_parameters.Iy;
    G6 = aircraft_parameters.Ixz/aircraft_parameters.Iy;
    G7 = (aircraft_parameters.Ix*(aircraft_parameters.Ix-aircraft_parameters.Iy)+aircraft_parameters.Ixz)/G;
    G8 = aircraft_parameters.Ix/G;





    % Rotational dynamics
    p_dot = G1*P*q - G2*q*r + G3*aero_moments(1)+G4*aero_moments(3);
    q_dot = G5*p*r - G6*(p^2-r^2) + aero_moments(2)/aircraft_parameters.Iy;
    r_dot = G7*P*q - G1*q*r + G4*aero_moments(1)+G8*aero_moments(3);
    eulerRate_dot = [p_dot; q_dot; r_dot];
    
    % Pack
    var_dot = [inertialPos_dot;
               eulerAng_dot;
               inertialVel_dot;
               eulerRate_dot];

end