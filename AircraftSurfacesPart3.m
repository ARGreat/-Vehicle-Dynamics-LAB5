function aircraft_surfaces = AircraftSurfacesPart3(t,aircraft_surfaces,doublet_time,doublet_size)
    

de = repmat(aircraft_surfaces,1,length(t));
    filter = t >= 0 & t < doublet_time;
    de(1,filter) = aircraft_surfaces(1) + doublet_size;
    de(1,t > doublet_time & t <= doublet_time) = aircraft_surfaces(1) - doublet_size;

    %{
    if(t > 0 && t <= doublet_time)
        de = aircraft_surfaces(1,1) + doublet_size;
    elseif(t > doublet_time && t <= doublet_time)
        
    else
        de = aircraft_surfaces(1,1);
    end
    %}

    aircraft_surfaces = [de,aircraft_surfaces(1,2),aircraft_surfaces(1,3)];
end