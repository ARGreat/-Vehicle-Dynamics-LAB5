function Task3(run_task3)

if(~run_task3)
    return;
end

%Should include all code for Lab Task 3

%We run ode45 and plot here

%load Aircraft Parameters
ttwistor

%%-----------------------------------%%
%% ----------Task 3 Plots----------- %%
%%-----------------------------------%%

%Set all the initial conditions
x_0 = [0;0;-1800;0;0.02780;0;20.99;0;0.5837;0;0;0]; %2.2
u_0 = [0.1079;0;0;0.3182]; %2.2 Deflections
doublet_size = deg2rad(15);
doublet_time = 0.25;

%Set Figure Label Parameters
figureSuffix = ["3.1","3.2"];
dispName= ["Non-Linearized Aircraft","Non-Linearized Aircraft"];

%Run ode45 and plot
aircraft_state_0 = x_0;

wind_inertial = [0;0;0];

t_run = [3,100];

for(i=1:2)
    aircraft_surfaces = u_0;
    tspan = [0, t_run(i)]; % from 0 to 100 seconds
    odefun = @(time,aircraft_state) AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size,...
    doublet_time, wind_inertial,aircraft_parameters);
    [time, aircraft_state_array] = ode45(odefun, tspan, aircraft_state_0);
    %Plot
    n = length(time);
    aircraft_surfaces = AircraftSurfacesPart3(time,aircraft_surfaces,doublet_time,doublet_size);
    fig = [3001;3002;3003;3004;3005;3006] + 10*i;
    col = ['b','b','b','b','b','b'];
    PlotAircraftSim(time, aircraft_state_array, aircraft_surfaces,fig, col,figureSuffix(i),dispName(i))
end

end
