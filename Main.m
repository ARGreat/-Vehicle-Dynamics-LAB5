%Name: 3801-Lab5 Main
%Authors: Alex Godbout
%Date: 11/11/2025
%Purpose: Run each Task as a fucntion, this allows a quick an easy way to
    %toggle tasks.  Additionally has the option of saving each figure as
    %its figure name


clear; clc; close all;

%Toggles
saveFigures = 0;
run_task2 = 0;
run_task3 = 1;

ttwistor
Task2(run_task2);
Task3(run_task3);

%%------------------------%%
%%-----Save Figures-------%%
%%------------------------%%

if(saveFigures == 1)
    figHandles = findall(0, 'Type', 'figure');
    n = length(figHandles);    
    for i=1:n
        name = strrep(figHandles(i).Name,'.','_');
        name = strrep(name,')','');
        name = strrep(name,'(','');
        saveas(figHandles(i),"./Figures/"+name,'png');
    end
end