clear all; clc; 
% rmpath('/home/eacappo/sandbox/unit_tests/wet/src/unit_tests_matlab/matlab_scripts/helper_fcns/YAMLMatlab_0.4.3')

datalogs_in = ls('trial_data');
datalogs_in = strsplit(datalogs_in,'.mat')';
temp = cell(size(datalogs_in));
keep = 1:length(datalogs_in);
% delete trailing spaces
for i= 1:length(datalogs_in)
    file = datalogs_in{i};
    while isspace(file(end))
        file(end) = [];
        if isempty(file); break; end
    end
    if isempty(file); keep(i) = []; break; end
    while ~isletter(file(1))
        file(1) = [];
    end
    temp{i} = [ file '.mat'];
end
datalogs_out = temp(keep);

% load 'trial_data/circle:_radial_heading_22-Jun-2015_0.mat'
%%
for uniQvarONLY = 1:length(datalogs_out)
    close all
    drawnow
    
    load(['trial_data/' datalogs_out{uniQvarONLY}])
    name = datalogs_out{uniQvarONLY};
    folder = ['/home/eacappo/Desktop/' name(1:end-4)  ];
    mkdir(folder)

str = {' ','loop_time','position','position_error',...
       'velocity', 'velocity_error', 'rpms', 'yaw_angular_rate',...
       'attitude','volume','xy_plane','xyzcon','qdes','omgd',...
       'moving average','RMS','histogram','fft','IMU_vs_Vicon','XYZ_plot3'};

   if ~exist('volume_stuff','var')
       volume_stuff = 0;
   end
   
   if strcmp(trajectory,'volume')             
       ToPlot = str([2,10,15,16,17]);
   else
%        ToPlot = str([2,15,16,17]);   
ToPlot = str([2 3 4 5 6 7 15 16 17 19 20]);    
   end
   
    plot_selected(ToPlot,...
        ROBOT_HISTORY,ROBOT_TRAJECTORY,IMU,ODOM,looptime,trimming,trajectory, trimming_stuff, volume_stuff);            

    clear volume_stuff
    
    drawnow
    
    figure_handles = sort(findobj('Type','Figure'));
    for foobar = 1:length(figure_handles)
        figure(figure_handles(foobar))
        drawnow
        time = clock;
        name = [ folder '/' strcat(num2str(round(time(end)*1000))) '.png'];
        print(name,'-dpng')
    end

end
