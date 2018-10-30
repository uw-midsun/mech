% ---------------------------------------------------------------------- %
%
% Double Wishbone Suspension Simulator - printRoad.m
% Fuction to add the road to the visualizer plot and format it
% 
%
% Notes:
%
%   - 
%
% ---------------------------------------------------------------------- %

function printLabels(limits) 

    % Define plot limits
    xlimits = limits(1,:); 
    ylimits = limits(2,:); 
    
    % Show ground
    plot([xlimits(1)*10 xlimits(2)*10],[0 0],'k--');
    
    % Legend
    legend('Neutral Position','Dynamic','Location','southoutside');
    
    hold off;
    xlabel('X-Position (mm)');
    ylabel('Y-Position (mm)');
    xlim(xlimits);
    ylim(ylimits);
    title('Suspension Visualizer');
end
    
