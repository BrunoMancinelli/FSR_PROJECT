function plot_arm(result)

    % Default graphics settings
    set(0, 'DefaultTextInterpreter', 'latex')
    set(0, 'DefaultLegendInterpreter', 'latex')
    set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
    lw = 2;


    % Figure generation
    h = figure('Renderer', 'painters', 'Position', [10 10 900 350]);

        
    % Plot
    p1 = plot(result.t, result.error, '-', 'Linewidth', lw ,'Color', [0.0, 0.0, 1.0]);

    % Title
    title('Look-at Alignment Over Time');
        
        
    % Labels
    xlabel('Time [s]');
    ylabel('Alignment [rad]');
    set(gca, 'FontSize', 19);


    %Grid
    grid on
    box on

    % Options
    set(gcf,'color','w');
    set(h, 'MenuBar', 'none');
    set(h, 'ToolBar', 'none');

    vec1 = result.t(:);
    vec2 = result.error(:);
        %Limits
    xlim_lb = min(vec1);
    xlim_ub = max(vec1);
    xlim([xlim_lb xlim_ub]);
    ylim_lb = min(vec2);
    ylim_ub = max(vec2);
    ylim([ylim_lb ylim_ub]);

    %Fix inner position of the plot inside the figure
    set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
    annotation('rectangle',[0 0 1 1],'Color','w');

    exportgraphics(h, 'FSR_arm_err.pdf');


    % Figure generation
    h2 = figure('Renderer', 'painters', 'Position', [10 10 900 350]);

    % Plot
    p1 = plot(result.t, result.dq1, '-', 'Linewidth', lw ,'Color', [0.0, 0.0, 1.0]);
    hold on
    p2 = plot(result.t, result.dq2, '-', 'Linewidth', lw ,'Color', [1.0, 0.3, 0.3]);
    p3 = plot(result.t, result.dq3, '-', 'Linewidth', lw ,'Color', [0.3, 1.0, 0.3]);   
    p4 = plot(result.t, result.dq4, '-', 'Linewidth', lw ,'Color', [1.0, 0.84, 0.0]);   
    p5 = plot(result.t, result.dq5, '-', 'Linewidth', lw ,'Color',  [0.4, 0, 0.6]);  
    p6 = plot(result.t, result.dq6, '-', 'Linewidth', lw ,'Color', [1.0, 0.6, 0.0]);     

    % Title
    title(['Joint Velocities Over Time'])

    legend({'$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$', '$\dot{q}_5$', '$\dot{q}_6$'}, ...
           'Interpreter', 'latex', ...
           'Location', 'northeast', ...
           'Orientation', 'horizontal', ...
           'AutoUpdate', 'off', ...
            'FontSize', 12);



        % Label
        xlabel('Time [s]');
        ylabel('Velocity [rad/s]');
        set(gca, 'FontSize',19);

        %Grid
        grid on
        box on

        % Options
        set(gcf,'color','w');
        set(h2, 'MenuBar', 'none');
        set(h2, 'ToolBar', 'none');
        
        vec1 = result.dq1(:);
        vec2 = result.dq2(:);
        vec3 = result.dq3(:);
        vec4 = result.dq4(:);
        vec5 = result.dq5(:);
        vec6 = result.dq6(:);
        %Limits
        xlim_lb = min(result.t);
        xlim_ub = max(result.t);
        xlim([xlim_lb xlim_ub]);
        ylim_lb = min([vec1;vec2;vec3;vec4;vec5;vec6]);
        ylim_ub = max([vec1;vec2;vec3;vec4;vec5;vec6]);
        ylim([ylim_lb ylim_ub]);

        %Fix inner position of the plot inside the figure
        set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
        annotation('rectangle',[0 0 1 1],'Color','w');

        exportgraphics(h2,'FSR_dq_arm.pdf');

        hold off
        
    end