function plot_ex2(results)

    % Default graphics settings
    set(0, 'DefaultTextInterpreter', 'latex')
    set(0, 'DefaultLegendInterpreter', 'latex')
    set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
    lw = 2;

    for i = 1 : length(results)

        % Figure generation
        h = figure('Renderer', 'painters', 'Position', [10 10 900 350]);

        
        % Plot
        p1 = plot(results(i).xd, results(i).yd, '-', 'Linewidth', lw ,'Color', [0.0, 0.0, 1.0]);
        hold on
        p2 = plot(results(i).x, results(i).y, '-', 'Linewidth', lw ,'Color', [1.0, 0.0, 0.0]);

        % Title
        title('Path of the bicycle');
        
        % Legend
        legend([p1, p2], {'Desired'; 'Real'});
        legend('Location','northwest','Orientation','horizontal','AutoUpdate','off');
        
        % Labels
        xlabel('x [m]');
        ylabel('y [m]');
        set(gca, 'FontSize', 19);


        %Grid
        grid on
        box on

        % Options
        set(gcf,'color','w');
        set(h, 'MenuBar', 'none');
        set(h, 'ToolBar', 'none');

        vec1 = results(i).x(:);
        vec2 = results(i).xd(:);
        vec3 = results(i).y(:);
        vec4 = results(i).yd(:);
        %Limits
        xlim_lb = min([vec1;vec2]);
        xlim_ub = max([vec1;vec2]);
        xlim([xlim_lb xlim_ub]);
        ylim_lb = min([vec3;vec4]);
        ylim_ub = max([vec3;vec4]);
        ylim([ylim_lb ylim_ub]);

        %Fix inner position of the plot inside the figure
        set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
        annotation('rectangle',[0 0 1 1],'Color','w');

        exportgraphics(h, ['FSR_traj_' num2str(i) '.pdf']);

        % Figure generation
        h2 = figure('Renderer', 'painters', 'Position', [10 10 900 350]);

        % Plot
        p1 = plot(results(i).t, results(i).ex, '-', 'Linewidth', lw ,'Color', [0.0, 0.0, 1.0]);
        hold on
        p2 = plot(results(i).t, results(i).ey, '-', 'Linewidth', lw ,'Color', [1.0, 0.0, 0.0]);

        % Title
        title(['Position Error'])

        %Legend
        legend([p1, p2], {'$e_x$', '$e_y$'}, ...
       'Interpreter', 'latex', ...
       'Location', 'northeast', ...
       'Orientation', 'horizontal', ...
       'AutoUpdate', 'off');


        % Label
        xlabel('Time [s]');
        ylabel('Error [m]');
        set(gca, 'FontSize',19);

        %Grid
        grid on
        box on

        % Options
        set(gcf,'color','w');
        set(h2, 'MenuBar', 'none');
        set(h2, 'ToolBar', 'none');
        
        vec1 = results(i).ex(:);
        vec2 = results(i).ey(:);
        %Limits
        xlim_lb = min(results(i).t);
        xlim_ub = max(results(i).t);
        xlim([xlim_lb xlim_ub]);
        ylim_lb = min([vec1;vec2]);
        ylim_ub = max([vec1;vec2]);
        ylim([ylim_lb ylim_ub]);

        %Fix inner position of the plot inside the figure
        set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
        annotation('rectangle',[0 0 1 1],'Color','w');

        exportgraphics(h2, ['FSR_epos_ ' num2str(i) '.pdf']);

        % Figure generation
        h3 = figure('Renderer', 'painters', 'Position', [10 10 900 350]);

        % Plot
        p1 = plot(results(i).t, results(i).v, '-', 'Linewidth', lw ,'Color', [0.0, 0.0, 1.0]);

        % Title
        title('Linear Velocity')

        % Label
        xlabel('Time [s]');
        ylabel('Velocity [m/s]');
        set(gca, 'FontSize',19);
       

        %Grid
        grid on
        box on

        % Options
        set(gcf,'color','w');
        set(h2, 'MenuBar', 'none');
        set(h2, 'ToolBar', 'none');

        vec1 = results(i).v(:);        

        %Limits
        xlim_lb = min([results(i).t]);
        xlim_ub = max([results(i).t]);
        xlim([xlim_lb xlim_ub]);
        ylim_lb = min(vec1);
        ylim_ub = max(vec1);
        ylim([ylim_lb ylim_ub]);

        %Fix inner position of the plot inside the figure
        set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
        annotation('rectangle',[0 0 1 1],'Color','w');

        exportgraphics(h3, ['FSR_linvel_ ' num2str(i) '.pdf']); 

        % Figure generation
        h4 = figure('Renderer', 'painters', 'Position', [10 10 900 350]);

        % Plot
        p1 = plot(results(i).t, results(i).omega, '-', 'Linewidth', lw ,'Color', [0.0, 0.0, 1.0]);

        % Title
        title(['Angular Velocity'])


        % Label
        xlabel('Time [s]');
        ylabel('Velocty [rad/s]');
        set(gca, 'FontSize',19);

        %Grid
        grid on
        box on

        % Options
        set(gcf,'color','w');
        set(h4, 'MenuBar', 'none');
        set(h4, 'ToolBar', 'none');

        vec1 = results(i).omega(:);
        %Limits
        xlim_lb = min(results(i).t);
        xlim_ub = max(results(i).t);
        xlim([xlim_lb xlim_ub]);
        ylim_lb = min(vec1);
        ylim_ub = max(vec1);
        ylim([ylim_lb ylim_ub]);

        %Fix inner position of the plot inside the figure
        set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
        annotation('rectangle',[0 0 1 1],'Color','w');

        exportgraphics(h4, ['FSR_angvel_ ' num2str(i) '.pdf']);                

        % Figure generation
        h5 = figure('Renderer', 'painters', 'Position', [10 10 900 350]);

        % Plot
        p1 = plot(results(i).t, results(i).etheta, '-', 'Linewidth', lw ,'Color', [0.0, 0.0, 1.0]);
        hold on
        p2 = plot(results(i).t, results(i).ephi, '-', 'Linewidth', lw ,'Color', [1.0, 0.0, 0.0]);

        % Title
        title(['Orientation Error'])

        %Legend
        legend([p1, p2], {'$e_{\theta}$', '$e_{\phi}$'}, ...
       'Interpreter', 'latex', ...
       'Location', 'northeast', ...
       'Orientation', 'horizontal', ...
       'AutoUpdate', 'off');


        % Label
        xlabel('Time [s]');
        ylabel('Error [rad]');
        set(gca, 'FontSize',19);

        %Grid
        grid on
        box on

        % Options
        set(gcf,'color','w');
        set(h5, 'MenuBar', 'none');
        set(h5, 'ToolBar', 'none');


        vec1 = results(i).ephi(:);
        vec2 = results(i).etheta(:);
        %Limits
        xlim_lb = min(results(i).t);
        xlim_ub = max(results(i).t);
        xlim([xlim_lb xlim_ub]);
        ylim_lb = min([vec1;vec2]);
        ylim_ub = max([vec1;vec2]);
        ylim([ylim_lb ylim_ub]);

        %Fix inner position of the plot inside the figure
        set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
        annotation('rectangle',[0 0 1 1],'Color','w');

        exportgraphics(h5, ['FSR_eang_ ' num2str(i) '.pdf']); 


        % Figure generation
        h6 = figure('Renderer', 'painters', 'Position', [10 10 900 350]);

        % Plot
        p1 = plot(results(i).t, results(i).theta, '-', 'Linewidth', lw ,'Color', [0.0, 0.0, 1.0]);
        hold on
        p2 = plot(results(i).t, results(i).phi, '-', 'Linewidth', lw ,'Color', [1.0, 0.0, 0.0]);

        % Title
        title(['Orientation'])

        %Legend
        legend([p1, p2], {'$\theta$', '$\phi$'}, ...
       'Interpreter', 'latex', ...
       'Location', 'northeast', ...
       'Orientation', 'horizontal', ...
       'AutoUpdate', 'off');


        % Label
        xlabel('Time [s]');
        ylabel('Orientation Angle [rad]');
        set(gca, 'FontSize',19);

        %Grid
        grid on
        box on

        % Options
        set(gcf,'color','w');
        set(h6, 'MenuBar', 'none');
        set(h6, 'ToolBar', 'none');

        vec1 = results(i).phi(:);
        vec2 = results(i).theta(:);        
        %Limits
        xlim_lb = min(results(i).t);
        xlim_ub = max(results(i).t);
        xlim([xlim_lb xlim_ub]);
        ylim_lb = min([vec1;vec2]);
        ylim_ub = max([vec1;vec2]);
        ylim([ylim_lb ylim_ub]);

        %Fix inner position of the plot inside the figure
        set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
        annotation('rectangle',[0 0 1 1],'Color','w');

        exportgraphics(h6, ['FSR_ang_ ' num2str(i) '.pdf']); 
        
    end
