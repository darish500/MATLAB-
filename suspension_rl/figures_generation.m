%% FINAL PUBLICATION-READY FIGURES — ALL ISSUES FIXED
clear; clc;
load('passive_baseline.mat');
load('all_timedata.mat');

% Colors
cp = [0.0  0.0  0.0];    % black — passive
ct = [0.0  0.45 0.74];   % blue  — TD3
ch = [0.85 0.33 0.10];   % red   — hybrid
lw = 2.0;
fs = 13;

function fixAxes(ax, fs)
    set(ax, 'FontSize', fs, 'FontName', 'Times New Roman', ...
        'Color', 'white', 'XColor', 'black', 'YColor', 'black', ...
        'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.5, ...
        'Box', 'on', 'LineWidth', 0.8);
    ax.Title.Color = 'black';
    ax.XLabel.Color = 'black';
    ax.YLabel.Color = 'black';
end

function fixLegend(lg)
    set(lg, 'Color', 'white', 'EdgeColor', 'black', ...
        'TextColor', 'black', 'FontName', 'Times New Roman');
end

%% ================================================================
%% FIGURE 1 — Body Acceleration: Smooth Road
%% ================================================================
fig1 = figure('Color','white','Position',[50 50 900 380]);
ax1 = axes(fig1);
plot(ax1, passive_smooth.time, passive_smooth.zs_ddot, 'k-',  'LineWidth', lw); hold(ax1,'on')
plot(ax1, td3_smooth_td.time,  td3_smooth_td.zs,       'b--', 'LineWidth', lw);
plot(ax1, hyb_smooth_td.time,  hyb_smooth_td.zs,       'r:',  'LineWidth', lw+0.5);
xlabel(ax1,'Time (s)','FontSize',fs,'FontName','Times New Roman','Color','black')
ylabel(ax1,'Body Acceleration (m/s²)','FontSize',fs,'FontName','Times New Roman','Color','black')
title(ax1,'Body Acceleration — Smooth Road','FontSize',fs+1,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg1 = legend(ax1,'Passive','TD3 Only','Hybrid TD3+Safety','Location','northeast','FontSize',fs-1);
fixAxes(ax1, fs-1); fixLegend(lg1);
grid(ax1,'on'); xlim(ax1,[0 10]);
saveas(fig1,'Fig1_Smooth_Body_Accel.png'); fprintf('✅ Fig1\n')

%% ================================================================
%% FIGURE 2 — Body Acceleration: Rough Road
%% ================================================================
fig2 = figure('Color','white','Position',[50 50 900 380]);
ax2 = axes(fig2);
plot(ax2, passive_rough.time, passive_rough.zs_ddot, 'k-',  'LineWidth', lw); hold(ax2,'on')
plot(ax2, td3_rough_td.time,  td3_rough_td.zs,       'b--', 'LineWidth', lw);
plot(ax2, hyb_rough_td.time,  hyb_rough_td.zs,       'r:',  'LineWidth', lw+0.5);
xlabel(ax2,'Time (s)','FontSize',fs,'FontName','Times New Roman','Color','black')
ylabel(ax2,'Body Acceleration (m/s²)','FontSize',fs,'FontName','Times New Roman','Color','black')
title(ax2,'Body Acceleration — Rough Road','FontSize',fs+1,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg2 = legend(ax2,'Passive','TD3 Only','Hybrid TD3+Safety','Location','northeast','FontSize',fs-1);
fixAxes(ax2, fs-1); fixLegend(lg2);
grid(ax2,'on'); xlim(ax2,[0 10]);
saveas(fig2,'Fig2_Rough_Body_Accel.png'); fprintf('✅ Fig2\n')

%% ================================================================
%% FIGURE 3 — Pothole: split view
%% ================================================================
fig3 = figure('Color','white','Position',[50 50 1100 420]);

ax3a = subplot(1,2,1);
plot(ax3a, passive_pothole.time, passive_pothole.zs_ddot, 'k-',  'LineWidth', lw); hold(ax3a,'on')
plot(ax3a, hyb_pothole_td.time,  hyb_pothole_td.zs,       'r--', 'LineWidth', lw);
xlabel(ax3a,'Time (s)','FontSize',fs,'FontName','Times New Roman','Color','black')
ylabel(ax3a,'Body Acceleration (m/s²)','FontSize',fs,'FontName','Times New Roman','Color','black')
title(ax3a,'(a) Passive vs Hybrid TD3+Safety','FontSize',fs,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg3a = legend(ax3a,'Passive','Hybrid TD3+Safety','Location','northeast','FontSize',fs-1);
fixAxes(ax3a, fs-1); fixLegend(lg3a);
grid(ax3a,'on'); xlim(ax3a,[0 10]); ylim(ax3a,[-2 6]);

ax3b = subplot(1,2,2);
plot(ax3b, passive_pothole.time, passive_pothole.zs_ddot, 'k-',  'LineWidth', lw); hold(ax3b,'on')
plot(ax3b, td3_pothole_td.time,  td3_pothole_td.zs,       'b--', 'LineWidth', lw);
xlabel(ax3b,'Time (s)','FontSize',fs,'FontName','Times New Roman','Color','black')
ylabel(ax3b,'Body Acceleration (m/s²)','FontSize',fs,'FontName','Times New Roman','Color','black')
title(ax3b,'(b) Passive vs TD3 Only (amplified)','FontSize',fs,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg3b = legend(ax3b,'Passive','TD3 Only','Location','northeast','FontSize',fs-1);
fixAxes(ax3b, fs-1); fixLegend(lg3b);
grid(ax3b,'on'); xlim(ax3b,[0 10]);

sgt3 = sgtitle('Body Acceleration — Pothole Road','FontSize',fs+2,'FontName','Times New Roman','FontWeight','bold','Color','black');
saveas(fig3,'Fig3_Pothole_Body_Accel.png'); fprintf('✅ Fig3\n')

%% ================================================================
%% FIGURE 4 — Battery Acceleration: All Roads
%% ================================================================
fig4 = figure('Color','white','Position',[50 50 1300 400]);

ax4a = subplot(1,3,1);
plot(ax4a, passive_smooth.time, passive_smooth.batt_accel, 'k-',  'LineWidth', lw); hold(ax4a,'on')
plot(ax4a, td3_smooth_td.time,  td3_smooth_td.batt,        'b--', 'LineWidth', lw);
plot(ax4a, hyb_smooth_td.time,  hyb_smooth_td.batt,        'r:',  'LineWidth', lw+0.5);
xlabel(ax4a,'Time (s)','FontSize',fs-1,'FontName','Times New Roman','Color','black')
ylabel(ax4a,'Battery Acceleration (m/s²)','FontSize',fs-1,'FontName','Times New Roman','Color','black')
title(ax4a,'(a) Smooth Road','FontSize',fs,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg4a = legend(ax4a,'Passive','TD3','Hybrid','Location','northeast','FontSize',fs-2);
fixAxes(ax4a, fs-2); fixLegend(lg4a); grid(ax4a,'on'); xlim(ax4a,[0 10]);

ax4b = subplot(1,3,2);
plot(ax4b, passive_rough.time, passive_rough.batt_accel, 'k-',  'LineWidth', lw); hold(ax4b,'on')
plot(ax4b, td3_rough_td.time,  td3_rough_td.batt,        'b--', 'LineWidth', lw);
plot(ax4b, hyb_rough_td.time,  hyb_rough_td.batt,        'r:',  'LineWidth', lw+0.5);
xlabel(ax4b,'Time (s)','FontSize',fs-1,'FontName','Times New Roman','Color','black')
ylabel(ax4b,'Battery Acceleration (m/s²)','FontSize',fs-1,'FontName','Times New Roman','Color','black')
title(ax4b,'(b) Rough Road','FontSize',fs,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg4b = legend(ax4b,'Passive','TD3','Hybrid','Location','northeast','FontSize',fs-2);
fixAxes(ax4b, fs-2); fixLegend(lg4b); grid(ax4b,'on'); xlim(ax4b,[0 10]);

ax4c = subplot(1,3,3);
plot(ax4c, passive_pothole.time, passive_pothole.batt_accel, 'k-',  'LineWidth', lw); hold(ax4c,'on')
plot(ax4c, hyb_pothole_td.time,  hyb_pothole_td.batt,        'r--', 'LineWidth', lw);
plot(ax4c, td3_pothole_td.time,  td3_pothole_td.batt,        'b:',  'LineWidth', lw+0.5);
xlabel(ax4c,'Time (s)','FontSize',fs-1,'FontName','Times New Roman','Color','black')
ylabel(ax4c,'Battery Acceleration (m/s²)','FontSize',fs-1,'FontName','Times New Roman','Color','black')
title(ax4c,'(c) Pothole Road','FontSize',fs,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg4c = legend(ax4c,'Passive','Hybrid','TD3 (amplified)','Location','northeast','FontSize',fs-2);
fixAxes(ax4c, fs-2); fixLegend(lg4c); grid(ax4c,'on'); xlim(ax4c,[0 10]);

sgtitle('Battery Pack Acceleration Comparison','FontSize',fs+2,'FontName','Times New Roman','FontWeight','bold','Color','black')
saveas(fig4,'Fig4_Battery_Accel_All_Roads.png'); fprintf('✅ Fig4\n')

%% ================================================================
%% FIGURE 5 — Degradation Index: All Roads
%% ================================================================
fig5 = figure('Color','white','Position',[50 50 1300 400]);

ax5a = subplot(1,3,1);
plot(ax5a, passive_smooth.time, passive_smooth.degradation, 'k-',  'LineWidth', lw); hold(ax5a,'on')
plot(ax5a, td3_smooth_td.time,  td3_smooth_td.deg,          'b--', 'LineWidth', lw);
plot(ax5a, hyb_smooth_td.time,  hyb_smooth_td.deg,          'r:',  'LineWidth', lw+0.5);
xlabel(ax5a,'Time (s)','FontSize',fs-1,'FontName','Times New Roman','Color','black')
ylabel(ax5a,'Degradation Index','FontSize',fs-1,'FontName','Times New Roman','Color','black')
title(ax5a,'(a) Smooth Road','FontSize',fs,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg5a = legend(ax5a,'Passive','TD3','Hybrid','Location','northwest','FontSize',fs-2);
fixAxes(ax5a, fs-2); fixLegend(lg5a); grid(ax5a,'on'); xlim(ax5a,[0 10]);

ax5b = subplot(1,3,2);
plot(ax5b, passive_rough.time, passive_rough.degradation, 'k-',  'LineWidth', lw); hold(ax5b,'on')
plot(ax5b, td3_rough_td.time,  td3_rough_td.deg,          'b--', 'LineWidth', lw);
plot(ax5b, hyb_rough_td.time,  hyb_rough_td.deg,          'r:',  'LineWidth', lw+0.5);
xlabel(ax5b,'Time (s)','FontSize',fs-1,'FontName','Times New Roman','Color','black')
ylabel(ax5b,'Degradation Index','FontSize',fs-1,'FontName','Times New Roman','Color','black')
title(ax5b,'(b) Rough Road','FontSize',fs,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg5b = legend(ax5b,'Passive','TD3','Hybrid','Location','northwest','FontSize',fs-2);
fixAxes(ax5b, fs-2); fixLegend(lg5b); grid(ax5b,'on'); xlim(ax5b,[0 10]);

ax5c = subplot(1,3,3);
plot(ax5c, passive_pothole.time, passive_pothole.degradation, 'k-',  'LineWidth', lw); hold(ax5c,'on')
plot(ax5c, hyb_pothole_td.time,  hyb_pothole_td.deg,          'r--', 'LineWidth', lw);
plot(ax5c, td3_pothole_td.time,  td3_pothole_td.deg,          'b:',  'LineWidth', lw+0.5);
xlabel(ax5c,'Time (s)','FontSize',fs-1,'FontName','Times New Roman','Color','black')
ylabel(ax5c,'Degradation Index','FontSize',fs-1,'FontName','Times New Roman','Color','black')
title(ax5c,'(c) Pothole Road','FontSize',fs,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg5c = legend(ax5c,'Passive','Hybrid','TD3 (amplified)','Location','northwest','FontSize',fs-2);
fixAxes(ax5c, fs-2); fixLegend(lg5c); grid(ax5c,'on'); xlim(ax5c,[0 10]);

sgtitle('Battery Degradation Index Comparison','FontSize',fs+2,'FontName','Times New Roman','FontWeight','bold','Color','black')
saveas(fig5,'Fig5_Degradation_All_Roads.png'); fprintf('✅ Fig5\n')

%% ================================================================
%% FIGURE 6 — Bar: RMS Body Acceleration
%% ================================================================
fig6 = figure('Color','white','Position',[50 50 820 500]);
ax6 = axes(fig6);

controllers = {'Passive','TD3 Only','Hybrid TD3+Safety'};
data6 = [0.0369, 0.0349, 0.0349;   % smooth
         0.6892, 0.6193, 0.6193;   % rough
         0.3276, 1.4857, 0.3299];  % pothole
b6 = bar(ax6, data6', 0.75);
b6(1).FaceColor = [0.20 0.63 0.17];
b6(2).FaceColor = [0.12 0.47 0.71];
b6(3).FaceColor = [0.89 0.10 0.11];
b6(1).EdgeColor = 'black'; b6(2).EdgeColor = 'black'; b6(3).EdgeColor = 'black';

set(ax6,'XTickLabel',controllers,'FontSize',fs-1,'FontName','Times New Roman')
ylabel(ax6,'RMS Body Acceleration (m/s²)','FontSize',fs,'FontName','Times New Roman','Color','black')
title(ax6,'RMS Body Acceleration Comparison','FontSize',fs+1,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg6 = legend(ax6,'Smooth Road','Rough Road','Pothole Road','Location','northwest','FontSize',fs-1);
fixAxes(ax6, fs-1); fixLegend(lg6); grid(ax6,'on'); ylim(ax6,[0 1.70]);
saveas(fig6,'Fig6_RMS_Body_Bar.png'); fprintf('✅ Fig6\n')

%% ================================================================
%% FIGURE 7 — Bar: Battery Degradation Index
%% ================================================================
fig7 = figure('Color','white','Position',[50 50 820 500]);
ax7 = axes(fig7);

data7 = [0.0087, 0.0078, 0.0078;   % smooth
         3.0442, 2.4700, 2.4700;   % rough
         0.6926, 11.4519, 0.7028]; % pothole
b7 = bar(ax7, data7', 0.75);
b7(1).FaceColor = [0.20 0.63 0.17];
b7(2).FaceColor = [0.12 0.47 0.71];
b7(3).FaceColor = [0.89 0.10 0.11];
b7(1).EdgeColor = 'black'; b7(2).EdgeColor = 'black'; b7(3).EdgeColor = 'black';

set(ax7,'XTickLabel',controllers,'FontSize',fs-1,'FontName','Times New Roman')
ylabel(ax7,'Battery Degradation Index','FontSize',fs,'FontName','Times New Roman','Color','black')
title(ax7,'Battery Degradation Index Comparison','FontSize',fs+1,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg7 = legend(ax7,'Smooth Road','Rough Road','Pothole Road','Location','northwest','FontSize',fs-1);
fixAxes(ax7, fs-1); fixLegend(lg7); grid(ax7,'on');
saveas(fig7,'Fig7_Degradation_Bar.png'); fprintf('✅ Fig7\n')

%% ================================================================
%% FIGURE 8 — Improvement %: Split scale
%% ================================================================
fig8 = figure('Color','white','Position',[50 50 1100 480]);

ax8a = subplot(1,2,1);
td3_cont = [5.40, 10.14];
hyb_cont = [5.40, 10.14];
b8a1 = bar(ax8a, [1 2]-0.2, td3_cont, 0.35, 'FaceColor', ct, 'EdgeColor','black'); hold(ax8a,'on')
b8a2 = bar(ax8a, [1 2]+0.2, hyb_cont, 0.35, 'FaceColor', ch, 'EdgeColor','black');
yline(ax8a, 0, 'k--', 'LineWidth', 1.5);
set(ax8a,'XTick',1:2,'XTickLabel',{'Smooth','Rough'},'FontSize',fs-1,'FontName','Times New Roman')
ylabel(ax8a,'Improvement vs Passive (%)','FontSize',fs,'FontName','Times New Roman','Color','black')
title(ax8a,'(a) Continuous Road Performance','FontSize',fs,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg8a = legend(ax8a,[b8a1 b8a2],{'TD3 Only','Hybrid TD3+Safety'},'FontSize',fs-1,'Location','northwest');
fixAxes(ax8a, fs-1); fixLegend(lg8a); grid(ax8a,'on'); ylim(ax8a,[-5 20]);

ax8b = subplot(1,2,2);
b8b1 = bar(ax8b, 1-0.2, -353.56, 0.35, 'FaceColor', ct, 'EdgeColor','black'); hold(ax8b,'on')
b8b2 = bar(ax8b, 1+0.2, -0.72,   0.35, 'FaceColor', ch, 'EdgeColor','black');
yline(ax8b, 0, 'k--', 'LineWidth', 1.5);
set(ax8b,'XTick',1,'XTickLabel',{'Pothole'},'FontSize',fs-1,'FontName','Times New Roman')
ylabel(ax8b,'Improvement vs Passive (%)','FontSize',fs,'FontName','Times New Roman','Color','black')
title(ax8b,'(b) Impulse Road Performance','FontSize',fs,'FontName','Times New Roman','FontWeight','bold','Color','black')
lg8b = legend(ax8b,[b8b1 b8b2],{'TD3 Only','Hybrid TD3+Safety'},'FontSize',fs-1,'Location','southwest');
fixAxes(ax8b, fs-1); fixLegend(lg8b); grid(ax8b,'on');

sgtitle('Performance Improvement over Passive Baseline','FontSize',fs+2,'FontName','Times New Roman','FontWeight','bold','Color','black')
saveas(fig8,'Fig8_Improvement_Percent.png'); fprintf('✅ Fig8\n')

fprintf('\n✅ ALL FIGURES COMPLETE.\n')