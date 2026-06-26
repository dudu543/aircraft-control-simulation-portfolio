clear; clc; close all;

figDir = "figures";
if ~exist(figDir, "dir")
    mkdir(figDir);
end

cases = [
    struct("initialHeadingDeg", 0, "targetManeuver", false)
    struct("initialHeadingDeg", 10, "targetManeuver", false)
    struct("initialHeadingDeg", -10, "targetManeuver", false)
    struct("initialHeadingDeg", 0, "targetManeuver", true)
    struct("initialHeadingDeg", 10, "targetManeuver", true)
];

results = cell(length(cases), 1);

for i = 1:length(cases)
    results{i} = simulatePN(cases(i).initialHeadingDeg, cases(i).targetManeuver);
end

plotTrajectory(results, figDir);
plotLOSRate(results, figDir);
plotMissDistance(results, figDir);

disp("PN guidance simulation completed.");
for i = 1:length(results)
    r = results{i};
    fprintf("heading = %6.1f deg, maneuver = %d, miss distance = %.2f m\n", ...
        r.initialHeadingDeg, r.targetManeuver, r.missDistance);
end


function result = simulatePN(initialHeadingDeg, targetManeuver)
    dt = 0.01;
    tFinal = 40.0;

    Vm = 300.0;
    Vt = 180.0;
    N = 4.0;

    missilePos = [0.0; 0.0];
    targetPos = [5000.0; 1000.0];

    gammaM = deg2rad(initialHeadingDeg);
    gammaT = deg2rad(180.0);

    maxSteps = floor(tFinal / dt);

    missileHistory = zeros(maxSteps, 2);
    targetHistory = zeros(maxSteps, 2);
    losHistory = zeros(maxSteps, 1);
    losRateHistory = zeros(maxSteps, 1);
    rangeHistory = zeros(maxSteps, 1);
    timeHistory = zeros(maxSteps, 1);

    prevLOS = NaN;
    missDistance = NaN;
    lastStep = maxSteps;

    for k = 1:maxSteps
        t = (k - 1) * dt;

        relativePos = targetPos - missilePos;
        R = norm(relativePos);

        if R < 5.0
            missDistance = R;
            lastStep = k;
            break;
        end

        lambda = atan2(relativePos(2), relativePos(1));

        if isnan(prevLOS)
            lambdaDot = 0.0;
        else
            lambdaDot = wrapAngle(lambda - prevLOS) / dt;
        end

        missileVel = Vm * [cos(gammaM); sin(gammaM)];
        targetVel = Vt * [cos(gammaT); sin(gammaT)];

        relativeVel = targetVel - missileVel;
        closingVelocity = -dot(relativePos, relativeVel) / R;

        lateralAccCmd = N * closingVelocity * lambdaDot;

        gammaM = gammaM + (lateralAccCmd / Vm) * dt;

        if targetManeuver
            targetAcc = 20.0 * sin(0.7 * t);
            gammaT = gammaT + (targetAcc / Vt) * dt;
        end

        missileVel = Vm * [cos(gammaM); sin(gammaM)];
        targetVel = Vt * [cos(gammaT); sin(gammaT)];

        missilePos = missilePos + missileVel * dt;
        targetPos = targetPos + targetVel * dt;

        missileHistory(k, :) = missilePos.';
        targetHistory(k, :) = targetPos.';
        losHistory(k) = lambda;
        losRateHistory(k) = lambdaDot;
        rangeHistory(k) = R;
        timeHistory(k) = t;

        prevLOS = lambda;
    end

    if isnan(missDistance)
        missDistance = R;
    end

    idx = 1:lastStep;

    result.missile = missileHistory(idx, :);
    result.target = targetHistory(idx, :);
    result.los = losHistory(idx);
    result.losRate = losRateHistory(idx);
    result.range = rangeHistory(idx);
    result.time = timeHistory(idx);
    result.missDistance = missDistance;
    result.initialHeadingDeg = initialHeadingDeg;
    result.targetManeuver = targetManeuver;
end


function plotTrajectory(results, figDir)
    figure("Color", "w", "Position", [100, 100, 1150, 520]);
    tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

    plotTrajectoryPanel(results, false, "Straight Target");
    plotTrajectoryPanel(results, true, "Maneuvering Target");

    sgtitle("PN Guidance Interception Trajectories");
    prepareFigureForExport(gcf);
    exportgraphics(gcf, fullfile(figDir, "trajectory.png"), "Resolution", 200);
    close(gcf);
end


function plotTrajectoryPanel(results, targetManeuver, panelTitle)
    nexttile;
    hold on;
    colors = lines(length(results));

    for i = 1:length(results)
        r = results{i};
        if r.targetManeuver ~= targetManeuver
            continue;
        end

        label = sprintf("\\gamma_0 = %+d deg", r.initialHeadingDeg);

        plot(r.missile(:, 1), r.missile(:, 2), "LineWidth", 1.8, ...
            "Color", colors(i, :), "DisplayName", "Missile, " + label);
        plot(r.target(:, 1), r.target(:, 2), "--", "LineWidth", 1.2, ...
            "Color", colors(i, :), "DisplayName", "Target, " + label);
    end

    xlabel("X Position (m)");
    ylabel("Y Position (m)");
    title(panelTitle);
    axis equal;
    grid on;
    legend("Location", "best", "FontSize", 8);
end


function plotLOSRate(results, figDir)
    figure("Color", "w", "Position", [100, 100, 900, 450]);
    hold on;
    colors = lines(length(results));

    for i = 1:length(results)
        r = results{i};

        if r.targetManeuver
            targetType = "Maneuver";
        else
            targetType = "Straight";
        end

        label = sprintf("\\gamma_0 = %+d deg, %s", r.initialHeadingDeg, targetType);
        plot(r.time, rad2deg(r.losRate), "LineWidth", 1.5, ...
            "Color", colors(i, :), "DisplayName", label);
    end

    xlabel("Time (s)");
    ylabel("LOS Rate (deg/s)");
    title("Line-of-Sight Rate Comparison");
    grid on;
    legend("Location", "northeast", "FontSize", 8);

    prepareFigureForExport(gcf);
    exportgraphics(gcf, fullfile(figDir, "los_rate.png"), "Resolution", 200);
    close(gcf);
end


function plotMissDistance(results, figDir)
    labels = strings(length(results), 1);
    values = zeros(length(results), 1);
    colors = zeros(length(results), 3);

    for i = 1:length(results)
        r = results{i};

        if r.targetManeuver
            targetType = "Maneuver";
            colors(i, :) = [0.85, 0.33, 0.10];
        else
            targetType = "Straight";
            colors(i, :) = [0.00, 0.45, 0.74];
        end

        labels(i) = sprintf("%+d deg | %s", r.initialHeadingDeg, targetType);
        values(i) = r.missDistance;
    end

    figure("Color", "w", "Position", [100, 100, 950, 460]);

    b = bar(values, "FaceColor", "flat");
    b.CData = colors;
    set(gca, "XTick", 1:length(labels), "XTickLabel", labels);
    xtickangle(20);
    ylabel("Miss Distance (m)");
    title("Miss Distance Comparison");
    grid on;
    ylim([0, max(values) * 1.25]);

    for i = 1:length(values)
        text(i, values(i), sprintf("%.2f m", values(i)), ...
            "HorizontalAlignment", "center", ...
            "VerticalAlignment", "bottom", ...
            "FontSize", 9);
    end

    prepareFigureForExport(gcf);
    exportgraphics(gcf, fullfile(figDir, "miss_distance_compare.png"), "Resolution", 200);
    close(gcf);
end


function prepareFigureForExport(fig)
    set(fig, "Toolbar", "none");
    set(fig, "MenuBar", "none");

    axesList = findall(fig, "Type", "axes");
    for i = 1:length(axesList)
        try
            axtoolbar(axesList(i), {});
            disableDefaultInteractivity(axesList(i));
        catch
            % Older MATLAB releases may not support these graphics helpers.
        end
    end

    drawnow;
end


function angle = wrapAngle(angle)
    angle = atan2(sin(angle), cos(angle));
end
