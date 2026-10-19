%% MTB DESIGN - LATERAL DYNAMICS -----------------------------------------
% Nomenclature:
%   R / F  : rear / front wheel bodies
%   B      : rear frame + rider (everything that does not steer or spin)
%   H      : front frame (fork, stem, bars, controls)
%   phi    : lean (roll) angle [rad],   delta : steer angle [rad]
%   W-frame: Whipple coordinates, origin at rear contact, x fwd, z DOWN [m]
%
% Purpose:
% Builds the 25 Whipple (Meijaard 2007) bicycle parameters from the frame
% geometry + component masses, then uses the HumanControl engine
% (whipple_pull_force_abcd, the function bicycle_state_space calls) to find
% the self-stable speed range, low-speed instability, weave frequency, and
% steady-turn handlebar torque.
% -------------------------------------------------------------------------


%% Houskeeping
% Clean up workspace
clc; clear; close all;


%% Inputs
% Bring in Geometry Data (must run first -- it clears the workspace)
run('MTBDesign_FrameGeometry.m');

% HumanControl location (folder containing whipple_pull_force_abcd.m)
hcPath = "/Users/danwanthj/Documents/Projects/MTB";
addpath(hcPath);

% Options
runBenchmarkCheck = true; % verify engine against Meijaard 2007 first
writeParFile = false; % also write <hcPath>/parameters/MTBDesignPar.txt

% Gravity
g = 9.81; % [m/s^2]

% Total Mass & CoM (KEEP IN SYNC WITH MTBDesign_Statics.m)
mRider = 82; % rider mass [kg]
mBike = 17; % projected bike mass [kg]
mT = mRider + mBike; % total mass [kg]
xG = 30; % total CoM from BB, horizontal; placeholder [mm]
yG = 750; % total CoM from BB, vertical; placeholder [mm]

% Rear Wheel R (DT E 1900 rear + DHR II 29x2.4 WT EXO+) -- all at rear axle
mRrim = 0.52; % rim [kg] (estimate; DT wheel total 1.126 kg)
mRspk = 0.20; % spokes + nipples [kg] (estimate)
mRhub = 0.406; % hub [kg] (wheel total - rim - spokes)
mRtire = 1.102; % DHR II 29x2.4 WT EXO+ [kg] (Maxxis)
mRseal = 0.10; % sealant [kg]
mRrot = 0.15; % 180 mm rotor [kg] (estimate)
mRcas = 0.45; % cassette [kg] (placeholder until custom cassette is designed)
hTRfree = 61; % rear tire unloaded height [mm] (ETRTO 61 x AR~1)

% Front Wheel F (DT E 1900 front + Assegai 29x2.5 WT EXO+) -- all at front axle
mFrim = 0.52; % rim [kg] (estimate; DT wheel total 1.005 kg)
mFspk = 0.20; % spokes + nipples [kg] (estimate)
mFhub = 0.285; % hub [kg] (wheel total - rim - spokes)
mFtire = 1.220; % Assegai 29x2.5 WT EXO+ [kg] (Maxxis)
mFseal = 0.10; % sealant [kg]
mFrot = 0.17; % 200 mm rotor [kg] (estimate)
hTFfree = 63; % front tire unloaded height [mm] (ETRTO 63 x AR~1)

% Front Frame H (Fox 36 GRIP X2 + cockpit)
mFork = 2.00; % fork incl. axle + steerer [kg] (estimate; weigh it)
mStem = 0.15; % stem [kg]
mBar = 0.30; % handlebar [kg]
mCtrl = 0.30; % grips + brake lever + remotes [kg]
mCal = 0.13; % front brake caliper [kg]
Lbar = 780; % bar width [mm]
bFork = 65; % half-spacing of fork legs (lateral) [mm]
Lstem = 45; % stem length [mm]
sStem = 30; % steerer stack above head tube to stem clamp [mm]
barRise = 20; % bar rise [mm]

% Body B inertia: benchmark rider + frame scaled by mass (PLACEHOLDER --
% largest uncertainty; replace with rider segment model + NX frame data)
IBbench = [9.2, 11, 2.8, 2.4]; % [Ixx Iyy Izz Ixz] for mB = 85 kg [kg m^2]

% Speed Sweep
v = linspace(0.1, 12, 600); % forward speeds [m/s]

% Steady-Turn Cases
vTurn = [1 2 3 4 5 6 8]; % speeds [m/s]
RTurn = [5 10 20]; % turn radii [m]


%% Benchmark Check (verify the engine before trusting results)
if runBenchmarkCheck
    parBench = struct('w',1.02,'c',0.08,'lam',pi/10,'g',9.81,'v',1, ...
        'rR',0.3,'mR',2,'IRxx',0.0603,'IRyy',0.12, ...
        'xB',0.3,'zB',-0.9,'mB',85,'IBxx',9.2,'IByy',11,'IBzz',2.8,'IBxz',2.4, ...
        'xH',0.9,'zH',-0.7,'mH',4,'IHxx',0.05892,'IHyy',0.06,'IHzz',0.00708, ...
        'IHxz',-0.00756,'rF',0.35,'mF',3,'IFxx',0.1405,'IFyy',0.28, ...
        'xcl',0.3,'zcl',-0.9);
    eigRef = [-14.0784; -0.7753+4.4649i; -0.7753-4.4649i; -0.3229]; % Meijaard Table 2, v = 5 m/s
    eigB = eig(lateralSystem(parBench, 5));
    errEig = max(arrayfun(@(r) min(abs(eigB - r)), eigRef));
    [vwB, vcB] = criticalSpeeds(parBench, linspace(0.1, 10, 400));
    benchOK = errEig < 1e-3 && abs(vwB - 4.2924) < 1e-3 && abs(vcB - 6.0243) < 1e-3;
end


%% Whipple Parameters (convert geometry: mm, BB origin, y up -> m, rear contact, z down)
toW = @(p) [(p(1) - RPos(1)) / 1000; -(p(2) + hBB) / 1000]; % point -> W-frame [m]
rR = RR / 1000; % loaded rear wheel radius [m]
rF = RF / 1000; % loaded front wheel radius [m]
lam = deg2rad(90 - theta); % steer axis tilt from vertical [rad]
eSteer = [sin(lam); cos(lam)]; % unit vector down steer axis, W-frame (x, z)

% Rear Wheel R (thin rings about the axle)
rRtire = (rimRad + 0.75 * hTRfree) / 1000; % tire mass radius [m]
mR = mRrim + mRspk + mRhub + mRtire + mRseal + mRrot + mRcas; % [kg]
IRyy = mRrim * (rimRad/1000)^2 + mRspk * (rimRad/1000)^2 / 3 ...
     + mRtire * rRtire^2 + mRseal * (0.33)^2 + mRrot * (0.08)^2 + mRcas * (0.035)^2; % spin [kg m^2]
IRxx = IRyy / 2; % about a diameter (planar wheel) [kg m^2]

% Front Wheel F
rFtire = (rimRad + 0.75 * hTFfree) / 1000; % tire mass radius [m]
mF = mFrim + mFspk + mFhub + mFtire + mFseal + mFrot; % [kg]
IFyy = mFrim * (rimRad/1000)^2 + mFspk * (rimRad/1000)^2 / 3 ...
     + mFtire * rFtire^2 + mFseal * (0.33)^2 + mFrot * (0.09)^2; % spin [kg m^2]
IFxx = IFyy / 2; % [kg m^2]

% Front Frame H (parts: [mass, xW, zW, Ixx, Iyy, Izz, Ixz] about own CoM)
forkCG = C + d * n + 0.45 * A2C * u; % fork CoM: 45% down stanchion line [mm]
stemPt = Htop - sStem * u + [Lstem/2; 0]; % stem CoM [mm]
barPt = Htop - sStem * u + [Lstem; barRise]; % bar center [mm]
calPt = FPos + [-60; 70]; % caliper (post mount behind leg) [mm]
Lfk = A2C / 1000; % fork length as slender rod [m]
IforkRod = mFork * Lfk^2 / 12; % rod inertia about its CoM [kg m^2]
Ispread = mFork * (bFork/1000)^2; % two-leg lateral spread [kg m^2]
partsH = [ ...
  mFork, toW(forkCG)', IforkRod*eSteer(2)^2 + Ispread, IforkRod, IforkRod*eSteer(1)^2 + Ispread, -IforkRod*eSteer(1)*eSteer(2);
  mStem, toW(stemPt)', 0, 0, 0, 0;
  mBar,  toW(barPt)',  mBar*(Lbar/1000)^2/12, 0, mBar*(Lbar/1000)^2/12, 0;
  mCtrl, toW(barPt)',  mCtrl*(Lbar/2000)^2,   0, mCtrl*(Lbar/2000)^2,   0;
  mCal,  toW(calPt)',  0, 0, 0, 0];
H = combineParts(partsH);

% Rear Frame + Rider B = Total - R - F - H
xyT = toW([xG; yG]); % total CoM, W-frame [m]
mB = mT - mR - mF - H.m; % [kg]
xB = (mT * xyT(1) - mR * 0 - mF * (w/1000) - H.m * H.x) / mB; % [m]
zB = (mT * xyT(2) - mR * (-rR) - mF * (-rF) - H.m * H.z) / mB; % [m]
IB = IBbench * mB / 85; % scaled placeholder [kg m^2]

% Parameter Struct (Meijaard 2007 names)
par = struct('w', w/1000, 'c', c/1000, 'lam', lam, 'g', g, 'v', 1, ...
    'rR', rR, 'mR', mR, 'IRxx', IRxx, 'IRyy', IRyy, ...
    'xB', xB, 'zB', zB, 'mB', mB, 'IBxx', IB(1), 'IByy', IB(2), 'IBzz', IB(3), 'IBxz', IB(4), ...
    'xH', H.x, 'zH', H.z, 'mH', H.m, 'IHxx', H.Ixx, 'IHyy', H.Iyy, 'IHzz', H.Izz, 'IHxz', H.Ixz, ...
    'rF', rF, 'mF', mF, 'IFxx', IFxx, 'IFyy', IFyy, ...
    'xcl', xB, 'zcl', zB); % lateral force point (unused; set to silence warning)

% Optional: parameter file so bicycle_state_space('MTBDesign', v, ...) also works
if writeParFile
    fid = fopen(fullfile(hcPath, 'parameters', 'MTBDesignPar.txt'), 'w');
    names = fieldnames(par);
    for k = 1:numel(names)
        fprintf(fid, '%s=%.10g\n', names{k}, par.(names{k}));
    end
    fclose(fid);
end


%% Lateral Dynamics
% Eigenvalues vs Speed
data.eig = zeros(4, numel(v));
for k = 1:numel(v)
    data.eig(:, k) = eig(lateralSystem(par, v(k)));
end
data.maxRe = max(real(data.eig), [], 1); % worst-case growth rate [1/s]

% Critical Speeds
[data.vWeave, data.vCapsize] = criticalSpeeds(par, v);

% Weave Frequency at Weave Speed
if ~isnan(data.vWeave)
    eW = eig(lateralSystem(par, data.vWeave));
    data.fWeave = max(abs(imag(eW))) / (2*pi); % [Hz]
else
    data.fWeave = NaN;
end

% Low-Speed Instability (Astrom rideability: growth rate < ~1 rad/s)
lowIdx = v >= 2 & v <= 4;
data.lowMaxRe = max(data.maxRe(lowIdx)); % [1/s]
data.rideableLow = data.lowMaxRe < 1;

% Steady-Turn Handlebar Torque  (q_dd = q_d = 0  ->  K q = [0; T])
data.turn.v = vTurn;
data.turn.TperDelta = zeros(size(vTurn)); % T/delta [N m/rad]
data.turn.phiPerDelta = zeros(size(vTurn)); % phi/delta [-]
data.turn.T = zeros(numel(vTurn), numel(RTurn)); % torque for each radius [N m]
data.turn.phi = zeros(numel(vTurn), numel(RTurn)); % lean for each radius [deg]
for k = 1:numel(vTurn)
    [~, ~, K] = lateralSystem(par, vTurn(k));
    data.turn.TperDelta(k) = det(K) / K(1,1);
    data.turn.phiPerDelta(k) = -K(1,2) / K(1,1);
    deltaR = (par.w ./ (RTurn * cos(lam))); % steer angle for radius (small angle) [rad]
    data.turn.T(k, :) = data.turn.TperDelta(k) * deltaR;
    data.turn.phi(k, :) = rad2deg(data.turn.phiPerDelta(k) * deltaR);
end


%% Display Results
ok = {'FAIL', 'ok'}; % flag labels, indexed by logical+1

fprintf('\n=============== LATERAL DYNAMICS ===============\n');
if runBenchmarkCheck
    fprintf('Benchmark check: eig err %.1e, weave %.4f, capsize %.4f m/s  -> %s\n', ...
        errEig, vwB, vcB, ok{benchOK+1});
end

fprintf('\n--- Whipple parameters (SI) ---\n');
fprintf('w = %.4f m, c = %.4f m, lam = %.2f deg, rR = %.4f m, rF = %.4f m\n', ...
    par.w, par.c, rad2deg(lam), rR, rF);
fprintf('%-3s %8s %8s %8s %9s %9s %9s %9s\n', 'Bdy', 'm[kg]', 'x[m]', 'z[m]', ...
    'Ixx', 'Iyy', 'Izz', 'Ixz');
fprintf('%-3s %8.3f %8.4f %8.4f %9.4f %9.4f %9.4f %9s\n', 'R', mR, 0, -rR, IRxx, IRyy, IRxx, '-');
fprintf('%-3s %8.3f %8.4f %8.4f %9.4f %9.4f %9.4f %9.4f\n', 'B', mB, xB, zB, IB(1), IB(2), IB(3), IB(4));
fprintf('%-3s %8.3f %8.4f %8.4f %9.5f %9.5f %9.5f %9.5f\n', 'H', H.m, H.x, H.z, H.Ixx, H.Iyy, H.Izz, H.Ixz);
fprintf('%-3s %8.3f %8.4f %8.4f %9.4f %9.4f %9.4f %9s\n', 'F', mF, par.w, -rF, IFxx, IFyy, IFxx, '-');

fprintf('\n--- Stability ---\n');
fprintf('Weave speed (stable above):    %6.2f m/s (%5.1f km/h)\n', data.vWeave, 3.6*data.vWeave);
fprintf('Capsize speed (unstable above): %6.2f m/s (%5.1f km/h)\n', data.vCapsize, 3.6*data.vCapsize);
fprintf('Weave frequency at weave speed: %6.2f Hz\n', data.fWeave);
fprintf('Max growth rate, 2-4 m/s:      %6.2f 1/s  -> rideable (<1): %s\n', ...
    data.lowMaxRe, ok{data.rideableLow+1});

fprintf('\n--- Steady turn (negative T = push bar OUT of turn) ---\n');
fprintf('%6s %10s %8s', 'v[m/s]', 'T/d[Nm/r]', 'phi/d');
fprintf('   T@R=%-4.0f', RTurn); fprintf('  phi@R=%-3.0f', RTurn); fprintf('\n');
for k = 1:numel(vTurn)
    fprintf('%6.1f %10.2f %8.2f', vTurn(k), data.turn.TperDelta(k), data.turn.phiPerDelta(k));
    fprintf('   %8.2f', data.turn.T(k, :));
    fprintf('   %8.1f', data.turn.phi(k, :));
    fprintf('\n');
end
fprintf('(T in N m, phi in deg; small-angle -- treat tight/slow cases as trends)\n');
fprintf('================================================\n');


%% Plots
% Eigenvalues vs Speed
figure('Name', 'Lateral Dynamics - Eigenvalues'); hold on; grid on; box on;
yl = [-10 10];
if ~isnan(data.vWeave)
    vHi = data.vCapsize; if isnan(vHi), vHi = v(end); end
    patch([data.vWeave vHi vHi data.vWeave], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.85 0.95 0.88], 'EdgeColor', 'none', 'HandleVisibility', 'off');
end
V = repmat(v, 4, 1);
plot(V(:), real(data.eig(:)), 'k.', 'MarkerSize', 4, 'DisplayName', 'Re(\lambda)');
imPos = imag(data.eig); imPos(imPos <= 0) = NaN;
plot(V(:), imPos(:), '.', 'Color', [0.2 0.4 0.8], 'MarkerSize', 4, 'DisplayName', 'Im(\lambda)');
plot(v, zeros(size(v)), 'k-', 'HandleVisibility', 'off');
plot(v, ones(size(v)), 'r:', 'DisplayName', 'Rideability limit');
ylim(yl); xlim([0 v(end)]);
xlabel('Forward speed v [m/s]'); ylabel('Eigenvalue [1/s]');
title(sprintf('Self-stable %.2f - %.2f m/s', data.vWeave, data.vCapsize));
legend('Location', 'southwest');

% Steady-Turn Torque vs Speed
figure('Name', 'Lateral Dynamics - Steady Turn'); hold on; grid on; box on;
vFine = linspace(0.5, 10, 200);
TdFine = zeros(size(vFine));
for k = 1:numel(vFine)
    [~, ~, K] = lateralSystem(par, vFine(k));
    TdFine(k) = det(K) / K(1,1);
end
plot(vFine, TdFine, 'k-', 'LineWidth', 1.5);
plot(vFine, zeros(size(vFine)), 'k:');
xlabel('Forward speed v [m/s]'); ylabel('Steady-turn torque T/\delta [N m/rad]');
title('Handlebar torque to hold a turn (negative = push out)');


%% Local Functions -------------------------------------------------------
function [A, M, K, C] = lateralSystem(par, speed)
% Lateral (phi, delta) state-space model from the HumanControl engine.
% x = [phi; delta; phiDot; deltaDot],  u = [tPhi; tDelta]
% Also recovers M, K = g*K0 + v^2*K2, C = v*C1 from A and B.
[Afull, Bfull] = whipple_pull_force_abcd(par, speed);
k = [4 7 9 11]; % phi, delta, phiDot, deltaDot in the full 11-state model
A = Afull(k, k);
Bl = Bfull(k, 1:2); % inputs: tPhi, tDelta
M = inv(Bl(3:4, :)); % lower block of B is M^-1
K = -M * A(3:4, 1:2);
C = -M * A(3:4, 3:4);
end

function [vW, vC] = criticalSpeeds(par, v)
% Weave speed (max Re goes + to -) and capsize speed (- to +).
f = @(s) max(real(eig(lateralSystem(par, s))));
mr = arrayfun(f, v);
vW = NaN; vC = NaN;
for k = 1:numel(v) - 1
    if isnan(vW) && mr(k) > 0 && mr(k+1) <= 0
        vW = fzero(f, [v(k) v(k+1)]);
    elseif ~isnan(vW) && mr(k) <= 0 && mr(k+1) > 0
        vC = fzero(f, [v(k) v(k+1)]);
        break
    end
end
end

function S = combineParts(P)
% Combine parts [m, x, z, Ixx, Iyy, Izz, Ixz] (own-CoM inertias, W-frame)
% into one body via parallel-axis theorem. Ixz uses the -integral(x z dm)
% convention of Meijaard 2007 (z down).
S.m = sum(P(:,1));
S.x = sum(P(:,1) .* P(:,2)) / S.m;
S.z = sum(P(:,1) .* P(:,3)) / S.m;
dx = P(:,2) - S.x; dz = P(:,3) - S.z;
S.Ixx = sum(P(:,4) + P(:,1) .* dz.^2);
S.Iyy = sum(P(:,5) + P(:,1) .* (dx.^2 + dz.^2));
S.Izz = sum(P(:,6) + P(:,1) .* dx.^2);
S.Ixz = sum(P(:,7) - P(:,1) .* dx .* dz);
end
