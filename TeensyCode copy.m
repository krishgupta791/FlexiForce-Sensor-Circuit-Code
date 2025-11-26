%% Teensy Force Sensor Live Plot (macOS version)
% ***** HAVE TO CLEAR AND CLC AFTER EVERY PROGRAM RUN TO RESET SERIAL MONITOR OR CODE WILL ERROR ******
clear;
clc;
%% Teensy Force Sensor Live Plot

port = "/dev/cu.usbmodem123419501";    % <-- Replace XXXX with your Teensy's number
                                            % Teensyduino --> Tools --> Port
baud = 115200;                          % Baud rate (matches Serial.begin() in Teensy setup)

% --- Clean up any old connections ---
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

% --- Set up serial connection ---
s = serialport(port, baud);
configureTerminator(s, "LF");  % Creates new line after each cycle of data recorded
flush(s);                      % Clears any old data or text

% --- Live plotting setup ---
figure('Name','Force Sensor Live Plot','NumberTitle','off');
h1 = animatedline('Color',[0 0.4 1],'LineWidth',1.5);       % raw voltage signal (blue)
h2 = animatedline('Color',[1 0.4 1],'LineWidth',1.5);       % cleaned voltage signal (pink)

xlabel('Time (s)');
ylabel('Voltage (V)');
title('Live Voltage Readings from Teensy');
grid on;

legend('Raw Voltage', 'Cleaned Voltage');

t = [];
v = [];

% --- Start streaming ---
tstart = tic;              % start timer to plot live readings
while ishandle(h1)         % checks to see if plot is open (true) or has been deleted (false)
    try
        % --- Raw Voltage ---
        vLine = readline(s);                % read one line from Teensy
        volts = str2double(strtrim(vLine)); % convert text to voltage
        if isnan(volts)
            continue;                      % skips line if voltage recorded is not a number
        end
        addpoints(h1, toc(tstart), volts); % plot new point
        drawnow limitrate;                 % update plot
        %  --- Cleaned Voltage ---
        line2 = readline(s);               % read next line from Teensy
        avg = str2double(strtrim(line2));  % convert text to voltage
        if isnan(avg)
            continue;                      % skips line if voltage recorded is not a number
        end
        addpoints(h2, toc(tstart), avg);   % plot new point
        drawnow limitrate;     

        if toc(tstart) <= 5                % plot first 5 seconds of recorded data
            t(end+1) = toc(tstart);
            v(end+1) = avg;
        else
            break;
        end
    catch
        print('Teensy disconnected ora bad input\n');
        break;                             % stop on error/port closed
    end
end

% --- Line of Best Fit (LOBF) calculation using cleaned signal
coeffs = polyfit(t, v, 1)
lobf = polyval(coeffs, t);
hold on;
plot(t, lobf, 'Color', [0 0 0], 'LineWidth', 1.5, 'DisplayName', sprintf('LOBF: %.2gx + %.2g', coeffs(1), coeffs(2)));
hold off;

%writematrix(v, 'Scissors3.csv');          % saving recorded data as .csv

%% Voltage-Mass Plot

% M = mass (kg); V = average voltage (V)

% hard coding (mass, voltage) points obtained from running section above 3x
% per object

phoneM = 0.285;
phoneV = 0.07783;

toolM = 0.02;
toolV = 0.00557;

ekM = 0.341;
ekV = 0.16293;

plungerM = 0;
plungerV = 0.000696;

glassesM = 0.18;
glassesV = 0.0758;

walletM = 0.27;
walletV = 0.1211;

tapeM = 0.034;
tapeV = 0.01387;

mass = [phoneM toolM ekM plungerM glassesM walletM tapeM];      % grouping all mass points
voltage = [phoneV toolV ekV plungerV glassesV walletV tapeV];   % grouping all voltage points

labels = {'iPhone14', 'Tool', 'EK', 'Plunger', 'Sunglasses Case', 'Wallet', 'Tape'};

scatter(mass, voltage, 'filled', 'blue', 'HandleVisibility','off'); % plotting scatter plot

for i = 1:length(mass)  % adding labels to each point for clarity
    if i == 6       % manually adjusting labels to be clearly seen
        text(mass(i)-0.0275, voltage(i)+0.003, labels{i}, 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
    elseif i == 3   % manually adjusting labels to be clearly seen
        text(mass(i)-0.015, voltage(i)+0.003, labels{i}, 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
    else
        text(mass(i)+0.005, voltage(i)-0.003, labels{i}, 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
    end
end
hold on;

% --- creating linear equation for scatter plot (LOBF) ---
p = polyfit(mass, voltage, 1);
trend = polyval(p, mass);

% -- calculating R^2 value
yresid = voltage - trend;
SSresid = sum(yresid.^2);
SStotal = (length(voltage)-1) * var(voltage);
rsq = 1 - SSresid/SStotal;

eq = sprintf('V = %.4fm - %.4f\nR^2 = %.4f', p(1), abs(p(2)), rsq); % LOBF equation

plot(mass, trend, 'LineWidth', 2, 'DisplayName', eq, 'Color', 'k');

% --- additional objects to test accuracy of scatter plot LOBF ---

% AV = actual voltage (V) (measured); PV = predicted voltage (V) (from LOBF)

mouseM = 0.132;
mouseAV = 0.0627;
mousePV = 0.05315;

tacksM = 0.071;
tacksAV = 0.00463;
tacksPV = 0.0278;

cardsM = 0.112;
cardsAV = 0.0111;
cardsPV = 0.0444;

scissorsM = 0.050;
scissorsAV = 0.0188;
scissorsPV = 0.0190;

% -- plotting AV as circles and PV as squares ---

plot(mouseM, mouseAV, 'ro', 'DisplayName', 'Mouse (A)');
plot(mouseM, mousePV, 'rs', 'DisplayName', 'Mouse (P)');
plot(tacksM, tacksAV, 'go', 'DisplayName', 'Tacks (A)');
plot(tacksM, tacksPV, 'gs', 'DisplayName', 'Tacks (P)');
plot(cardsM, cardsAV, 'mo', 'DisplayName', 'Cards (A)');
plot(cardsM, cardsPV, 'ms', 'DisplayName', 'Cards (P)');
plot(scissorsM, scissorsAV, 'bo', 'DisplayName', 'Scissors (A)');
plot(scissorsM, scissorsPV, 'bs', 'DisplayName', 'Scissors (P)');

hold off;

xlabel('Mass (kg)');
ylabel('Voltage (V)');
title('Modeling FlexiForce Sensor Function (Voltage Vs. Mass)');
legend('Location', 'northwest');

%% Force-Voltage plot (same steps as Voltage-Mass plot)

force = mass*9.81;

labels = {'iPhone14', 'Tool', 'EK', 'Plunger', 'Sunglasses Case', 'Wallet', 'Tape'};

scatter(voltage, force, 'filled', 'blue', 'HandleVisibility','off');
for i = 1:length(force)
    if i == 4
        text(voltage(i)+0.002, force(i)+0.075, labels{i}, 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
    else
        text(voltage(i)+0.002, force(i), labels{i}, 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
    end
end
hold on;

p = polyfit(voltage, force, 1);
trend = polyval(p, voltage);

yresid = force - trend;
SSresid = sum(yresid.^2);
SStotal = (length(force)-1) * var(force);
rsq = 1 - SSresid/SStotal;      % R^2 value

eq = sprintf('F = %.4fV + %.4f\nR^2 = %.4f', p(1), p(2), rsq);  % LOBF equation

plot(voltage, trend, 'LineWidth', 2, 'DisplayName', eq, 'Color', 'k');

% AF = actual force (N) (measured); PF = predicted force (N) (from LOBF)

mouseAF = 0.132 * 9.81;
mouseV = 0.0627;
mousePF = 1.526;

tacksAF = 0.071 * 9.81;
tacksV = 0.00463;
tacksPF = 0.2848;

cardsAF = 0.112 * 9.81;
cardsV = 0.0111;
cardsPF = 0.4231;

scissorsAF = 0.050 * 9.81;
scissorsV = 0.0188;
scissorsPF = 0.5876;

plot(mouseV, mouseAF, 'ro', 'DisplayName', 'Mouse (A)');
plot(mouseV, mousePF, 'rs', 'DisplayName', 'Mouse (P)');
plot(tacksV, tacksAF, 'go', 'DisplayName', 'Tacks (A)');
plot(tacksV, tacksPF, 'gs', 'DisplayName', 'Tacks (P)');
plot(cardsV, cardsAF, 'mo', 'DisplayName', 'Cards (A)');
plot(cardsV, cardsPF, 'ms', 'DisplayName', 'Cards (P)');
plot(scissorsV, scissorsAF, 'bo', 'DisplayName', 'Scissors (A)');
plot(scissorsV, scissorsPF, 'bs', 'DisplayName', 'Scissors (P)');

hold off;

xlabel('Voltage (V)');
ylabel('Force (N)');
title('Modeling FlexiForce Sensor Function (Force Vs. Voltage)');
legend('Location', 'northwest');
