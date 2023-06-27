clc, close all, clear all
open = 0;
warning('off','serialport:serialport:ReadlineWarning')
validEntry = 0;
while(~validEntry)
    prompt = "Emulate or Real test('E' or 'R'): ";
    type = input(prompt,"s");
    if type == "E"
        emulate = 1;
        validEntry = 1;
    elseif type == "R"
        emulate = 0;
        validEntry = 1;
    else
        fprintf('Invalid Entry\n');
    end
end
%% Auto. finds the arduino and connects to it
if ~emulate
    fprintf("\nAttempting to connect...(Approx 30 sec)\n")
    dev_name = 'Arduino Uno';
    [~,res]=system('wmic path Win32_SerialPort');
    ind = strfind(res,dev_name);
    if (~isempty(ind))
        port_name = res(ind(1)+length(dev_name)+2:ind(1)+length(dev_name)+6);
        port_name = strip(port_name);
        port_name = strip(port_name,'right',')'); %handling for the port being 2 digits
        fprintf('%s found on COM-port: %s\n',dev_name, port_name);

        try
            arduino = serialport(port_name,115200); %%The arduino object at 115200 baud
            fprintf('%s is opened\n',port_name);
            open = 1;
        catch err
            fprintf('%s\n%s\n',err.identifier,err.message);
        end
    else
        fprintf('COM-port is not found\nPlug in device, manually enter COM Port, or Emulate the test\n');
        open = 0;
    end
end

%% Manually connects to Arduino
if open == 0 && ~emulate
    arduino = serialport("COM23",115200); %Replace 'COM23' with your location
    open = 1;
    fprintf("Manual Opening Successful\n");
end
%% emualte import
if emulate
    dataE = importdata("TwistAngleTorqueSampleData_pla_11.68E_May-16-2023.mat","dataNice");
end
%% Data collection
if open || emulate
    loadCellE = 0;
    if ~emulate
        arduino.Timeout = 10;
    end
    angle = []; %angle vector
    torque =[]; %torque vector
    dataV = [];
    a1=[];
    ready = 0;
    materialIn = "\nWhat is the material: ";
    material = input(materialIn,"s");
    diaIn = "\nWhat is the diameter in mm: ";
    input_d = str2double(input(diaIn,"s"));

    if emulate
        ready = 1;
    else
        fprintf("-=-=-=-=-=-=-=-=-=-=-=-=-\nFollow prompts on LCD\n");
    end
    
    while ~ready
        writeline(arduino,"MATLAB");
        data = readline(arduino);
        if ~isempty(data)
            data = strip(data);
            if data == "ready"
                ready = 1;
            end
        end
    end
    done = 0;
    if ~emulate
        prompt = "Press enter to start test";
        type = input(prompt,"s");
        writeline(arduino,"start");
    end
    livePlot = animatedline;
    title("Torque vs. Angle of Twist", 'FontSize', 18, 'FontName', 'Times')
    xlabel("Angle of Twist (deg)", 'FontSize', 14, 'FontName', 'Times')
    ylabel("Torque (N-m)", 'FontSize', 14, 'FontName', 'Times')
    while ~done
        data = "";
        if ~emulate
            data = readline(arduino);
            if isempty(data)
                data = "";
                if length(dataV)>1
                    done = 1;
                end
            end
        end
        
        if strlength(data) > 0 && ~done && ~emulate
            data = strip(data);
            if data == "EC"
                fprintf("\nCAPACITY EXCEEDED!\n");
                done = 1;
            elseif data == "LC"
                loadCellE = 1;
                fprintf("\n*Warning: Load Cell Error Detected*\n");
            elseif data == "B"
                done = 1;
            else
                dataV = [dataV;data];
                newStr = split(data,':');
                angle = [angle;str2double(newStr(1))];
                torque = [torque;str2double(newStr(2))];
                xlim([0 max(angle)+5]);
                ylim([0 max(torque)+.2]);
                addpoints(livePlot,angle(end),torque(end));
                dim = [0.15 0.6 0.3 0.3];
                str = {"Angle = "+ sprintf('%.2f',angle(end)) + " deg", "Torque = "+ sprintf('%.2f',torque(end)) + " N-m"};
                delete(a1);
                a1 = annotation('textbox',dim,'string',str,'FitBoxToText','on', 'FontSize', 12, 'FontName', 'Times');
                drawnow;
                %fprintf(['Angle: %s' char(176) '\tTorque: %sNm\n'],newStr(1),newStr(2));
            end
        end

        if emulate
            for c=1:length(dataE)
                angle = [angle;dataE(c,1)];
                torque = [torque;dataE(c,2)];
                xlim([0 max(angle)+5]);
                ylim([0 max(torque)+.2]);
                addpoints(livePlot,angle(end),torque(end));
                dim = [0.15 0.6 0.3 0.3];
                str = {"Angle = "+ sprintf('%.2f',angle(end)) + " deg", "Torque = "+ sprintf('%.4f',torque(end)) + " N-m"};
                delete(a1);
                a1 = annotation('textbox',dim,'string',str,'FitBoxToText','on', 'FontSize', 12, 'FontName', 'Times');
                drawnow;
                %fprintf(['Angle: %.2f' char(176) '\tTorque: %.4fNm\n'],angle(end),torque(end));
                %pause(.05);
            end
            [rownum,colnum]=size(dataE);
            if colnum == 3
                loadCellE = dataE(1,3);
            end
            done = 1;
        end

    end
    if ~emulate
        clear arduino;
        fprintf("\nTest ended, re-run script for the next sample\n-=-=-=-=-=-=-=-=-=-=-=-=-\n");
        open = 0;
    end
    delete(a1);
else
    fprintf("NO OPEN PORTS");
end
%% Fixing the offset
if ~emulate
    try
        TF=ischange(torque(1:40,1),'variance','Threshold',15); % Threshold tuned on nice data
        breakID = find(TF,1);
        fprintf('The applied offset is %.2f deg\n',angle(breakID));
        torque = torque(breakID:end);
        angle = angle-angle(breakID);
        angle = angle(breakID:end);
    catch
        fprintf('Failed to offset data');
    end
end

%% Angle Vector Conversion from Degrees to Radians
angle_rad = angle.*(pi/180); % Angle conversion from deg to rad.

%% Sample Dimensions
d = input_d*10^-3; % [in m]
r = d/2; % [in m]
l = 20e-3; % [in m]

%% Mechanical Properties of PLA (from Literature)
if(contains(material,"pla",'IgnoreCase',true))
    shearmodulus_lit = 700; % [in MPa]
    ultimateshear_lit = 39; % [in MPa]
    materialType = "PLA";
elseif(contains(material,"al",'IgnoreCase',true) || contains(material,"aluminum",'IgnoreCase',true))
    shearmodulus_lit = 26000; % [in MPa]
    ultimateshear_lit = 207; % [in MPa]
    materialType = "AL";
elseif(contains(material,"abs",'IgnoreCase',true))
    shearmodulus_lit = 680; % [in MPa] +-40
    ultimateshear_lit = 24.4; % [in MPa] 
    materialType = "ABS";
else
    shearmodulus_lit = 700; % [in MPa]
    ultimateshear_lit = 39; % [in MPa]
    materialType = "UNKNOWN(using PLA props.)";
end
%% Plotting of Torque vs. Angle of Twist
torque_smooth = smoothdata(torque,'gaussian'); % Smooth Torque Data

plot(angle,torque_smooth, 'k', 'LineWidth', 2)
title("Torque vs. Angle of Twist", 'FontSize', 18, 'FontName', 'Times')
xlabel("Angle of Twist (rad)", 'FontSize', 14, 'FontName', 'Times')
ylabel("Torque (N-m)", 'FontSize', 14, 'FontName', 'Times')

%% Plotting of Shear Stress vs. Shear Strain
shear_strain = (angle_rad .* r)/l;
shear_stress = (torque .* r)./((pi/32)*d^4);
shear_stress_MPa = shear_stress/(10^6);

figure
plot(shear_strain,shear_stress_MPa, 'k', 'LineWidth', 2, 'DisplayName', 'Sample Data')
title("Shear Stress - Shear Strain Curve", 'FontSize', 18, 'FontName', 'Times')
xlabel("Shear Strain (rad)", 'FontSize', 14, 'FontName', 'Times')
ylabel("Shear Stress (MPa)", 'FontSize', 14, 'FontName', 'Times')
hold on

% Identifying and Plotting Experimental Mechanical Properties:
    % Finding and Labelling Ultimate Shear Stress:
    ult_shear_stress = max(shear_stress_MPa);
    ult = find(shear_stress_MPa == ult_shear_stress);
    plot(shear_strain(ult),shear_stress_MPa(ult), '.r','MarkerSize',20, 'DisplayName', 'Ultimate Shear Stress')
    hold on
    text(shear_strain(ult) - 0.01, shear_stress_MPa(ult) - 1.5, [num2str(shear_stress_MPa(ult)) ' MPa'], 'FontSize', 12.5, 'FontName', 'Times', 'Color', 'r')
    
    % Plotting the Offset Elastic Region, Finding and Labelling the Shear
    % Modulus
    [idx val] = max(shear_stress_MPa);
    Thresh = val;
    [TF S] = ischange(shear_stress_MPa, 'linear', 'Threshold', Thresh);
    idx = find(TF);
    offset = shear_strain(idx(1))*0.2;
    plot(shear_strain(1:idx(1))+offset, shear_stress_MPa(1:idx(1)), '--b', 'LineWidth', 1.5, 'DisplayName', 'Elastic Region')
    p = polyfit(shear_strain(1:idx(1))+offset, shear_stress_MPa(1:idx(1)), 1);
    shear_modulus = p(1);
    text(shear_strain(round(idx(1)/2))+offset+0.001, shear_stress_MPa(round(idx(1)/2)), ['\rightarrow Shear Modulus = ' num2str(shear_modulus) ' MPa'], 'FontSize', 12, 'FontName', 'Times','Color','b')
    
    % Adding a Legend
    legend('Location', 'southeast', 'FontName', 'Times')
    
%% Percent Errors Associated with the Experimental Mechanical Property Values
err_shearmodulus = (abs(shearmodulus_lit - shear_modulus)/shearmodulus_lit)*100;
err_shearstrength = (abs(ultimateshear_lit - ult_shear_stress)/ultimateshear_lit)*100;

% Print the Results in a Message Box
if loadCellE
    err_LC = "Error Detected";
else
    err_LC = "No Error Detected";
end
EM = msgbox(sprintf('The percent errors associated with the experimental shear modulus and ultimate shear stress are %.2f%% and %.2f%%, respectively.\n\nLoad Cell Status: %s   Material: %s',err_shearmodulus,err_shearstrength,err_LC,materialType));
EMT = findall(EM,'Type','Text');
EMT.FontSize = 16;
EMT.FontName = 'Times';
deltaWidth = sum(EMT.Extent([1,3]))-EM.Position(3) + EMT.Extent(1);
deltaHeight = sum(EMT.Extent([2,4]))-EM.Position(4) + 10;
EM.Position([3,4]) = EM.Position([3,4]) + [deltaWidth, deltaHeight];
EM.Resize = 'on';

%% Saving the file
if ~emulate
    FileName = ['TwistAngleTorqueSampleData_',material,'_',sprintf('%.2f',err_shearstrength),'E_',datestr(now, 'mmm-dd-yyyy'),'.mat'];
    dataSave = [angle(:,1) torque(:,1)];
    dataSave(1,3) = loadCellE;
    save(FileName,'dataSave') % Saves Data
    fprintf("\nData Saved as: %s\n", FileName);
    pause(20);
    clear arduino;
end