classdef LSS_App < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        DoFArmControlUIFigure          matlab.ui.Figure
        InverseKinematicsPanel         matlab.ui.container.Panel
        XinSliderLabel                 matlab.ui.control.Label
        XinSlider                      matlab.ui.control.Slider
        ZinSliderLabel                 matlab.ui.control.Label
        ZinSlider                      matlab.ui.control.Slider
        GSliderLabel                   matlab.ui.control.Label
        GSlider                        matlab.ui.control.Slider
        YinSliderLabel                 matlab.ui.control.Label
        YinSlider                      matlab.ui.control.Slider
        ForwardKinematicsPanel         matlab.ui.container.Panel
        BaseEditFieldLabel             matlab.ui.control.Label
        BaseEditField                  matlab.ui.control.NumericEditField
        ShoulderEditFieldLabel         matlab.ui.control.Label
        ShoulderEditField              matlab.ui.control.NumericEditField
        ElbowEditFieldLabel            matlab.ui.control.Label
        ElbowEditField                 matlab.ui.control.NumericEditField
        WristEditFieldLabel            matlab.ui.control.Label
        WristEditField                 matlab.ui.control.NumericEditField
        GripperEditFieldLabel          matlab.ui.control.Label
        GripperEditField               matlab.ui.control.NumericEditField
        LIMPButton                     matlab.ui.control.Button
        BAUDDropDownLabel              matlab.ui.control.Label
        BAUDDropDown                   matlab.ui.control.DropDown
        COMDropDownLabel               matlab.ui.control.Label
        COMDropDown                    matlab.ui.control.DropDown
        LSS4DoFRoboticArmControlLabel  matlab.ui.control.Label
        LEDDropDownLabel               matlab.ui.control.Label
        LEDDropDown                    matlab.ui.control.DropDown
        UPDATEButton_2                 matlab.ui.control.Button
        SENDButton                     matlab.ui.control.Button
        VIEWButton                     matlab.ui.control.Button
        STOPButton_2                   matlab.ui.control.Button
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            global s;
            global q;
            global LSSrobot;
            
            % LSS Arm joint lenghts (inches)
            d1 = 4.13;   % Bottom to shoulder
            d2 = 5.61;   % Shoulder to elbow
            d3 = 6.39;   % Elbow to wrist
            d4 = 4.52;   % Wrist to end of gripper
            
            % Links for 4DoF Arm
            L(1) = Link([0 d1 0 pi/2]);
            L(2) = Link([0 0 d2 0]);
            L(3) = Link([0 0 d3 0]);
            L(4) = Link([0 0 d4 0]);
            
            % Joint limit angles
            L(1).qlim = [-pi pi];
            L(2).qlim = [-pi/2 pi/2];
            L(3).qlim = [-pi/2 pi/2];
            L(4).qlim = [-pi/2 pi/2];
            
            % Joint offsets
            L(1).offset = pi;
            L(2).offset = pi/2;
            L(3).offset = pi/2;
            
            % Serial Links -> Robotic Arm
            LSSrobot = SerialLink(L);
            LSSrobot.name = "LSS Robot";
            LSSrobot.plotopt = {'workspace', [-20,20,-20,20,0,20], 'tile1color', [232 232 232], 'jointcolor', 'c','noarrow', 'nowrist'};
            
            % Display robot in teach mode
            q = [0 0 0 0];
            LSSrobot.teach(q);
            
            if isempty(seriallist)
                helpdlg({'LSS arm is not connected',...
                         'Please connect the arm to the PC'});
                app.COMDropDown.Value = "OFF";
            else
                delete(instrfind);
                s = serial(seriallist);
                s.Baudrate = str2double(app.BAUDDropDown.Value);
                s.Terminator = 'CR';
                fopen(s);
                
                % Configure Max Speed to 60 deg/s
                fprintf(s, sprintf('#%d%s%d', 254, 'CSD', 60));
                
                % Disable the Trapezoidal Motion Profile (shoulder, elbow
                % and wrist)
                fprintf(s, sprintf('#%d%s%d', 2, 'EM', 0));
                fprintf(s, sprintf('#%d%s%d', 3, 'EM', 0));
                fprintf(s, sprintf('#%d%s%d', 4, 'EM', 0));
                
                % Change the Filter Position Count
                fprintf(s, sprintf('#%d%s%d', 2, 'FPC', 15));
                fprintf(s, sprintf('#%d%s%d', 3, 'FPC', 15));
                fprintf(s, sprintf('#%d%s%d', 4, 'FPC', 15));
            end
        end

        % Button pushed function: SENDButton
        function SENDButtonPushed(app, event)
            global s;
            
            % Get interface values
            base_ang = app.BaseEditField.Value;
            shoulder_ang = app.ShoulderEditField.Value;
            elbow_ang = app.ElbowEditField.Value;
            wrist_ang = app.WristEditField.Value;
            gripper_ang = app.GripperEditField.Value;
            
            if app.COMDropDown.Value == "ON"
                % Limit LSS angle values to [-90,90]
                base = min(max(base_ang,-90),90);
                shoulder = min(max(shoulder_ang,-90),90);
                elbow = min(max(elbow_ang,-90),90);
                wrist = min(max(wrist_ang,-90),90);
                gripper = min(max(-gripper_ang,-90),0);
                
                % Send serial commands to LSS Arm
                fprintf(s, sprintf('#%d%s%d', 1, 'D', round((base)*10)));
                fprintf(s, sprintf('#%d%s%d', 2, 'D', round((shoulder)*10)));
                fprintf(s, sprintf('#%d%s%d', 3, 'D', round((elbow)*10)));
                fprintf(s, sprintf('#%d%s%d', 4, 'D', round((wrist)*10)));
                fprintf(s, sprintf('#%d%s%d%s%d', 5, 'D', round((gripper)*10), 'CH', 400));
            else
                helpdlg({'COM port is OFF',...
                         'Please turn it ON'});
            end
        end

        % Button pushed function: VIEWButton
        function VIEWButtonPushed(app, event)
            global q;
            global LSSrobot;
        
            % Get interface values
            X = app.XinSlider.Value;
            Y = app.YinSlider.Value;
            Z = app.ZinSlider.Value;
            G = app.GSlider.Value;
            
            Ti = [1 0 0 X;
                  0 1 0 Y;
                  0 0 1 Z;
                  0 0 0 1];
                  
            % Calculate inverse kinematics
            J = LSSrobot.ikcon(Ti, q)*180/pi;
            
            if isempty(J)   % Error message
               helpdlg({'Error: Failed to converge',...
                        'Try different values of joint coordinates'});
            else             
                % Update interface values in the forward kinematics panel
                app.BaseEditField.Value = round(J(1));
                app.ShoulderEditField.Value = round(J(2));
                app.ElbowEditField.Value = round(J(3));
                app.WristEditField.Value = round(J(4));
                app.GripperEditField.Value = round(G);
                
                % Plot results
                q = J*pi/180;
                LSSrobot.plot(q);
            end
            
            clear J;
        end

        % Button pushed function: LIMPButton
        function LIMPButtonPushed(app, event)
            global s;
            pause(2);
            if app.COMDropDown.Value == "ON"
                fprintf(s, sprintf('#%d%s', 254, 'L'));
            else
                helpdlg({'COM port is OFF',...
                         'Please turn it ON'});
            end
        end

        % Value changed function: BAUDDropDown
        function BAUDDropDownValueChanged(app, event)
            global s;
            if app.COMDropDown.Value == "ON"
                fprintf(s, sprintf('#%d%s%d', 254, 'CB', str2double(app.BAUDDropDown.Value)));
                fprintf(s, sprintf('#%d%s', 254, 'RESET'));
                pause(2);
                fprintf(s, sprintf('#%d%s', 254, 'CONFIRM'));
                delete(instrfind);
                s = serial(seriallist);
                s.Baudrate = str2double(app.BAUDDropDown.Value);
                s.Terminator = 'CR';
                fopen(s);
            else
                helpdlg({'COM port is OFF',...
                         'Please turn it ON'});
            end
        end

        % Value changed function: COMDropDown
        function COMDropDownValueChanged(app, event)
            global s;
            if app.COMDropDown.Value == "ON"
                if isempty(seriallist)
                    helpdlg({'LSS arm is not connected',...
                             'Please connect the arm to the PC'});
                    app.COMDropDown.Value = "OFF";
                else
                    delete(instrfind);
                    s = serial(seriallist);
                    s.Baudrate = str2double(app.BAUDDropDown.Value);
                    s.Terminator = 'CR';
                    fopen(s);
                end
            else
                fclose(s);
            end
        end

        % Close request function: DoFArmControlUIFigure
        function DoFArmControlUIFigureCloseRequest(app, event)
            global s;
            if ~isempty(seriallist)
                fclose(s);
                delete(instrfind);
            end
            delete(app);
        end

        % Value changed function: LEDDropDown
        function LEDDropDownValueChanged(app, event)
            LED = app.LEDDropDown.Value;
            global s
            if app.COMDropDown.Value == "ON"
                switch LED
                    case "Black"
                        fprintf(s, sprintf('#%d%s%d', 254, 'LED', 0));
                    case "Red"
                        fprintf(s, sprintf('#%d%s%d', 254, 'LED', 1));
                    case "Green"
                        fprintf(s, sprintf('#%d%s%d', 254, 'LED', 2));
                    case "Blue"
                        fprintf(s, sprintf('#%d%s%d', 254, 'LED', 3));
                    case "Yellow"
                        fprintf(s, sprintf('#%d%s%d', 254, 'LED', 4));
                    case "Cyan"
                        fprintf(s, sprintf('#%d%s%d', 254, 'LED', 5));
                    case "Magenta"
                        fprintf(s, sprintf('#%d%s%d', 254, 'LED', 6));
                    case "White"
                        fprintf(s, sprintf('#%d%s%d', 254, 'LED', 7));
                end
            else
                helpdlg({'COM port is OFF',...
                         'Please turn it ON'});
            end
        end

        % Button pushed function: UPDATEButton_2
        function UPDATEButton_2Pushed(app, event)
            global q;
            global LSSrobot;

            % Get joint coordinates from graphical display
            q = LSSrobot.getpos();

            % Update interface values in forward kinematics panel
            app.BaseEditField.Value = round(q(1)*180/pi);
            app.ShoulderEditField.Value = round(q(2)*180/pi);
            app.ElbowEditField.Value = round(q(3)*180/pi);
            app.WristEditField.Value = round(q(4)*180/pi);
            
            % Calculate forward kinematics
            T = LSSrobot.fkine(q);
            [x,y,z] = transl(T);
            
            % Update interface values in inverse kinematics panel
            app.XinSlider.Value = round(x);
            app.YinSlider.Value = round(y);
            if round(z) > 0
                app.ZinSlider.Value = round(z);
            else
                helpdlg({'Z value is out of limits',...
                         'Please change the joint positions'});
            end
        end

        % Button pushed function: STOPButton_2
        function STOPButtonPushed(app, event)
            global s;
            if app.COMDropDown.Value == "ON"
                fprintf(s, sprintf('#%d%s', 254, 'H'));
            else
                helpdlg({'COM port is OFF',...
                         'Please turn it ON'});
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create DoFArmControlUIFigure and hide until all components are created
            app.DoFArmControlUIFigure = uifigure('Visible', 'off');
            app.DoFArmControlUIFigure.Position = [100 100 436 470];
            app.DoFArmControlUIFigure.Name = '4 DoF Arm Control';
            app.DoFArmControlUIFigure.CloseRequestFcn = createCallbackFcn(app, @DoFArmControlUIFigureCloseRequest, true);

            % Create InverseKinematicsPanel
            app.InverseKinematicsPanel = uipanel(app.DoFArmControlUIFigure);
            app.InverseKinematicsPanel.TitlePosition = 'centertop';
            app.InverseKinematicsPanel.Title = 'Inverse Kinematics';
            app.InverseKinematicsPanel.Position = [18 70 268 282];

            % Create XinSliderLabel
            app.XinSliderLabel = uilabel(app.InverseKinematicsPanel);
            app.XinSliderLabel.HorizontalAlignment = 'right';
            app.XinSliderLabel.FontSize = 11;
            app.XinSliderLabel.Position = [11 220 31 22];
            app.XinSliderLabel.Text = 'X (in)';

            % Create XinSlider
            app.XinSlider = uislider(app.InverseKinematicsPanel);
            app.XinSlider.Limits = [-17 17];
            app.XinSlider.MajorTicks = [-17 -12 -8 -4 0 4 8 12 17];
            app.XinSlider.MinorTicks = [-17 -16 -15 -14 -13 -12 -11 -10 -9 -8 -7 -6 -5 -4 -3 -2 -1 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17];
            app.XinSlider.FontSize = 11;
            app.XinSlider.Position = [57 229 189 3];

            % Create ZinSliderLabel
            app.ZinSliderLabel = uilabel(app.InverseKinematicsPanel);
            app.ZinSliderLabel.HorizontalAlignment = 'right';
            app.ZinSliderLabel.FontSize = 11;
            app.ZinSliderLabel.Position = [11 98 31 22];
            app.ZinSliderLabel.Text = 'Z (in)';

            % Create ZinSlider
            app.ZinSlider = uislider(app.InverseKinematicsPanel);
            app.ZinSlider.Limits = [1 22];
            app.ZinSlider.FontSize = 11;
            app.ZinSlider.Position = [57 107 190 3];
            app.ZinSlider.Value = 1;

            % Create GSliderLabel
            app.GSliderLabel = uilabel(app.InverseKinematicsPanel);
            app.GSliderLabel.HorizontalAlignment = 'right';
            app.GSliderLabel.FontSize = 11;
            app.GSliderLabel.Position = [3 37 39 22];
            app.GSliderLabel.Text = 'G (°)';

            % Create GSlider
            app.GSlider = uislider(app.InverseKinematicsPanel);
            app.GSlider.Limits = [0 90];
            app.GSlider.MajorTicks = [0 15 30 45 60 75 90];
            app.GSlider.FontSize = 11;
            app.GSlider.Position = [56 46 191 3];

            % Create YinSliderLabel
            app.YinSliderLabel = uilabel(app.InverseKinematicsPanel);
            app.YinSliderLabel.HorizontalAlignment = 'right';
            app.YinSliderLabel.FontSize = 11;
            app.YinSliderLabel.Position = [10 159 31 22];
            app.YinSliderLabel.Text = 'Y (in)';

            % Create YinSlider
            app.YinSlider = uislider(app.InverseKinematicsPanel);
            app.YinSlider.Limits = [-17 17];
            app.YinSlider.MajorTicks = [-17 -12 -8 -4 0 4 8 12 17];
            app.YinSlider.MinorTicks = [-17 -16 -15 -14 -13 -12 -11 -10 -9 -8 -7 -6 -5 -4 -3 -2 -1 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17];
            app.YinSlider.FontSize = 11;
            app.YinSlider.Position = [56 168 191 3];

            % Create ForwardKinematicsPanel
            app.ForwardKinematicsPanel = uipanel(app.DoFArmControlUIFigure);
            app.ForwardKinematicsPanel.TitlePosition = 'centertop';
            app.ForwardKinematicsPanel.Title = 'Forward Kinematics';
            app.ForwardKinematicsPanel.Position = [295 70 124 282];

            % Create BaseEditFieldLabel
            app.BaseEditFieldLabel = uilabel(app.ForwardKinematicsPanel);
            app.BaseEditFieldLabel.HorizontalAlignment = 'right';
            app.BaseEditFieldLabel.Position = [23 214 49 22];
            app.BaseEditFieldLabel.Text = 'Base (°)';

            % Create BaseEditField
            app.BaseEditField = uieditfield(app.ForwardKinematicsPanel, 'numeric');
            app.BaseEditField.Limits = [-180 180];
            app.BaseEditField.Editable = 'off';
            app.BaseEditField.HorizontalAlignment = 'center';
            app.BaseEditField.Position = [85 214 33 22];

            % Create ShoulderEditFieldLabel
            app.ShoulderEditFieldLabel = uilabel(app.ForwardKinematicsPanel);
            app.ShoulderEditFieldLabel.HorizontalAlignment = 'right';
            app.ShoulderEditFieldLabel.Position = [1 167 70 22];
            app.ShoulderEditFieldLabel.Text = 'Shoulder (°)';

            % Create ShoulderEditField
            app.ShoulderEditField = uieditfield(app.ForwardKinematicsPanel, 'numeric');
            app.ShoulderEditField.Limits = [-90 90];
            app.ShoulderEditField.Editable = 'off';
            app.ShoulderEditField.HorizontalAlignment = 'center';
            app.ShoulderEditField.Position = [85 167 33 22];

            % Create ElbowEditFieldLabel
            app.ElbowEditFieldLabel = uilabel(app.ForwardKinematicsPanel);
            app.ElbowEditFieldLabel.HorizontalAlignment = 'right';
            app.ElbowEditFieldLabel.Position = [16 120 55 22];
            app.ElbowEditFieldLabel.Text = 'Elbow (°)';

            % Create ElbowEditField
            app.ElbowEditField = uieditfield(app.ForwardKinematicsPanel, 'numeric');
            app.ElbowEditField.Limits = [-90 90];
            app.ElbowEditField.Editable = 'off';
            app.ElbowEditField.HorizontalAlignment = 'center';
            app.ElbowEditField.Position = [85 120 31 22];

            % Create WristEditFieldLabel
            app.WristEditFieldLabel = uilabel(app.ForwardKinematicsPanel);
            app.WristEditFieldLabel.HorizontalAlignment = 'right';
            app.WristEditFieldLabel.Position = [22 73 49 22];
            app.WristEditFieldLabel.Text = 'Wrist (°)';

            % Create WristEditField
            app.WristEditField = uieditfield(app.ForwardKinematicsPanel, 'numeric');
            app.WristEditField.Limits = [-90 90];
            app.WristEditField.Editable = 'off';
            app.WristEditField.HorizontalAlignment = 'center';
            app.WristEditField.Position = [86 73 30 22];

            % Create GripperEditFieldLabel
            app.GripperEditFieldLabel = uilabel(app.ForwardKinematicsPanel);
            app.GripperEditFieldLabel.HorizontalAlignment = 'right';
            app.GripperEditFieldLabel.Position = [8 27 62 22];
            app.GripperEditFieldLabel.Text = 'Gripper (°)';

            % Create GripperEditField
            app.GripperEditField = uieditfield(app.ForwardKinematicsPanel, 'numeric');
            app.GripperEditField.Limits = [0 90];
            app.GripperEditField.Editable = 'off';
            app.GripperEditField.HorizontalAlignment = 'center';
            app.GripperEditField.Position = [85 27 31 22];

            % Create LIMPButton
            app.LIMPButton = uibutton(app.DoFArmControlUIFigure, 'push');
            app.LIMPButton.ButtonPushedFcn = createCallbackFcn(app, @LIMPButtonPushed, true);
            app.LIMPButton.Position = [272 26 65 22];
            app.LIMPButton.Text = 'LIMP';

            % Create BAUDDropDownLabel
            app.BAUDDropDownLabel = uilabel(app.DoFArmControlUIFigure);
            app.BAUDDropDownLabel.HorizontalAlignment = 'right';
            app.BAUDDropDownLabel.Position = [170 370 39 22];
            app.BAUDDropDownLabel.Text = 'BAUD';

            % Create BAUDDropDown
            app.BAUDDropDown = uidropdown(app.DoFArmControlUIFigure);
            app.BAUDDropDown.Items = {'2400', '9600', '38400', '115200'};
            app.BAUDDropDown.ValueChangedFcn = createCallbackFcn(app, @BAUDDropDownValueChanged, true);
            app.BAUDDropDown.Position = [224 370 72 22];
            app.BAUDDropDown.Value = '115200';

            % Create COMDropDownLabel
            app.COMDropDownLabel = uilabel(app.DoFArmControlUIFigure);
            app.COMDropDownLabel.HorizontalAlignment = 'right';
            app.COMDropDownLabel.Position = [314 370 33 22];
            app.COMDropDownLabel.Text = 'COM';

            % Create COMDropDown
            app.COMDropDown = uidropdown(app.DoFArmControlUIFigure);
            app.COMDropDown.Items = {'ON', 'OFF'};
            app.COMDropDown.ValueChangedFcn = createCallbackFcn(app, @COMDropDownValueChanged, true);
            app.COMDropDown.Position = [362 370 57 22];
            app.COMDropDown.Value = 'ON';

            % Create LSS4DoFRoboticArmControlLabel
            app.LSS4DoFRoboticArmControlLabel = uilabel(app.DoFArmControlUIFigure);
            app.LSS4DoFRoboticArmControlLabel.FontSize = 20;
            app.LSS4DoFRoboticArmControlLabel.FontWeight = 'bold';
            app.LSS4DoFRoboticArmControlLabel.Position = [69 421 306 25];
            app.LSS4DoFRoboticArmControlLabel.Text = 'LSS 4DoF Robotic Arm Control';

            % Create LEDDropDownLabel
            app.LEDDropDownLabel = uilabel(app.DoFArmControlUIFigure);
            app.LEDDropDownLabel.HorizontalAlignment = 'right';
            app.LEDDropDownLabel.Position = [18 370 29 22];
            app.LEDDropDownLabel.Text = 'LED';

            % Create LEDDropDown
            app.LEDDropDown = uidropdown(app.DoFArmControlUIFigure);
            app.LEDDropDown.Items = {'OFF', 'Red', 'Green', 'Blue', 'Yellow', 'Cyan', 'Magenta', 'White'};
            app.LEDDropDown.ValueChangedFcn = createCallbackFcn(app, @LEDDropDownValueChanged, true);
            app.LEDDropDown.Position = [62 370 82 22];
            app.LEDDropDown.Value = 'OFF';

            % Create UPDATEButton_2
            app.UPDATEButton_2 = uibutton(app.DoFArmControlUIFigure, 'push');
            app.UPDATEButton_2.ButtonPushedFcn = createCallbackFcn(app, @UPDATEButton_2Pushed, true);
            app.UPDATEButton_2.Position = [18 26 74 22];
            app.UPDATEButton_2.Text = 'UPDATE';

            % Create SENDButton
            app.SENDButton = uibutton(app.DoFArmControlUIFigure, 'push');
            app.SENDButton.ButtonPushedFcn = createCallbackFcn(app, @SENDButtonPushed, true);
            app.SENDButton.Position = [190 26 67 22];
            app.SENDButton.Text = 'SEND';

            % Create VIEWButton
            app.VIEWButton = uibutton(app.DoFArmControlUIFigure, 'push');
            app.VIEWButton.ButtonPushedFcn = createCallbackFcn(app, @VIEWButtonPushed, true);
            app.VIEWButton.Position = [105 26 70 22];
            app.VIEWButton.Text = 'VIEW';

            % Create STOPButton_2
            app.STOPButton_2 = uibutton(app.DoFArmControlUIFigure, 'push');
            app.STOPButton_2.ButtonPushedFcn = createCallbackFcn(app, @STOPButtonPushed, true);
            app.STOPButton_2.Position = [353 26 66 22];
            app.STOPButton_2.Text = 'STOP';

            % Show the figure after all components are created
            app.DoFArmControlUIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = LSS_App

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.DoFArmControlUIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.DoFArmControlUIFigure)
        end
    end
end