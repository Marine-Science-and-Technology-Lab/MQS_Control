classdef AutoManeuver_Sequencer_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure          matlab.ui.Figure
        AddsegmentsdescribingthemotionoftheendeffectorLabel_2  matlab.ui.control.Label
        CancelButton      matlab.ui.control.Button
        DoneButton        matlab.ui.control.Button
        AddSegmentButton  matlab.ui.control.Button
        UITable           matlab.ui.control.Table
    end


    properties (Access = public)
        Seg_Table=table(); % Motion segment table
        MotionType=0;
    end

    properties (Access = private)
dT=0.01; %Assumes 100Hz playback
        CurrentPoseAbs=0;
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
           
            CurrentPose=[127;127;0;127;0];
            app.CurrentPoseAbs=CurrentPose;
            %             CurrentPose(4:6)=rad2deg(CurrentPose);
            TableRow=array2table([0 CurrentPose'],'VariableNames',{'Time','Marine Steer','Land Drive','Waterjet','Land Steer','Retraction'});
            app.UITable.Data=TableRow;
        end

        % Button pushed function: AddSegmentButton
        function AddSegmentButtonPushed(app, event)
            OldTable=app.UITable.Data;
            N_rows=size(OldTable,1);
            NewTable=[OldTable;OldTable(end,:)];
            app.Seg_Table=NewTable;
            app.UITable.Data=NewTable;
        end

        % Cell edit callback: UITable
        function UpdateTable(app, event)
            indices = event.Indices;
            newData = event.NewData;
            app.Seg_Table=app.UITable.Data;
%             selectedButton = app.DatumTypeButtonGroup.SelectedObject

          
               
            Seg_Table=app.Seg_Table;
            app.UITable.Data=Seg_Table;

        end

        % Button pushed function: DoneButton
        function DoneButtonPushed(app, event)
            Segmenttypebutton=app.MotionProfileTypeButtonGroup.SelectedObject;
            switch Segmenttypebutton.Text
                case 'Trapezoidal'
                    Segmenttype=0;
                case 'Smoothed'
                    Segmenttype=1;
            end

           

            if size(app.Seg_Table,1)>1
SegTable=app.Seg_Table;
                timecolumn=SegTable.(1);
                if and(max(timecolumn)>0,timecolumn(1)==0)

                    

SegTable2=table2array(SegTable);
            N_segments=size(SegTable2,1)-1;
            MasterPath=[]; MasterTime=[];
            for n=1:N_segments
                Ttemp=[SegTable2(n,1):app.dT:SegTable2(n+1,1)];
                temppath=[];
                for j=1:5
                    
                            temppath(j,:)=linspace(SegTable2(n,j+1),SegTable2(n+1,j+1),length(Ttemp));
                        
                end
                MasterTime=[MasterTime Ttemp];
                MasterPath=[MasterPath temppath];
                MasterPath=(round(MasterPath));
            end
        
           MasterTraj=[MasterTime' MasterPath']

          
           MasterTable=array2table(MasterTraj);
           MasterTable.Properties.VariableNames={'t','strm','ld','jet','strl','wr'};

           [fname fpath]=uiputfile('*.csv')
            
           writetable(MasterTable,[fpath fname])
                else
                    msgbox('Invalid Times')
                end
            else
                msgbox('Must have more than one row')
            end

        end

        % Button pushed function: CancelButton
        function CancelButtonPushed(app, event)
            delete(app);
        end

        % Callback function: not associated with a component
        function DatumTypeButtonGroupSelectionChanged(app, event)
            selectedButton = app.DatumTypeButtonGroup.SelectedObject

            switch selectedButton.Text
                case 'Relative';
                    FirstRow=array2table([0 0 0 0 0 0 0]);
                case 'Absolute'
                    CurrentPose=app.CurrentPoseAbs;
                    CurrentPose(4:6)=rad2deg(CurrentPose(4:6));
                    FirstRow=array2table([0 CurrentPose']);
            end
            Seg_Table=app.Seg_Table;
            Seg_Table(1,:)=FirstRow;
            app.UITable.Data=Seg_Table;

        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 834 437];
            app.UIFigure.Name = 'MATLAB App';

            % Create UITable
            app.UITable = uitable(app.UIFigure);
            app.UITable.ColumnName = {'Time'; 'Marine Steer'; 'Land Drive'; 'Waterjet'; 'Land Steer'; 'Retraction'};
            app.UITable.RowName = {};
            app.UITable.ColumnEditable = true;
            app.UITable.CellEditCallback = createCallbackFcn(app, @UpdateTable, true);
            app.UITable.Position = [14 146 795 271];

            % Create AddSegmentButton
            app.AddSegmentButton = uibutton(app.UIFigure, 'push');
            app.AddSegmentButton.ButtonPushedFcn = createCallbackFcn(app, @AddSegmentButtonPushed, true);
            app.AddSegmentButton.Position = [24 49 239 82];
            app.AddSegmentButton.Text = {'Add '; 'Segment'};

            % Create DoneButton
            app.DoneButton = uibutton(app.UIFigure, 'push');
            app.DoneButton.ButtonPushedFcn = createCallbackFcn(app, @DoneButtonPushed, true);
            app.DoneButton.Position = [678 96 132 35];
            app.DoneButton.Text = 'Done';

            % Create CancelButton
            app.CancelButton = uibutton(app.UIFigure, 'push');
            app.CancelButton.ButtonPushedFcn = createCallbackFcn(app, @CancelButtonPushed, true);
            app.CancelButton.Position = [677 49 132 39];
            app.CancelButton.Text = 'Cancel';

            % Create AddsegmentsdescribingthemotionoftheendeffectorLabel_2
            app.AddsegmentsdescribingthemotionoftheendeffectorLabel_2 = uilabel(app.UIFigure);
            app.AddsegmentsdescribingthemotionoftheendeffectorLabel_2.Position = [16 12 526 38];
            app.AddsegmentsdescribingthemotionoftheendeffectorLabel_2.Text = {'Trapezoidal motion will use straight-line segments to connect each point. '; 'Smoothed motion will use a scaled hyperbolic tangent to produce smooth "S" motion segments.'};

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = AutoManeuver_Sequencer_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end