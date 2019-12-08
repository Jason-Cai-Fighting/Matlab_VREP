a = newfis('Car_control');
%input(1):dis-l
a = addInput(a,[0 2],'Name','dis_l');
a = addMF(a,'dis_l','trapmf',[-1 0 0.2 0.5],'Name','close');
a = addMF(a,'dis_l','trimf',[0.4 0.4 1.4],'Name','near');
a = addmf(a,'dis_l','trapmf',[1 1.4 2 3],'Name','far');

%input(2):dis-lf
a = addInput(a,[0 2],'Name','dis_lf');
a = addMF(a,'dis_lf','trapmf',[-1 0 1 1.4],'Name','close');
a = addMF(a,'dis_lf','trimf',[1.2 1.5 1.8],'Name','near');
a = addMF(a,'dis_lf','trimf',[1.8 2 2],'Name','far');

%input(3):dis-f
a = addInput(a,[0 2],'Name','dis_f');
a = addMF(a,'dis_f','trapmf',[-1 0 1 1.4],'Name','close');
a = addMF(a,'dis_f','trimf',[1.2 1.5 1.8],'Name','near');
a = addmf(a,'dis_f','trapmf',[1.6 1.8 2 3],'Name','far');

%input(4):dis-rf
a = addInput(a,[0 2],'Name','dis_rf');
a = addMF(a,'dis_rf','trapmf',[-1 0 1 1.4],'Name','close');
a = addMF(a,'dis_rf','trimf',[1.2 1.5 1.8],'Name','near');
a = addMF(a,'dis_rf','trimf',[1.8 2 2],'Name','far');

%input(5):dis-r
a = addInput(a,[0 2],'Name','dis_r');
a = addMF(a,'dis_r','trapmf',[-1 0 0.2 0.5],'Name','close');
a = addMF(a,'dis_r','trimf',[0.4 0.4 1.4],'Name','near');
a = addmf(a,'dis_r','trapmf',[1 1.4 2 3],'Name','far');

%input(6):goal_position
a = addInput(a,[-150 150],'Name','goal_position');
a = addMF(a,'goal_position','trapmf',[10 60 150 180],'Name','left');
a = addMF(a,'goal_position','trimf',[-10 0 10],'Name','front');
a = addmf(a,'goal_position','trapmf',[-180 -150 -60 -10],'Name','right');

%output(1):steer_angle
a = addOutput(a,[50 130],'Name','steer_angle');
a = addMF(a,'steer_angle','trimf',[0 50 80],'Name','turn_l');
a = addMF(a,'steer_angle','trimf',[55 75 90],'Name','turn_lsmall');
a = addMF(a,'steer_angle','trimf',[70 90 110],'Name','forward');
a = addMF(a,'steer_angle','trimf',[90 105 125],'Name','turn_rsmall');
a = addMF(a,'steer_angle','trimf',[100 130 180],'Name','turn_r');

%output(2):speed
a = addOutput(a,[0 2],'Name','speed');
a = addMF(a,'speed','trimf',[-1 0 0],'Name','stop');
a = addMF(a,'speed','trimf',[0.2 0.2 1.2],'Name','slow');
a = addMF(a,'speed','trimf',[0.8 1.4 1.4],'Name','quick');

%rulelist
rulelist = [1 0 1 0 1 0 3 1 1 1;
            0 -1 3 -1 0 2 3 3 1 1
            0 1 3 -1 0 2 4 2 1 1
            0 -1 3 1 0 2 2 2 1 1
            0 1 3 1 0 2 3 2 1 1
            3 -1 -3 0 0 2 1 2 1 1
            -3 0 -3 -1 3 2 5 2 1 1
            2 0 -3 0 -3 2 2 2 1 1
            1 0 -3 0 2 2 4 2 1 1
            1 0 -3 0 1 2 3 1 1 1
            3 -1 0 0 0 1 1 2 1 1
            -3 0 3 0 0 1 3 3 1 1
            -3 0 -3 -1 3 1 5 2 1 1
            1 0 -1 -1 -1 1 5 2 1 1
            0 0 0 -1 3 3 5 2 1 1
            0 0 3 0 -3 3 3 3 1 1
            3 -1 -3 0 -3 3 1 2 1 1
            -1 -1 -1 0 1 3 1 2 1 1
            0 1 0 2 0 0 5 2 1 1
            0 1 0 3 0 0 5 2 1 1
            0 2 0 3 0 0 4 2 1 1
            0 2 0 1 0 0 1 2 1 1
            0 3 0 1 0 0 1 2 1 1
            0 3 0 2 0 0 2 2 1 1];
        
a = addRule(a,rulelist);        