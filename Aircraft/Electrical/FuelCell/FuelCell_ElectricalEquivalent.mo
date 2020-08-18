within CHEETA.Aircraft.Electrical.FuelCell;
model FuelCell_ElectricalEquivalent
  parameter Real faultTime = 5;
  Real current;

  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-50,24},{-30,44}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.0146)
    annotation (Placement(transformation(extent={{10,74},{30,94}})));
  Modelica.Electrical.Analog.Sources.SignalCurrent   signalCurrent
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-40,66})));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(v(fixed=true, start=1000),
      C=1.6635)                                                  annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-4,66})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage E_cell(V=1000) annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-52,-18})));
  Modelica.Electrical.Analog.Basic.Resistor R_act(R=0.0065)
    annotation (Placement(transformation(extent={{-30,4},{-10,24}})));
  Modelica.Electrical.Analog.Basic.Resistor R_conc(R=0.13)
    annotation (Placement(transformation(extent={{-4,4},{16,24}})));
  Modelica.Electrical.Analog.Basic.Capacitor C_dl(C=10.6635)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-4,-8})));
  Modelica.Electrical.Analog.Basic.Resistor R_ohm(R=0.0233/10)
    annotation (Placement(transformation(extent={{34,4},{54,24}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-62,-54},{-42,-34}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=current)
    annotation (Placement(transformation(extent={{-90,56},{-70,76}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                "Positive electrical pin"
    annotation (Placement(transformation(extent={{90,0},{110,20}})));
  Modelica.Electrical.Analog.Ideal.IdealCommutingSwitch switch
    annotation (Placement(transformation(extent={{84,36},{64,16}})));
  Modelica.Blocks.Sources.BooleanStep booleanStep(startTime=faultTime)
    annotation (Placement(transformation(extent={{28,-34},{48,-14}})));

equation

  when booleanStep.y == true then
    current = -pre(E_cell.i);
  elsewhen booleanStep.y == false then
    current = 0;
  end when;
  connect(ground2.p, signalCurrent.p)
    annotation (Line(points={{-40,44},{-40,56}}, color={0,0,255}));
  connect(signalCurrent.n, resistor.p)
    annotation (Line(points={{-40,76},{-40,84},{10,84}}, color={0,0,255}));
  connect(capacitor.p,resistor. p)
    annotation (Line(points={{-4,76},{-4,84},{10,84}},  color={0,0,255}));
  connect(capacitor.n,ground2. p) annotation (Line(points={{-4,56},{-4,44},{-40,
          44}},        color={0,0,255}));
  connect(R_act.p, E_cell.p)
    annotation (Line(points={{-30,14},{-52,14},{-52,-8}}, color={0,0,255}));
  connect(R_act.n, R_conc.p)
    annotation (Line(points={{-10,14},{-4,14}},
                                             color={0,0,255}));
  connect(R_conc.n, R_ohm.p)
    annotation (Line(points={{16,14},{34,14}}, color={0,0,255}));
  connect(E_cell.n, ground1.p)
    annotation (Line(points={{-52,-28},{-52,-34}}, color={0,0,255}));
  connect(signalCurrent.i, realExpression.y)
    annotation (Line(points={{-52,66},{-69,66}}, color={0,0,127}));
  connect(booleanStep.y, switch.control)
    annotation (Line(points={{49,-24},{74,-24},{74,14}},
                                                       color={255,0,255}));
  connect(switch.n2, resistor.n) annotation (Line(points={{64,26},{48,26},{48,84},
          {30,84}}, color={0,0,255}));
  connect(switch.n1, R_ohm.n) annotation (Line(points={{64,22},{60,22},{60,14},{
          54,14}}, color={0,0,255}));
  connect(switch.p, p1) annotation (Line(points={{84,26},{90,26},{90,10},{100,10}},
        color={0,0,255}));
  connect(C_dl.p, E_cell.p) annotation (Line(points={{-14,-8},{-34,-8},{-34,4},
          {-52,4},{-52,-8}}, color={0,0,255}));
  connect(C_dl.n, R_ohm.p) annotation (Line(points={{6,-8},{24,-8},{24,14},{34,
          14}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuelCell_ElectricalEquivalent;
