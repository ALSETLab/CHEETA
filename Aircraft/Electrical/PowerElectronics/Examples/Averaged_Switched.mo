within CHEETA.Aircraft.Electrical.PowerElectronics.Examples;
model Averaged_Switched
  ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.CurrentInputConstantEfficiency converter_ce(
      redeclare
      ElectrifiedPowertrains.PowerElectronics.Converters.Averaged.Records.Data.ConstantEfficiency.Constant100percent
      data)
    annotation (Placement(transformation(extent={{-18,48},{2,68}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor_ce(R=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={32,58})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage_ce(V=1)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-48,58})));
  Modelica.Blocks.Sources.Constant current_ce(k=10) annotation (Placement(transformation(extent={{-28,18},
            {-8,38}})));
  Modelica.Electrical.Analog.Basic.Ground ground1_ce annotation (Placement(transformation(extent={{-58,18},
            {-38,38}})));
  Modelica.Electrical.Analog.Basic.Ground ground2_ce annotation (Placement(transformation(extent={{22,18},
            {42,38}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(final V=1)
             annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-18})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp   chopperStepUp(
      useHeatPort=false)
    annotation (Placement(transformation(extent={{-20,-28},{0,-8}})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(extent={{-70,-58},{-50,-38}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM(final
      constantDutyCycle=0.5, final f=1000)
                                   annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={-10,-50})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=1) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={28,-18})));
equation
  connect(ground2_ce.p,resistor_ce. n)
    annotation (Line(
      points={{32,38},{32,48}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(current_ce.y,converter_ce. i)
    annotation (Line(
      points={{-7,28},{0,28},{0,46}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(converter_ce.p2,resistor_ce. p) annotation (Line(points={{2,64},{12,
          64},{12,68},{32,68}},                                                                      color={0,0,255}));
  connect(resistor_ce.n,converter_ce. n2) annotation (Line(points={{32,48},{12,
          48},{12,52},{2,52}},                                                                       color={0,0,255}));
  connect(constantVoltage_ce.n,ground1_ce. p) annotation (Line(points={{-48,48},
          {-48,38}},                                                                                color={0,0,255}));
  connect(constantVoltage_ce.p,converter_ce. p1) annotation (Line(points={{-48,68},
          {-28,68},{-28,64},{-18,64}},                                                                          color={0,0,255}));
  connect(converter_ce.n1,constantVoltage_ce. n) annotation (Line(points={{-18,52},
          {-28,52},{-28,48},{-48,48}},                                                                          color={0,0,255}));
  connect(constantVoltage.n,chopperStepUp. dc_n1) annotation (Line(points={{-60,-28},
          {-30,-28},{-30,-24},{-20,-24}},    color={0,0,255}));
  connect(constantVoltage.n,ground. p) annotation (Line(
      points={{-60,-28},{-60,-38}}, color={0,0,255}));
  connect(signalPWM.fire,chopperStepUp. fire_p)
    annotation (Line(points={{-16,-39},{-16,-30}}, color={255,0,255}));
  connect(constantVoltage.p, chopperStepUp.dc_p1) annotation (Line(points={{-60,
          -8},{-60,0},{-34,0},{-34,-12},{-20,-12}}, color={0,0,255}));
  connect(chopperStepUp.dc_p2, resistor.p) annotation (Line(points={{0,-12},{10,
          -12},{10,-8},{28,-8}}, color={0,0,255}));
  connect(resistor.n, chopperStepUp.dc_n2) annotation (Line(points={{28,-28},{
          28,-32},{14,-32},{14,-24},{0,-24}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Averaged_Switched;
