within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampCurrrent_CHEETA_Comparison_Gas
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent(
    I=90000,
    duration=15,
    offset=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-70,-24})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=0)   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-10,-24})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-34,-76},{-14,-56}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{-4,26},{-24,46}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{42,26},{22,46}})));
  GasCooled.HTS_GasCooling_Voltage              hydrogen(
    l=10,
    n=5.29,
    I_c0=37000,
    A=2.835e-4,
    epsilon_r=2.2,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 0,
    a(displayUnit="mm"),
    b(displayUnit="mm")) "liquid hydrogen cooling"
    annotation (Placement(transformation(extent={{-44,24},{-28,16}})));
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent1(
    I=50000,
    duration=15,
    offset=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={88,-22})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=0)   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={148,-22})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{154,28},{134,48}})));
  Modelica.Blocks.Sources.Constant const1(k=20)
    annotation (Placement(transformation(extent={{200,28},{180,48}})));
  LiquidCooled.HTS_filmboiling_Voltage_Nitrogen nitrogen(
    l=10,
    n=20,
    I_c0=37000,
    A=2.835e-4,
    epsilon_r=2.2,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 0,
    a(displayUnit="mm"),
    b(displayUnit="mm")) "liquid nitrogen cooling"
    annotation (Placement(transformation(extent={{112,26},{128,18}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{124,-74},{144,-54}})));
equation
  connect(rampCurrent.p,ground2. p) annotation (Line(points={{-70,-34},{-70,-56},
          {-24,-56}},      color={0,0,255}));
  connect(resistor1.n,ground2. p)
    annotation (Line(points={{-10,-34},{-10,-56},{-24,-56}},
                                                        color={0,0,255}));
  connect(prescribedTemperature.T, const.y)
    annotation (Line(points={{-2,36},{21,36}},   color={0,0,127}));
  connect(hydrogen.pin_p, rampCurrent.n)
    annotation (Line(points={{-45,20},{-70,20},{-70,-14}}, color={0,0,255}));
  connect(hydrogen.pin_n, resistor1.p)
    annotation (Line(points={{-27,20},{-10,20},{-10,-14}}, color={0,0,255}));
  connect(hydrogen.port_a, prescribedTemperature.port) annotation (Line(points=
          {{-35.8,24},{-35.8,36},{-24,36}}, color={191,0,0}));
  connect(rampCurrent1.p, ground1.p)
    annotation (Line(points={{88,-32},{88,-54},{134,-54}}, color={0,0,255}));
  connect(resistor2.n,ground1. p)
    annotation (Line(points={{148,-32},{148,-54},{134,-54}},
                                                        color={0,0,255}));
  connect(prescribedTemperature1.T, const1.y)
    annotation (Line(points={{156,38},{179,38}}, color={0,0,127}));
  connect(nitrogen.pin_p, rampCurrent1.n)
    annotation (Line(points={{111,22},{88,22},{88,-12}}, color={0,0,255}));
  connect(nitrogen.pin_n, resistor2.p)
    annotation (Line(points={{129,22},{148,22},{148,-12}}, color={0,0,255}));
  connect(nitrogen.port_a, prescribedTemperature1.port) annotation (Line(points=
         {{120.2,26},{120.2,38},{134,38}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{240,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{240,
            100}}), graphics={Text(
          extent={{-76,70},{18,60}},
          lineColor={28,108,200},
          textString="Hydrogen"), Text(
          extent={{90,70},{184,60}},
          lineColor={28,108,200},
          textString="Nitrogen")}),
    experiment(
      StopTime=15,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file="GasCoolingPlot.mos" "GasCoolingSetup"));
end RampCurrrent_CHEETA_Comparison_Gas;
