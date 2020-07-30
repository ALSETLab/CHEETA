within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampCurrent_CHEETA_parameters
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent(
    I=6280.95,
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
    annotation (Placement(transformation(extent={{48,30},{28,50}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{94,30},{74,50}})));
  LiquidCooled.HTS_filmboiling_Current hTS_filmboiling2_2(
    l=10,
    n=20,
    I_c0=9000,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d=0,
    a(displayUnit="mm") = 0.00256,
    b(displayUnit="mm") = 0.00363,
    P=0.1) annotation (Placement(transformation(extent={{-48,24},{-32,16}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage
                                                 constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={78,-24})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=0)   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={138,-24})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{114,-76},{134,-56}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{154,28},{134,48}})));
  Modelica.Blocks.Sources.Constant const1(k=20)
    annotation (Placement(transformation(extent={{200,28},{180,48}})));
  LiquidCooled.HTS_filmboiling_Voltage hTS_filmboiling2_1(
    l=10,
    n=20,
    I_c0=9000,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d=0,
    a(displayUnit="mm") = 0.00256,
    b(displayUnit="mm") = 0.00363,
    P=0.1) annotation (Placement(transformation(extent={{100,24},{116,16}})));
equation
  connect(rampCurrent.p,ground2. p) annotation (Line(points={{-70,-34},{-70,-56},
          {-24,-56}},      color={0,0,255}));
  connect(resistor1.n,ground2. p)
    annotation (Line(points={{-10,-34},{-10,-56},{-24,-56}},
                                                        color={0,0,255}));
  connect(prescribedTemperature.T, const.y)
    annotation (Line(points={{50,40},{73,40}},   color={0,0,127}));
  connect(hTS_filmboiling2_2.pin_p, rampCurrent.n)
    annotation (Line(points={{-49,20},{-70,20},{-70,-14}}, color={0,0,255}));
  connect(hTS_filmboiling2_2.pin_n, resistor1.p)
    annotation (Line(points={{-31,20},{-10,20},{-10,-14}}, color={0,0,255}));
  connect(hTS_filmboiling2_2.port_a, prescribedTemperature.port)
    annotation (Line(points={{-39.8,24},{-39.8,40},{28,40}}, color={191,0,0}));
  connect(resistor2.n,ground1. p)
    annotation (Line(points={{138,-34},{138,-56},{124,-56}},
                                                        color={0,0,255}));
  connect(prescribedTemperature1.T, const1.y)
    annotation (Line(points={{156,38},{179,38}}, color={0,0,127}));
  connect(hTS_filmboiling2_1.pin_n,resistor2. p)
    annotation (Line(points={{117,20},{138,20},{138,-14}}, color={0,0,255}));
  connect(hTS_filmboiling2_1.port_a, prescribedTemperature1.port) annotation (
      Line(points={{108.2,24},{108.2,38},{134,38}}, color={191,0,0}));
  connect(constantVoltage.p, hTS_filmboiling2_1.pin_p)
    annotation (Line(points={{78,-14},{78,20},{99,20}}, color={0,0,255}));
  connect(constantVoltage.n, ground1.p)
    annotation (Line(points={{78,-34},{78,-56},{124,-56}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{240,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{240,
            100}})),
    experiment(
      StopTime=20,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Euler"));
end RampCurrent_CHEETA_parameters;
