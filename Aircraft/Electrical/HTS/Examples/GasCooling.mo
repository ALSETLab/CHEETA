within CHEETA.Aircraft.Electrical.HTS.Examples;
model GasCooling
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
  GasCooled.HTS_GasCooling2 HTS(
    l=10,
    n=20,
    I_c0=5000,
    A=0.1,
    A_cu=0.1,
    I_crit=3000,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d=10,
    a(displayUnit="cm") = 0.256,
    b(displayUnit="cm") = 0.363,
    P=0.1) annotation (Placement(transformation(extent={{-46,24},{-30,16}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage
                                                 constantVoltage(V=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={74,-28})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=1)   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={134,-26})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{110,-80},{130,-60}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{192,26},{172,46}})));
  Modelica.Blocks.Sources.Constant const1(k=20)
    annotation (Placement(transformation(extent={{238,26},{218,46}})));
  GasCooled.HTS_GasCooling_Voltage HTS1(
    l=10,
    n=20,
    I_c0=9000,
    A=0.1,
    A_cu=0.1,
    I_crit=10000,
    T_c(displayUnit="K"),
    R_L=1e-6,
    G_d=0,
    a=0.1,
    b=0.5,
    P=10) annotation (Placement(transformation(extent={{102,10},{118,2}})));
equation
  connect(rampCurrent.p,ground2. p) annotation (Line(points={{-70,-34},{-70,-56},
          {-24,-56}},      color={0,0,255}));
  connect(resistor1.n,ground2. p)
    annotation (Line(points={{-10,-34},{-10,-56},{-24,-56}},
                                                        color={0,0,255}));
  connect(prescribedTemperature.T, const.y)
    annotation (Line(points={{50,40},{73,40}},   color={0,0,127}));
  connect(HTS.pin_p, rampCurrent.n)
    annotation (Line(points={{-47,20},{-70,20},{-70,-14}}, color={0,0,255}));
  connect(HTS.pin_n, resistor1.p)
    annotation (Line(points={{-29,20},{-10,20},{-10,-14}}, color={0,0,255}));
  connect(HTS.port_a, prescribedTemperature.port)
    annotation (Line(points={{-37.8,24},{-37.8,40},{28,40}}, color={191,0,0}));
  connect(resistor2.n,ground1. p)
    annotation (Line(points={{134,-36},{134,-60},{120,-60}},
                                                        color={0,0,255}));
  connect(prescribedTemperature1.T, const1.y)
    annotation (Line(points={{194,36},{217,36}}, color={0,0,127}));
  connect(constantVoltage.n, ground1.p)
    annotation (Line(points={{74,-38},{74,-60},{120,-60}}, color={0,0,255}));
  connect(HTS1.port_a, prescribedTemperature1.port) annotation (Line(points={{
          110.2,10},{110,10},{110,36},{172,36}}, color={191,0,0}));
  connect(HTS1.pin_p, constantVoltage.p)
    annotation (Line(points={{101,6},{74,6},{74,-18}}, color={0,0,255}));
  connect(HTS1.pin_n, resistor2.p)
    annotation (Line(points={{119,6},{134,6},{134,-16}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{240,100}})),                                   Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{240,100}})),
    experiment(
      StopTime=10,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end GasCooling;
