within CHEETA.Aircraft.Electrical.HTS.Examples;
model CopperLine_StabilityTest
  Stekly.Stekly_TransmissionLosses_constantdT
                             stekly_TransmissionLosses_constantdT(
    l=1,
    n=15.2,
    I_c0=10,
    A_cu=1e-4,
    G_d=100,
    R_0=0.1,
    R=0.1,
    I_crit=1038) annotation (Placement(transformation(extent={{-120,4},{-104,-4}})));
  Modelica.Electrical.Analog.Sources.RampVoltage rampVoltage(V=0.5, duration=5,
    offset=0.0001)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-132,-22})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-122,-60},{-102,-40}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{-172,10},{-152,30}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor(G=0.1)
           annotation (Placement(transformation(extent={{-142,10},{-122,30}})));
  HTS_Piline3                      hTS_Piline3_1(
    l=1,
    n=15.2,
    I_c0=1000,
    A_cu=1e-4,
    R_L=0.1,
    G_d=100,
    I_crit=1038) annotation (Placement(transformation(extent={{-6,4},{10,-4}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature1(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor1(G=0.1)
           annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
  Modelica.Electrical.Analog.Sources.RampVoltage rampVoltage1(
    V=0.5,
    duration=5,
    offset=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-36,-18})));
equation
  connect(stekly_TransmissionLosses_constantdT.pin_p, rampVoltage.p)
    annotation (Line(points={{-121,0},{-132,0},{-132,-12}},
                                                        color={0,0,255}));
  connect(rampVoltage.n, ground.p)
    annotation (Line(points={{-132,-32},{-132,-40},{-112,-40}},
                                                           color={0,0,255}));
  connect(fixedTemperature.port, thermalConductor.port_a)
    annotation (Line(points={{-152,20},{-142,20}},
                                                 color={191,0,0}));
  connect(thermalConductor.port_b, stekly_TransmissionLosses_constantdT.port_a)
    annotation (Line(points={{-122,20},{-112,20},{-112,4}},
                                                     color={191,0,0}));
  connect(stekly_TransmissionLosses_constantdT.pin_n, ground.p)
    annotation (Line(points={{-103,0},{-82,0},{-82,-40},{-112,-40}},
                                                             color={0,0,255}));
  connect(fixedTemperature1.port, thermalConductor1.port_a)
    annotation (Line(points={{-40,20},{-30,20}},
                                               color={191,0,0}));
  connect(thermalConductor1.port_b, hTS_Piline3_1.port_a)
    annotation (Line(points={{-10,20},{2,20},{2,4}}, color={191,0,0}));
  connect(hTS_Piline3_1.pin_n, ground1.p) annotation (Line(points={{11,0},{30,0},
          {30,-40},{0,-40}}, color={0,0,255}));
  connect(hTS_Piline3_1.pin_p, rampVoltage1.p)
    annotation (Line(points={{-7,0},{-36,0},{-36,-8}}, color={0,0,255}));
  connect(ground1.p, rampVoltage1.n)
    annotation (Line(points={{0,-40},{-36,-40},{-36,-28}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-180,-100},{240,
            60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-180,-100},{
            240,60}})),
    experiment(StopTime=5, __Dymola_Algorithm="Euler"));
end CopperLine_StabilityTest;
