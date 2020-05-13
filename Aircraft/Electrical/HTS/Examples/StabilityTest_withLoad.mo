within CHEETA.Aircraft.Electrical.HTS.Examples;
model StabilityTest_withLoad
  HTS_Piline2                            hTS_Piline2_1(
    l=4,
    E_0=1e-2,
    n=20,
    I_c0=10,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=1e-3,
    G_d=0,
    a=0.4,
    b=0.5,
    P=0.1,
    UIC=false,
    IC=0.1)      annotation (Placement(transformation(extent={{-36,-28},{-20,
            -36}})));
  Modelica.Electrical.Analog.Sources.RampVoltage rampVoltage1(
    V=0.5,
    duration=5)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-54,-50})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor1(G=0.1)
           annotation (Placement(transformation(extent={{30,-30},{10,-10}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature1(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{70,-30},{50,-10}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-22,-98},{-2,-78}})));
  HTS_Piline3                            hTS_Piline3_1(
    l=4,
    n=20,
    I_c0=1,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=1e-3,
    G_d=0,
    a=0.1,
    b=0.5,
    P=0.1,
    UIC=false,
    IC=0.1)      annotation (Placement(transformation(extent={{-30,62},{-14,54}})));
  Modelica.Electrical.Analog.Sources.RampVoltage rampVoltage2(
    V=0.5,
    duration=5,
    offset=0.5)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-62,36})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor2(G=0.1)
           annotation (Placement(transformation(extent={{18,70},{-2,90}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature2(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{62,70},{42,90}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-26,-20},{-6,0}})));
  HTS_Piline3                            hTS_Piline3_2(
    l=4,
    n=20,
    I_c0=1,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=1e-3,
    G_d=0,
    a=0.1,
    b=0.5,
    P=0.1,
    UIC=false,
    IC=0.1)      annotation (Placement(transformation(extent={{126,64},{142,56}})));
  Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(
    V=0.1,
    freqHz=60,
    offset=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={94,38})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor3(G=0.1)
           annotation (Placement(transformation(extent={{174,72},{154,92}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature3(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{218,72},{198,92}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{112,-20},{132,0}})));
equation
  connect(thermalConductor1.port_a, fixedTemperature1.port)
    annotation (Line(points={{30,-20},{50,-20}}, color={191,0,0}));
  connect(thermalConductor1.port_b, hTS_Piline2_1.port_a)
    annotation (Line(points={{10,-20},{-28,-20},{-28,-28}},
                                                      color={191,0,0}));
  connect(hTS_Piline2_1.pin_p, rampVoltage1.p)
    annotation (Line(points={{-37,-32},{-54,-32},{-54,-40}},
                                                        color={0,0,255}));
  connect(rampVoltage1.n, ground.p)
    annotation (Line(points={{-54,-60},{-54,-78},{-12,-78}},
                                                           color={0,0,255}));
  connect(hTS_Piline2_1.pin_n, ground.p) annotation (Line(points={{-19,-32},{0,
          -32},{0,-78},{-12,-78}},
                             color={0,0,255}));
  connect(thermalConductor2.port_a,fixedTemperature2. port)
    annotation (Line(points={{18,80},{42,80}},   color={191,0,0}));
  connect(thermalConductor2.port_b,hTS_Piline3_1. port_a)
    annotation (Line(points={{-2,80},{-22,80},{-22,62}},
                                                      color={191,0,0}));
  connect(hTS_Piline3_1.pin_p,rampVoltage2. p)
    annotation (Line(points={{-31,58},{-62,58},{-62,46}},
                                                        color={0,0,255}));
  connect(rampVoltage2.n, ground1.p)
    annotation (Line(points={{-62,26},{-62,0},{-16,0}}, color={0,0,255}));
  connect(hTS_Piline3_1.pin_n, ground1.p) annotation (Line(points={{-13,58},{-4,
          58},{-4,0},{-16,0}}, color={0,0,255}));
  connect(thermalConductor3.port_a,fixedTemperature3. port)
    annotation (Line(points={{174,82},{198,82}}, color={191,0,0}));
  connect(thermalConductor3.port_b,hTS_Piline3_2. port_a)
    annotation (Line(points={{154,82},{134,82},{134,64}},
                                                      color={191,0,0}));
  connect(hTS_Piline3_2.pin_p, sineVoltage.p)
    annotation (Line(points={{125,60},{94,60},{94,48}}, color={0,0,255}));
  connect(hTS_Piline3_2.pin_n, ground2.p) annotation (Line(points={{143,60},{
          152,60},{152,0},{122,0}}, color={0,0,255}));
  connect(sineVoltage.n, ground2.p) annotation (Line(points={{94,28},{94,0},{
          122,0},{122,0}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{240,
            100}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            240,100}})),
    experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
end StabilityTest_withLoad;
