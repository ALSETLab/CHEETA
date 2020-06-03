within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampVI_R_load
  HTS_Piline3                            hTS_Piline3_2(
    l=4,
    n=2,
    I_c0=1000,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=1e-3,
    G_d=0,
    a=0.1,
    b=0.5,
    P=0.1)       annotation (Placement(transformation(extent={{-58,64},{-42,56}})));
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent(
    I=250,
    duration=5,
    offset=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-80,30})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor1(G=0.1)
           annotation (Placement(transformation(extent={{0,70},{-20,90}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature1(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{44,70},{24,90}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-44,-20},{-24,0}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=100) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,30})));
  HTS_Piline3                            hTS_Piline3_1(
    l=4,
    n=20,
    I_c0=1000,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=1e-3,
    G_d=0,
    a=0.1,
    b=0.5,
    P=0.1)       annotation (Placement(transformation(extent={{88,64},{104,56}})));
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent1(
    I=250,
    duration=5,
    offset=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={60,30})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor2(G=0.1)
           annotation (Placement(transformation(extent={{136,72},{116,92}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature2(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{180,72},{160,92}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{92,-20},{112,0}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=50)  annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={120,30})));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=1e-3) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={158,30})));
  HTS_Piline3                            hTS_Piline3_3(
    l=4,
    n=20,
    I_c0=1000,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=1e-3,
    G_d=0,
    a=0.1,
    b=0.5,
    P=0.1)       annotation (Placement(transformation(extent={{238,64},{254,56}})));
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent2(
    I=250,
    duration=5,
    offset=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={210,30})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor3(G=0.1)
           annotation (Placement(transformation(extent={{286,72},{266,92}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature3(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{330,72},{310,92}})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{242,-20},{262,0}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor3(R=100) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={270,30})));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C=1e-3)
                                                               annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={308,30})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.001) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={340,30})));
equation
  connect(thermalConductor1.port_a,fixedTemperature1. port)
    annotation (Line(points={{0,80},{24,80}},    color={191,0,0}));
  connect(thermalConductor1.port_b,hTS_Piline3_2. port_a)
    annotation (Line(points={{-20,80},{-50,80},{-50,64}},
                                                      color={191,0,0}));
  connect(rampCurrent.n,hTS_Piline3_2. pin_p)
    annotation (Line(points={{-80,40},{-80,60},{-59,60}}, color={0,0,255}));
  connect(rampCurrent.p,ground2. p) annotation (Line(points={{-80,20},{-80,0},{
          -34,0}},         color={0,0,255}));
  connect(hTS_Piline3_2.pin_n, resistor1.p)
    annotation (Line(points={{-41,60},{-20,60},{-20,40}}, color={0,0,255}));
  connect(resistor1.n, ground2.p)
    annotation (Line(points={{-20,20},{-20,0},{-34,0}}, color={0,0,255}));
  connect(thermalConductor2.port_a,fixedTemperature2. port)
    annotation (Line(points={{136,82},{160,82}}, color={191,0,0}));
  connect(thermalConductor2.port_b,hTS_Piline3_1. port_a)
    annotation (Line(points={{116,82},{96,82},{96,64}},
                                                      color={191,0,0}));
  connect(rampCurrent1.n, hTS_Piline3_1.pin_p)
    annotation (Line(points={{60,40},{60,60},{87,60}}, color={0,0,255}));
  connect(rampCurrent1.p, ground1.p)
    annotation (Line(points={{60,20},{60,0},{102,0}}, color={0,0,255}));
  connect(hTS_Piline3_1.pin_n, resistor2.p)
    annotation (Line(points={{105,60},{120,60},{120,40}}, color={0,0,255}));
  connect(resistor2.n, ground1.p)
    annotation (Line(points={{120,20},{120,0},{102,0}}, color={0,0,255}));
  connect(capacitor.p, resistor2.p) annotation (Line(points={{158,40},{158,60},
          {120,60},{120,40}}, color={0,0,255}));
  connect(capacitor.n, ground1.p)
    annotation (Line(points={{158,20},{158,0},{102,0}}, color={0,0,255}));
  connect(thermalConductor3.port_a,fixedTemperature3. port)
    annotation (Line(points={{286,82},{310,82}}, color={191,0,0}));
  connect(thermalConductor3.port_b,hTS_Piline3_3. port_a)
    annotation (Line(points={{266,82},{246,82},{246,64}},
                                                      color={191,0,0}));
  connect(rampCurrent2.n,hTS_Piline3_3. pin_p)
    annotation (Line(points={{210,40},{210,60},{237,60}},
                                                       color={0,0,255}));
  connect(rampCurrent2.p,ground3. p)
    annotation (Line(points={{210,20},{210,0},{252,0}},
                                                      color={0,0,255}));
  connect(hTS_Piline3_3.pin_n,resistor3. p)
    annotation (Line(points={{255,60},{270,60},{270,40}}, color={0,0,255}));
  connect(resistor3.n,ground3. p)
    annotation (Line(points={{270,20},{270,0},{252,0}}, color={0,0,255}));
  connect(capacitor1.p, resistor3.p) annotation (Line(points={{308,40},{308,60},
          {270,60},{270,40}}, color={0,0,255}));
  connect(capacitor1.n, ground3.p)
    annotation (Line(points={{308,20},{308,0},{252,0}}, color={0,0,255}));
  connect(inductor.p, resistor3.p) annotation (Line(points={{340,40},{340,60},{
          270,60},{270,40}}, color={0,0,255}));
  connect(inductor.n, ground3.p)
    annotation (Line(points={{340,20},{340,0},{252,0}}, color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-20},{380,
            100}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-20},{380,
            100}})),
    experiment(
      StopTime=10,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end RampVI_R_load;
