within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampCurrent
  HTS_Piline4 hTS_Piline4_1(
    l=4,
    n=20,
    I_c0=1000,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=0.001,
    G_d=0,
    a=0.1,
    b=0.5,
    P=0.1) annotation (Placement(transformation(extent={{-28,62},{-12,54}})));
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent(
    I=2000,
    duration=5,
    offset=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-62,36})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor2(G=0.1)
           annotation (Placement(transformation(extent={{18,70},{-2,90}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature2(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{62,70},{42,90}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-26,-20},{-6,0}})));
equation
  connect(thermalConductor2.port_a,fixedTemperature2. port)
    annotation (Line(points={{18,80},{42,80}},   color={191,0,0}));
  connect(thermalConductor2.port_b,hTS_Piline4_1. port_a)
    annotation (Line(points={{-2,80},{-20,80},{-20,62}}, color={191,0,0}));
  connect(hTS_Piline4_1.pin_n, ground1.p) annotation (Line(points={{-11,58},{-4,
          58},{-4,0},{-16,0}}, color={0,0,255}));
  connect(rampCurrent.n,hTS_Piline4_1. pin_p)
    annotation (Line(points={{-62,46},{-62,58},{-29,58}}, color={0,0,255}));
  connect(rampCurrent.p, ground1.p) annotation (Line(points={{-62,26},{-62,0},{
          -16,0}},         color={0,0,255}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-20},{120,
            100}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-20},{120,
            100}})),
    experiment(
      StopTime=50,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end RampCurrent;
