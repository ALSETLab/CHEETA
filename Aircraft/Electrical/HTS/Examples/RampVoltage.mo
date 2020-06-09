within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampVoltage
  HTS_Piline4 hTS_Piline4_1(
    l=4,
    n=20,
    I_c0=2000,
    A=1,
    I_crit=1000,
    T_c(displayUnit="K") = 102,
    R_L=0.01,
    G_d=0,
    P=0.1,
    f=0.1) annotation (Placement(transformation(extent={{112,64},{128,56}})));
  Modelica.Electrical.Analog.Sources.RampVoltage rampVoltage1(
    V=1.5,
    duration=5,
    offset=0.05)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={78,36})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor1(G=0.1)
           annotation (Placement(transformation(extent={{160,72},{140,92}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature1(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{204,72},{184,92}})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{116,-18},{136,2}})));
equation
  connect(thermalConductor1.port_a,fixedTemperature1. port)
    annotation (Line(points={{160,82},{184,82}}, color={191,0,0}));
  connect(hTS_Piline4_1.pin_n,ground2. p) annotation (Line(points={{129,60},{
          138,60},{138,2},{126,2}},
                               color={0,0,255}));
  connect(rampVoltage1.p,hTS_Piline4_1. pin_p)
    annotation (Line(points={{78,46},{78,60},{111,60}}, color={0,0,255}));
  connect(rampVoltage1.n, ground2.p)
    annotation (Line(points={{78,26},{78,2},{126,2}}, color={0,0,255}));
  connect(thermalConductor1.port_b, hTS_Piline4_1.port_a) annotation (Line(
        points={{140,82},{120.2,82},{120.2,64}}, color={191,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-20},{220,
            100}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-20},{220,
            100}})),
    experiment(
      StopTime=10,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end RampVoltage;
