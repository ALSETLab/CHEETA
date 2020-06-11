within CHEETA.Aircraft.Electrical.HTS.Examples;
model StabilityTest_withLoad
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
    P=0.1)       annotation (Placement(transformation(extent={{126,64},{142,56}})));
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
    experiment(
      StopTime=0.1,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end StabilityTest_withLoad;
