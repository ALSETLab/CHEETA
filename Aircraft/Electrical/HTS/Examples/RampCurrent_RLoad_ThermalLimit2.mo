within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampCurrent_RLoad_ThermalLimit2
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent(
    I=1000,
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
  Modelica.Blocks.Sources.Constant const(k=77)
    annotation (Placement(transformation(extent={{94,30},{74,50}})));
  HTS_filmboiling2                       hTS_filmboiling2_2(
    l=4,
    n=5.29,
    I_c0=1000,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=1e-3,
    G_d=0,
    a=0.1,
    b=0.5,
    P=0.1)       annotation (Placement(transformation(extent={{-48,24},{-32,16}})));
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
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=20,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Euler"));
end RampCurrent_RLoad_ThermalLimit2;
