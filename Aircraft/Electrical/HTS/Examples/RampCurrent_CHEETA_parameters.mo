within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampCurrent_CHEETA_parameters
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent(
    I=10000.95,
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
  LiquidCooled.HTS_filmboiling_Current hTS_filmboiling2_2(
    l=10,
    n=5.29,
    I_c0=6000,
    A=2.835e-4,
    A_cu=5.04e-6,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 0,
    a(displayUnit="mm") = 0.00256,
    b(displayUnit="mm") = 0.00363,
    P(displayUnit="mm") = 0.1035)
           annotation (Placement(transformation(extent={{-46,24},{-30,16}})));
equation
  connect(rampCurrent.p,ground2. p) annotation (Line(points={{-70,-34},{-70,-56},
          {-24,-56}},      color={0,0,255}));
  connect(resistor1.n,ground2. p)
    annotation (Line(points={{-10,-34},{-10,-56},{-24,-56}},
                                                        color={0,0,255}));
  connect(prescribedTemperature.T, const.y)
    annotation (Line(points={{50,40},{73,40}},   color={0,0,127}));
  connect(hTS_filmboiling2_2.pin_p, rampCurrent.n)
    annotation (Line(points={{-47,20},{-70,20},{-70,-14}}, color={0,0,255}));
  connect(hTS_filmboiling2_2.pin_n, resistor1.p)
    annotation (Line(points={{-29,20},{-10,20},{-10,-14}}, color={0,0,255}));
  connect(hTS_filmboiling2_2.port_a, prescribedTemperature.port)
    annotation (Line(points={{-37.8,24},{-37.8,40},{28,40}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{120,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{120,
            100}})),
    experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end RampCurrent_CHEETA_parameters;
