within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampCurrent_RLoad_ThermalLimit
  LiquidCooled.HTS_filmboiling hTS_filmboiling2_1(
    l=4,
    n=15.2,
    I_c0=1000,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=1e-3,
    G_d=0,
    a=0.1,
    b=0.5,
    P=0.1) annotation (Placement(transformation(extent={{-46,12},{-30,4}})));
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent(
    I=1000,
    duration=5,
    offset=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-70,-24})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=100) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-10,-24})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-34,-74},{-14,-54}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{-14,32},{-34,52}})));
  Modelica.Blocks.Sources.Constant const(k=77)
    annotation (Placement(transformation(extent={{32,32},{12,52}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=0)   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={146,-24})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{122,-74},{142,-54}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{142,32},{122,52}})));
  Modelica.Blocks.Sources.Constant const1(k=77)
    annotation (Placement(transformation(extent={{188,32},{168,52}})));
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent1(
    I=1000,
    duration=5,
    offset=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={70,-28})));
  GasCooled.HTS_GasCooling hTS_GasCooling(
    l=4,
    n=15.2,
    I_c0=1000,
    A=0.1,
    A_cu=0.1,
    I_crit=10000,
    T_c(displayUnit="K"),
    R_L=1e-3,
    G_d=0,
    a=0.1,
    b=0.5) annotation (Placement(transformation(extent={{104,4},{120,-4}})));
equation
  connect(rampCurrent.n, hTS_filmboiling2_1.pin_p)
    annotation (Line(points={{-70,-14},{-70,8},{-47,8}}, color={0,0,255}));
  connect(rampCurrent.p,ground2. p) annotation (Line(points={{-70,-34},{-70,-54},
          {-24,-54}},      color={0,0,255}));
  connect(hTS_filmboiling2_1.pin_n, resistor1.p)
    annotation (Line(points={{-29,8},{-10,8},{-10,-14}}, color={0,0,255}));
  connect(resistor1.n,ground2. p)
    annotation (Line(points={{-10,-34},{-10,-54},{-24,-54}},
                                                        color={0,0,255}));
  connect(prescribedTemperature.T,const. y)
    annotation (Line(points={{-12,42},{11,42}},  color={0,0,127}));
  connect(prescribedTemperature.port, hTS_filmboiling2_1.port_a) annotation (
      Line(points={{-34,42},{-37.8,42},{-37.8,12}}, color={191,0,0}));
  connect(resistor2.n,ground1. p)
    annotation (Line(points={{146,-34},{146,-54},{132,-54}},
                                                        color={0,0,255}));
  connect(prescribedTemperature1.T, const1.y)
    annotation (Line(points={{144,42},{167,42}}, color={0,0,127}));
  connect(ground1.p, rampCurrent1.p)
    annotation (Line(points={{132,-54},{70,-54},{70,-38}}, color={0,0,255}));
  connect(hTS_GasCooling.pin_p, rampCurrent1.n)
    annotation (Line(points={{103,0},{70,0},{70,-18}}, color={0,0,255}));
  connect(hTS_GasCooling.pin_n, resistor2.p)
    annotation (Line(points={{121,0},{146,0},{146,-14}}, color={0,0,255}));
  connect(prescribedTemperature1.port, hTS_GasCooling.port_a) annotation (Line(
        points={{122,42},{118,42},{118,40},{112.2,40},{112.2,4}}, color={191,0,
          0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{220,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{220,
            100}})),
    experiment(
      StopTime=10,
      __Dymola_NumberOfIntervals=100000,
      __Dymola_Algorithm="dassl"),
    __Dymola_experimentFlags(
      Advanced(
        EvaluateAlsoTop=false,
        GenerateVariableDependencies=false,
        OutputModelicaCode=false),
      Evaluate=true,
      OutputCPUtime=true,
      OutputFlatModelica=false));
end RampCurrent_RLoad_ThermalLimit;
