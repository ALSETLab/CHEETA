within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampCurrent_RLoad_ThermalLimit3
  HTS_Piline6                            hTS_Piline6_1(
    l=4,
    n=15.8,
    I_c0=5000,
    A=0.1,
    A_cu=0.1,
    I_crit=1000,
    T_c(displayUnit="K"),
    R_L=1e-3,
    G_d=0,
    a=0.1,
    b=0.5,
    P=0.1)       annotation (Placement(transformation(extent={{-48,12},{-32,4}})));
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
  Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor
                                         temperatureSensor annotation (Placement(
        transformation(
        origin={44,50},
        extent={{-10,-10},{10,10}},
        rotation=90)));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=1)
                                                        annotation (Placement(
        transformation(extent={{72,16},{92,36}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{134,16},{114,36}})));
  Modelica.Blocks.Sources.Constant const(k=77)
    annotation (Placement(transformation(extent={{180,16},{160,36}})));
equation
  connect(rampCurrent.n, hTS_Piline6_1.pin_p)
    annotation (Line(points={{-70,-14},{-70,8},{-49,8}}, color={0,0,255}));
  connect(rampCurrent.p,ground2. p) annotation (Line(points={{-70,-34},{-70,-54},
          {-24,-54}},      color={0,0,255}));
  connect(hTS_Piline6_1.pin_n, resistor1.p)
    annotation (Line(points={{-31,8},{-10,8},{-10,-14}}, color={0,0,255}));
  connect(resistor1.n,ground2. p)
    annotation (Line(points={{-10,-34},{-10,-54},{-24,-54}},
                                                        color={0,0,255}));
  connect(prescribedTemperature.port, thermalConductor.port_b)
    annotation (Line(points={{114,26},{92,26}}, color={191,0,0}));
  connect(prescribedTemperature.T, const.y)
    annotation (Line(points={{136,26},{159,26}}, color={0,0,127}));
  connect(thermalConductor.port_a, hTS_Piline6_1.port_a)
    annotation (Line(points={{72,26},{-39.8,26},{-39.8,12}}, color={191,0,0}));
  connect(temperatureSensor.port, hTS_Piline6_1.port_a) annotation (Line(points
        ={{44,40},{44,26},{-39.8,26},{-39.8,12}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RampCurrent_RLoad_ThermalLimit3;
