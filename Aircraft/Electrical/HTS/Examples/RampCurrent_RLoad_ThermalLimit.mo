within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampCurrent_RLoad_ThermalLimit
  HTS_Piline4                            hTS_Piline4_1(
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
    P=0.1)       annotation (Placement(transformation(extent={{-48,12},{-32,4}})));
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent(
    I=250,
    duration=5,
    offset=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-70,-24})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor1(G=0.1)
           annotation (Placement(transformation(extent={{10,16},{-10,36}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature1(T(
        displayUnit="K") = 77)
    annotation (Placement(transformation(extent={{54,16},{34,36}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=100) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-10,-24})));
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-34,-74},{-14,-54}})));
equation
  connect(thermalConductor1.port_a,fixedTemperature1. port)
    annotation (Line(points={{10,26},{34,26}},   color={191,0,0}));
  connect(thermalConductor1.port_b,hTS_Piline4_1. port_a)
    annotation (Line(points={{-10,26},{-39.8,26},{-39.8,12}},
                                                      color={191,0,0}));
  connect(rampCurrent.n,hTS_Piline4_1. pin_p)
    annotation (Line(points={{-70,-14},{-70,8},{-49,8}},  color={0,0,255}));
  connect(rampCurrent.p,ground2. p) annotation (Line(points={{-70,-34},{-70,-54},
          {-24,-54}},      color={0,0,255}));
  connect(hTS_Piline4_1.pin_n,resistor1. p)
    annotation (Line(points={{-31,8},{-10,8},{-10,-14}},  color={0,0,255}));
  connect(resistor1.n,ground2. p)
    annotation (Line(points={{-10,-34},{-10,-54},{-24,-54}},
                                                        color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RampCurrent_RLoad_ThermalLimit;
