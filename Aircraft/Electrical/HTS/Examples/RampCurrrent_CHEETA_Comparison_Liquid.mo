within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampCurrrent_CHEETA_Comparison_Liquid
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent(
    I=90000,
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
    annotation (Placement(transformation(extent={{-4,26},{-24,46}})));
  Modelica.Blocks.Sources.Constant const(k=20)
    annotation (Placement(transformation(extent={{42,26},{22,46}})));
  LiquidCooled.HTS_filmboiling_Voltage_Hydrogen hydrogen(
    l=10,
    n=10.29,
    I_c0=37000,
    A=2.835e-4,
    epsilon_r=2.2,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 0,
    a(displayUnit="mm"),
    b(displayUnit="mm")) "liquid hydrogen cooling"
    annotation (Placement(transformation(extent={{-44,26},{-28,18}})));
  Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent1(
    I=50000,
    duration=15,
    offset=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={70,-22})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=0)   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={130,-22})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{136,28},{116,48}})));
  Modelica.Blocks.Sources.Constant const1(k=20)
    annotation (Placement(transformation(extent={{182,28},{162,48}})));
  LiquidCooled.HTS_filmboiling_Voltage_Nitrogen nitrogen(
    l=10,
    n=20,
    I_c0=37000,
    A=2.835e-4,
    epsilon_r=2.2,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 0,
    a(displayUnit="mm"),
    b(displayUnit="mm")) "liquid nitrogen cooling"
    annotation (Placement(transformation(extent={{94,26},{110,18}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{106,-74},{126,-54}})));
  Modelica.Electrical.Analog.Sources.RampVoltage rampVoltage(
    V=0.2,
    duration=15,
    offset=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={200,-24})));
  Modelica.Electrical.Analog.Basic.Resistor resistor3(R=0)   annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={260,-24})));
  Modelica.Electrical.Analog.Basic.Ground ground3
    annotation (Placement(transformation(extent={{236,-76},{256,-56}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature2
    annotation (Placement(transformation(extent={{266,26},{246,46}})));
  Modelica.Blocks.Sources.Constant const2(k=20)
    annotation (Placement(transformation(extent={{312,26},{292,46}})));
  LiquidCooled.HTS_filmboiling_Voltage hydrogen_V(
    l=10,
    n=20,
    I_c0=37000,
    A=2.835e-4,
    epsilon_r=2.2,
    T_c(displayUnit="K"),
    R_L=3.3e-3,
    G_d(displayUnit="kW") = 0,
    a(displayUnit="mm"),
    b(displayUnit="mm")) "liquid hydrogen cooling"
    annotation (Placement(transformation(extent={{226,26},{242,18}})));
equation
  connect(rampCurrent.p,ground2. p) annotation (Line(points={{-70,-34},{-70,-56},
          {-24,-56}},      color={0,0,255}));
  connect(resistor1.n,ground2. p)
    annotation (Line(points={{-10,-34},{-10,-56},{-24,-56}},
                                                        color={0,0,255}));
  connect(prescribedTemperature.T, const.y)
    annotation (Line(points={{-2,36},{21,36}},   color={0,0,127}));
  connect(hydrogen.pin_p, rampCurrent.n)
    annotation (Line(points={{-45,22},{-70,22},{-70,-14}}, color={0,0,255}));
  connect(hydrogen.pin_n, resistor1.p)
    annotation (Line(points={{-27,22},{-10,22},{-10,-14}}, color={0,0,255}));
  connect(hydrogen.port_a, prescribedTemperature.port) annotation (Line(points={{-35.8,
          26},{-35.8,36},{-24,36}},         color={191,0,0}));
  connect(rampCurrent1.p, ground1.p)
    annotation (Line(points={{70,-32},{70,-54},{116,-54}}, color={0,0,255}));
  connect(resistor2.n,ground1. p)
    annotation (Line(points={{130,-32},{130,-54},{116,-54}},
                                                        color={0,0,255}));
  connect(prescribedTemperature1.T, const1.y)
    annotation (Line(points={{138,38},{161,38}}, color={0,0,127}));
  connect(nitrogen.pin_p, rampCurrent1.n)
    annotation (Line(points={{93,22},{70,22},{70,-12}},  color={0,0,255}));
  connect(nitrogen.pin_n, resistor2.p)
    annotation (Line(points={{111,22},{130,22},{130,-12}}, color={0,0,255}));
  connect(nitrogen.port_a, prescribedTemperature1.port) annotation (Line(points={{102.2,
          26},{102.2,38},{116,38}},        color={191,0,0}));
  connect(resistor3.n,ground3. p)
    annotation (Line(points={{260,-34},{260,-56},{246,-56}},
                                                        color={0,0,255}));
  connect(prescribedTemperature2.T, const2.y)
    annotation (Line(points={{268,36},{291,36}}, color={0,0,127}));
  connect(hydrogen_V.pin_n, resistor3.p)
    annotation (Line(points={{243,22},{260,22},{260,-14}}, color={0,0,255}));
  connect(hydrogen_V.port_a, prescribedTemperature2.port) annotation (Line(
        points={{234.2,26},{234.2,36},{246,36}}, color={191,0,0}));
  connect(hydrogen_V.pin_p, rampVoltage.p)
    annotation (Line(points={{225,22},{200,22},{200,-14}}, color={0,0,255}));
  connect(rampVoltage.n, ground3.p)
    annotation (Line(points={{200,-34},{200,-56},{246,-56}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{320,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{320,
            100}}), graphics={Text(
          extent={{-72,70},{22,60}},
          lineColor={28,108,200},
          textString="Hydrogen"), Text(
          extent={{58,72},{152,62}},
          lineColor={28,108,200},
          textString="Nitrogen"),
                              Text(
          extent={{198,70},{292,60}},
          lineColor={28,108,200},
          textString="Hydrogen")}),
    experiment(
      StopTime=15,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end RampCurrrent_CHEETA_Comparison_Liquid;
