within CHEETA.Aircraft.Electrical.FuelCell.Examples;
model FuelCell_EquivalentCircuit
  "Electrical circuit equivalent for the fuel cell"
  Modelica.Electrical.Analog.Basic.Ground ground2
    annotation (Placement(transformation(extent={{-50,-38},{-30,-18}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.0146)
    annotation (Placement(transformation(extent={{-2,4},{18,24}})));
  Modelica.Electrical.Analog.Sources.RampVoltage     rampVoltage(
    V=100,
    duration=2,
    offset=1000)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-40,2})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=0.2)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,-2})));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(v(fixed=true, start=10),
      C=1.6635)                                                  annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={8,36})));
  HTS.LiquidCooled.HTS_filmboiling_Voltage2
                                        hTS_filmboiling3_1(
    l=1,
    n=20,
    I_c0=7800,
    A=0.1,
    A_cu=0.1,
    I_crit=100000,
    T_c(displayUnit="K"),
    R_L=1e-4,
    G_d=0,
    a=0.1,
    b=0.5,
    P=1)  annotation (Placement(transformation(extent={{22,-22},{6,-14}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature1
    annotation (Placement(transformation(extent={{44,-78},{24,-58}})));
  Modelica.Blocks.Sources.Constant const1(k=20)
    annotation (Placement(transformation(extent={{90,-78},{70,-58}})));
equation
  connect(resistor.n, resistor1.p)
    annotation (Line(points={{18,14},{40,14},{40,8}}, color={0,0,255}));
  connect(ground2.p, rampVoltage.n)
    annotation (Line(points={{-40,-18},{-40,-8}}, color={0,0,255}));
  connect(rampVoltage.p, resistor.p)
    annotation (Line(points={{-40,12},{-40,14},{-2,14}}, color={0,0,255}));
  connect(capacitor.p, resistor.p) annotation (Line(points={{-2,36},{-12,36},{
          -12,14},{-2,14}}, color={0,0,255}));
  connect(capacitor.n, resistor1.p) annotation (Line(points={{18,36},{26,36},{
          26,14},{40,14},{40,8}}, color={0,0,255}));
  connect(prescribedTemperature1.T, const1.y)
    annotation (Line(points={{46,-68},{69,-68}}, color={0,0,127}));
  connect(hTS_filmboiling3_1.port_a, prescribedTemperature1.port) annotation (
      Line(points={{13.8,-22},{14,-22},{14,-68},{24,-68}}, color={191,0,0}));
  connect(hTS_filmboiling3_1.pin_n, ground2.p)
    annotation (Line(points={{5,-18},{-40,-18}}, color={0,0,255}));
  connect(hTS_filmboiling3_1.pin_p, resistor1.n)
    annotation (Line(points={{23,-18},{40,-18},{40,-12}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuelCell_EquivalentCircuit;
