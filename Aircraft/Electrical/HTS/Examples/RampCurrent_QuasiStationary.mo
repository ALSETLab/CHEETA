within CHEETA.Aircraft.Electrical.HTS.Examples;
model RampCurrent_QuasiStationary
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor1(G=0.1)
           annotation (Placement(transformation(extent={{10,16},{-10,36}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor
                                        heatCapacitor(C=10, T(
      start=77,
      fixed=true,
      displayUnit="K"))
    annotation (Placement(transformation(extent={{32,16},{52,-4}})));
  Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor
                                         temperatureSensor annotation (Placement(
        transformation(
        origin={42,46},
        extent={{-10,-10},{10,10}},
        rotation=90)));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
                                           thermalConductor(G=0.1)
                                                        annotation (Placement(
        transformation(extent={{72,16},{92,36}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{134,16},{114,36}})));
  Modelica.Blocks.Sources.Constant const(k=77)
    annotation (Placement(transformation(extent={{180,16},{160,36}})));
  HTS_QuasiStationary hTS_QuasiStationary(
    l=1,
    I_crit=1000,
    R_L=0.1,
    G_d=0) annotation (Placement(transformation(extent={{-48,6},{-32,-2}})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Sources.CurrentSource
    currentSource(
    f=60,
    I=100,
    phi=0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-60,-18})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Resistor resistor(R_ref=
       100) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-22,-20})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground
    annotation (Placement(transformation(extent={{-70,-70},{-50,-50}})));
equation
  connect(thermalConductor.port_a, thermalConductor1.port_a)
    annotation (Line(points={{72,26},{10,26}}, color={191,0,0}));
  connect(heatCapacitor.port, thermalConductor1.port_a)
    annotation (Line(points={{42,16},{42,26},{10,26}}, color={191,0,0}));
  connect(temperatureSensor.port, thermalConductor1.port_a)
    annotation (Line(points={{42,36},{42,26},{10,26}}, color={191,0,0}));
  connect(prescribedTemperature.port, thermalConductor.port_b)
    annotation (Line(points={{114,26},{92,26}}, color={191,0,0}));
  connect(prescribedTemperature.T, const.y)
    annotation (Line(points={{136,26},{159,26}}, color={0,0,127}));
  connect(hTS_QuasiStationary.port_a, thermalConductor1.port_b)
    annotation (Line(points={{-39.8,6},{-39.8,26},{-10,26}}, color={191,0,0}));
  connect(hTS_QuasiStationary.pin_p, currentSource.pin_n)
    annotation (Line(points={{-48.2,2},{-60,2},{-60,-8}}, color={85,170,255}));
  connect(hTS_QuasiStationary.pin_n, resistor.pin_p)
    annotation (Line(points={{-32,2},{-22,2},{-22,-10}}, color={85,170,255}));
  connect(currentSource.pin_p, ground.pin)
    annotation (Line(points={{-60,-28},{-60,-50}}, color={85,170,255}));
  connect(resistor.pin_n, ground.pin) annotation (Line(points={{-22,-30},{-22,
          -42},{-60,-42},{-60,-50}}, color={85,170,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RampCurrent_QuasiStationary;
