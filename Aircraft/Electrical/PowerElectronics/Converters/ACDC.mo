within CHEETA.Aircraft.Electrical.PowerElectronics.Converters;
model ACDC "3 phase AC to single phase DC rectifier"
  Modelica.Electrical.MultiPhase.Basic.Star star(m=3) annotation (Placement(
        transformation(
        origin={-24,66},
        extent={{-10,10},{10,-10}},
        rotation=0)));
  AircraftPowerSystem.Components.Conversion.Switches.Diode_Snubber diode_Snubber
    annotation (Placement(transformation(
        extent={{-9,-8},{9,8}},
        rotation=180,
        origin={-51,48})));
  AircraftPowerSystem.Components.Conversion.Switches.Diode_Snubber diode_Snubber1
    annotation (Placement(transformation(
        extent={{-9,-8},{9,8}},
        rotation=180,
        origin={-51,20})));
  parameter Modelica.SIunits.Resistance Rcond=1e-5
    "Forward state-on differential resistance (closed resistance)";
  parameter Modelica.SIunits.Voltage Vt=0 "Forward threshold voltage";
  parameter Modelica.SIunits.Conductance Gof=1e-5
    "Backward state-off conductance (opened conductance)";
  parameter Modelica.SIunits.Resistance R1=1e-3 "Snubber Resistance";
  parameter Modelica.SIunits.Capacitance C1=1e-6 "SnubberCapacitance";
  Modelica.Electrical.MultiPhase.Basic.Star star1(m=3)
                                                      annotation (Placement(
        transformation(
        origin={-16,-6},
        extent={{-10,10},{10,-10}},
        rotation=0)));
  Modelica.Electrical.MultiPhase.Interfaces.PositivePlug positivePlug
    annotation (Placement(transformation(extent={{-114,24},{-94,44}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
    annotation (Placement(transformation(extent={{34,56},{54,76}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
    annotation (Placement(transformation(extent={{34,-16},{54,4}})));
equation
  connect(diode_Snubber.negativePlug, star.plug_p) annotation (Line(points={{-51.2,
          56},{-52,56},{-52,66},{-34,66}}, color={0,0,255}));
  connect(diode_Snubber1.negativePlug, diode_Snubber.positivePlug)
    annotation (Line(points={{-51.2,28},{-51,28},{-51,39.8}}, color={0,
          0,255}));
  connect(star1.plug_p, diode_Snubber1.positivePlug) annotation (Line(
        points={{-26,-6},{-50,-6},{-50,11.8},{-51,11.8}}, color={0,0,
          255}));
  connect(positivePlug, diode_Snubber.positivePlug) annotation (Line(
        points={{-104,34},{-51,34},{-51,39.8}}, color={0,0,255}));
  connect(pin_p, star.pin_n)
    annotation (Line(points={{44,66},{-14,66}}, color={0,0,255}));
  connect(pin_n, star1.pin_n)
    annotation (Line(points={{44,-6},{-6,-6}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},
            {40,80}})),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{40,80}})),
    experiment(StopTime=0.1, __Dymola_NumberOfIntervals=5000),
    Documentation(info="<html>
<p>This rectifier converter interfaces a 3 phase system to a single phase DC system.</p>
</html>"));
end ACDC;
