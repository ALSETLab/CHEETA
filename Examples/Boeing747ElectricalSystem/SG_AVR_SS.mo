within CHEETA.Examples.Boeing747ElectricalSystem;
model SG_AVR_SS
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.4,11900; 0.5,
        12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 6,0.0; 10,0.0],
      timeScale=3600) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-60,20})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=Modelica.Constants.pi/30)
                                                annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-20,20})));
  Modelica.Electrical.MultiPhase.Basic.Resistor resistor(R={100,100,100})
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,-20})));
  Modelica.Electrical.MultiPhase.Basic.Star star annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-40,-20})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        origin={-74,-20},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Blocks.Interfaces.RealInput u
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Aircraft.Electrical.Machines.Boeing747_SG_disturbance_input
    boeing747_SG_disturbance_input
    annotation (Placement(transformation(extent={{24,12},{70,-12}})));
equation
  connect(RPMtoRPS.u, timeTable.y)
    annotation (Line(points={{-32,20},{-49,20}}, color={0,0,127}));
  connect(ground.p, star.pin_n)
    annotation (Line(points={{-64,-20},{-50,-20}},
                                                 color={0,0,255}));
  connect(resistor.plug_n, star.plug_p)
    annotation (Line(points={{-10,-20},{-30,-20}},
                                                 color={0,0,255}));
  connect(u, boeing747_SG_disturbance_input.Vd) annotation (Line(points={{-120,
          0},{-40,0},{-40,-7.2},{21.24,-7.2}}, color={0,0,127}));
  connect(boeing747_SG_disturbance_input.plugSupply, resistor.plug_p)
    annotation (Line(points={{47,-12.96},{47,-20},{10,-20}}, color={0,0,255}));
  connect(RPMtoRPS.y, boeing747_SG_disturbance_input.w_ref) annotation (Line(
        points={{-9,20},{8,20},{8,7.2},{21.24,7.2}}, color={0,0,127}));
  connect(y, boeing747_SG_disturbance_input.y) annotation (Line(points={{110,0},
          {88,0},{88,4.44089e-16},{72.3,4.44089e-16}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{100,40}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{100,
            40}})),
    experiment(
      StopTime=36000,
      __Dymola_NumberOfIntervals=5000,
      Tolerance=1e-06),
    Documentation(info="<html>
<p>synchronous generator, avr, synchronous </p>
</html>"));
end SG_AVR_SS;
