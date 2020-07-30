within CHEETA.Examples.Boeing747ElectricalSystem;
model SG_AVR
  Aircraft.Electrical.Machines.Boeing747_SG_AVR sG_AVR
    annotation (Placement(transformation(extent={{-8,34},{10,26}})));
  Modelica.Blocks.Math.Gain RPMtoRPS(k=3.14/30) annotation (Placement(
        transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={30,18})));
  Modelica.Blocks.Sources.TimeTable timeTable(table=[0.0,0; 0.4,11900; 0.5,
        12000; 1,12000; 2,12000; 3,10000; 4.5,18000; 6,1; 6,0.0; 10,0.0],
      timeScale=3600) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={62,2})));
  Modelica.Electrical.MultiPhase.Basic.Resistor resistor(R={100,100,100})
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=270,
        origin={1,17})));
  Modelica.Electrical.MultiPhase.Basic.Star star annotation (Placement(
        transformation(
        extent={{-5,-5},{5,5}},
        rotation=270,
        origin={1,3})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        origin={1,-15},
        extent={{-7,-7},{7,7}},
        rotation=0)));
equation
  connect(timeTable.y, RPMtoRPS.u)
    annotation (Line(points={{51,2},{30,2},{30,10.8}}, color={0,0,127}));
  connect(ground.p,star. pin_n)
    annotation (Line(points={{1,-8},{1,-2}},     color={0,0,255}));
  connect(star.plug_p, resistor.plug_n)
    annotation (Line(points={{1,8},{1,12}}, color={0,0,255}));
  connect(sG_AVR.plugSupply1, resistor.plug_p)
    annotation (Line(points={{1,25.68},{1,22}}, color={0,0,255}));
  connect(sG_AVR.w_ref1, RPMtoRPS.y) annotation (Line(points={{11.8,32.4},{30,
          32.4},{30,24.6}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-20},{80,40}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-40,-20},{80,
            40}})),
    experiment(
      StopTime=36000,
      __Dymola_NumberOfIntervals=5000,
      Tolerance=1e-06),
    Documentation(info="<html>
<p>synchronous generator with AVR</p>
</html>"));
end SG_AVR;
