within CHEETA.Aircraft.Electrical.FuelCell;
model SimulinkFuelCell "Basic hydrogen fuel cell from MATLAB"

  import Modelica.Constants.k;
  import Modelica.Constants.h;

  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=V)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={34,-22})),__Dymola_choicesAllMatching=true);
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={34,2})));
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (Placement(
        transformation(extent={{60,30},{80,50}}), iconTransformation(extent={{60,30},
            {80,50}})));
  parameter Modelica.SIunits.Resistance R "Internal resistance";
  parameter Modelica.SIunits.Voltage V=1000 "Fuel cell voltage";




  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p1
                                                          annotation (Placement(
        transformation(extent={{54,-50},{74,-30}}),
                                                  iconTransformation(extent={{60,-50},
            {80,-30}})));
  Modelica.Electrical.Analog.Semiconductors.Diode diode annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={34,28})));
equation

  connect(constantVoltage.p, resistor.p)
    annotation (Line(points={{34,-12},{34,-8}}, color={0,0,255}));
  connect(pin_p1, constantVoltage.n)
    annotation (Line(points={{64,-40},{34,-40},{34,-32}}, color={0,0,255}));
  connect(pin_p, pin_p) annotation (Line(points={{70,40},{68,40},{68,40},{70,40}},
        color={0,0,255}));
  connect(resistor.n, diode.p)
    annotation (Line(points={{34,12},{34,18}}, color={0,0,255}));
  connect(diode.n, pin_p)
    annotation (Line(points={{34,38},{34,40},{70,40}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},
            {60,60}}), graphics={Rectangle(extent={{-60,60},{60,-60}},
            lineColor={28,108,200}), Bitmap(
          extent={{-88,-52},{90,52}},
          imageSource="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",
          fileName="modelica://CHEETA/Images/Fuel Cell/fuel_cell_stack_ic.png")}),
                                       Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-60,-60},{60,60}})),
    Documentation(revisions="<html>
<p>Basic hydrogen fuel cell based off of MATLAB [1].</p>
<p><img src=\"modelica://CHEETA/Images/Fuel Cell/fcmodel_simplified.gif\"/></p>
<p><br>[1] <a href=\"https://www.mathworks.com/help/physmod/sps/powersys/ref/fuelcellstack.html\">https://www.mathworks.com/help/physmod/sps/powersys/ref/fuelcellstack.html</a></p>
</html>"));
end SimulinkFuelCell;
