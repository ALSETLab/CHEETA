within CHEETA.Aircraft.Controls;
model MasterSystemControl

protected
  Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces.ControlBus
                                           controlBus annotation (Placement(
        transformation(
        origin={0,-100},
        extent={{-20,20},{20,-20}},
        rotation=0)));
public
  Modelica.Blocks.Sources.CombiTimeTable ReferenceSpeed
    annotation (Placement(transformation(extent={{-90,62},{-70,82}})));
  Modelica.Blocks.Interfaces.RealInput DCDC_voltage
    annotation (Placement(transformation(extent={{-140,22},{-100,62}})));
  Modelica.Blocks.Interfaces.RealInput DCDC_inverter
    annotation (Placement(transformation(extent={{-140,-8},{-100,32}})));
  Modelica.Blocks.Interfaces.RealInput inverter_voltage
    annotation (Placement(transformation(extent={{-140,-40},{-100,0}})));
  Modelica.Blocks.Interfaces.RealInput inverter_current
    annotation (Placement(transformation(extent={{-140,-72},{-100,-32}})));
equation
  connect(ReferenceSpeed.y[1], controlBus.realSignal1) annotation (Line(points={{-69,72},
          {0,72},{0,-102},{0.1,-102},{0.1,-100.1}},            color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Rectangle(
          extent={{-100,-100},{100,100}},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid), Rectangle(
          extent={{-98,-98},{98,98}},
          lineColor={0,0,0},
          pattern=LinePattern.Solid,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-70,-24},{-64,-20},{-64,2},{-72,-4},{-70,-24}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{-76,0},{-68,4},{-52,0},{-60,-6},{-76,0}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175}),
        Polygon(
          points={{-60,-8},{-60,-6},{-76,0},{-72,-24},{-70,-24},{-72,-4},{-60,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{-46,-8},{-40,-4},{-40,18},{-48,12},{-46,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{-52,16},{-44,20},{-28,16},{-36,10},{-52,16}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175}),
        Polygon(
          points={{-36,8},{-36,10},{-52,16},{-48,-8},{-46,-8},{-48,12},{-36,8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{-20,6},{-14,10},{-14,32},{-22,26},{-20,6}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{-26,30},{-18,34},{-2,30},{-10,24},{-26,30}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175}),
        Polygon(
          points={{-10,22},{-10,24},{-26,30},{-22,6},{-20,6},{-22,26},{-10,22}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{-70,-8},{-70,-24},{-4,-42},{-4,-26},{-70,-8}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{-4,-42},{66,18},{66,32},{-4,-26},{-4,-42}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={48,48,48}),
        Polygon(
          points={{-70,-8},{-4,-26},{66,32},{4,42},{-70,-8}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={95,95,95}),
        Polygon(
          points={{4,-30},{4,-32},{16,-34},{12,-56},{14,-58},{20,-32},{4,-30}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{4,-30},{12,-24},{28,-26},{20,-32},{4,-30}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175}),
        Polygon(
          points={{14,-58},{20,-54},{28,-26},{20,-32},{14,-58}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{28,-8},{28,-10},{40,-12},{36,-34},{38,-36},{44,-10},{28,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{38,-36},{44,-32},{52,-4},{44,-10},{38,-36}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{28,-8},{36,-2},{52,-4},{44,-10},{28,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175}),
        Polygon(
          points={{52,12},{52,10},{64,8},{60,-14},{62,-16},{68,10},{52,12}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={215,215,215}),
        Polygon(
          points={{62,-16},{68,-12},{76,16},{68,10},{62,-16}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={135,135,135}),
        Polygon(
          points={{52,12},{60,18},{76,16},{68,10},{52,12}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175})}), Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end MasterSystemControl;
