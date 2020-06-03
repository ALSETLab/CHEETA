within CHEETA.Aircraft.Electrical.Interfaces;
connector ElectricV_p "Electric vector terminal p ('positive')"
  parameter Integer m(min=1)=2 "Number of single contacts";
  ElectricPower.Interfaces.Connectors.Electric[m] pin
    "Vector of single contacts";
annotation (defaultComponentName = "term_p",
  Documentation(revisions="<html><hr><p><font color=\"#E72614\"><b>Copyright &copy; 2004-2019, MODELON AB</b></font> <font color=\"#AFAFAF\"><br /><br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br /> This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br /> or by other means have their origin from any Modelon Library. </font></p></html>",info="<html>
<p>Electric connector with a vector of 'pin's, ('positive').</p>
</html>"),
  Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Polygon(
          points={{-100,0},{0,-100},{100,0},{0,100},{-100,0}},
          lineColor={0,0,255},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid)}),
  Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={
        Text(
          extent={{-100,120},{120,60}},
          lineColor={0,0,255},
          textString="%name"),
        Polygon(
          points={{-60,0},{0,-60},{60,0},{0,60},{-60,0}},
          lineColor={0,0,255},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-50,52},{50,-48}},
          lineColor={255,255,255},
          pattern=LinePattern.None,
          textString="%m")}));
end ElectricV_p;
