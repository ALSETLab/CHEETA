within CHEETA.Aircraft.Electrical;
model BusBar "Busbar, 1-phase"

  output Modelica.SIunits.Voltage v(stateSelect=StateSelect.never);
  CHEETA.Aircraft.Electrical.Interfaces.ElectricV_p term(m=2) "Terminal"
    annotation (Placement(transformation(extent={{-7,-60},{7,60}},
          rotation=0)));

equation
  term.pin.i = zeros(2);
  v = term.pin[1].v - term.pin[2].v;
  annotation (defaultComponentName = "bus1",
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Text(
          extent={{-100,-90},{100,-130}},
          lineColor={0,0,0},
          textString="%name"), Rectangle(
          extent={{-10,80},{10,-80}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid)}),
    Documentation(
            info="<html>
<p>Calculates difference voltage conductor 1 - conductor 2.</p>
<p><img src=\"modelica://ElectricPower/Resources/Images/DC/Nodes/BusBar.png\"/></p>
</html>", revisions="<html>
 <hr><p><font color=\"#E72614\"><b>Copyright &copy; 2004-2019, MODELON AB</b></font> <font color=\"#AFAFAF\"><br /><br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br /> This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br /> or by other means have their origin from any Modelon Library. </font></p>
</html>"));
end BusBar;
