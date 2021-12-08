within CHEETA.Aircraft.Electrical.FuelCell.PEM;
block Inv "1/x-1"
  extends Modelica.Blocks.Interfaces.SISO;
  parameter Boolean generateEvent=false
    "Choose whether events shall be generated" annotation (Evaluate=true);
equation
  //y = abs(u);
  y = 1/u -1;
  annotation (
    defaultComponentName="abs1",
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}})),
    Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong>
as <em>absolute value</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>abs</strong>( u );
</pre></blockquote>
<p>
The Boolean parameter generateEvent decides whether Events are generated at zero crossing (Modelica specification before 3) or not.
</p>
</html>"));
end Inv;
