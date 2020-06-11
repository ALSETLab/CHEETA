within CHEETA.Aircraft.Electrical;
model BusExt

  parameter Integer np(min=0) = 0 "Number of left connection"
    annotation (Dialog(connectorSizing=true), HideResult=true);
  parameter Integer nn(min=0) = 0 "Number of right connections"
    annotation (Dialog(connectorSizing=true), HideResult=true);
  Modelica.Electrical.Analog.Interfaces.PositivePin p[np] annotation (Placement(
      visible=true,
      transformation(
        origin={-20.0001,1},
        extent={{-9.99994,-99},{10.0001,99}},
        rotation=0),
      iconTransformation(
        origin={20,0},
        extent={{-4,-60},{4,60}},
        rotation=0)));
  Modelica.Electrical.Analog.Interfaces.NegativePin n[nn] annotation (Placement(
      visible=true,
      transformation(
        origin={20,0},
        extent={{-12,-100},{12,100}},
        rotation=0),
      iconTransformation(
        origin={0,0},
        extent={{-4,-60},{4,60}},
        rotation=0)));
  Real V "Bus voltage magnitude";
  Real I "Bus current magnitude";


equation
  if np > 1 then
    for i in 2:np loop
      connect(p[1], p[i]);
    end for;
  end if;
  if nn > 1 then
    for i in 2:nn loop
      connect(n[1], n[i]);
    end for;
  end if;
  if np > 0 and nn > 0 then
    connect(p[1], n[1]);
  end if;
   if np > 0 then
    V = p[1].v;
    I = p[1].i;
  elseif nn > 0 then
    V = n[1].v;
    I = n[1].i;
  else
    V = 0;
    I = 0;
   end if;




  annotation (
    Diagram(coordinateSystem(extent={{0,-100},{20,100}})),
    Icon(coordinateSystem(extent={{0,-100},{20,100}}, preserveAspectRatio=false),
        graphics={Rectangle(
          extent={{0,100},{20,-100}},
          lineColor={0,0,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid)}),
    Documentation(info="<html>
<table cellspacing=\"1\" cellpadding=\"1\" border=\"1\"><tr>
<td><p>Reference</p></td>
<td><p>None</p></td>
</tr>
<tr>
<td><p>Last update</p></td>
<td></td>
</tr>
<tr>
<td><p><br>Author</p></td>
<td></td>
</tr>
<tr>
<td><p><br>Contact</p></td>
<td></td>
</tr>
</table>
</html>"));
end BusExt;
