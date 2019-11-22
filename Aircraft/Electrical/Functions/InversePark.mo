within CHEETA.Aircraft.Electrical.Functions;
model InversePark "Inverse Park transformation (dq0 to ABC)"
  Modelica.Blocks.Interfaces.RealInput id "Input signal-vector" annotation (
      Placement(transformation(extent={{-120,50},{-100,70}}, rotation=0)));
  Modelica.Blocks.Interfaces.RealInput theta "Transformation angle"
    annotation (Placement(transformation(
        origin={0,110},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Blocks.Interfaces.RealInput iq "Input signal-vector" annotation (
      Placement(transformation(extent={{-120,-10},{-100,10}}, rotation=0)));
  Modelica.Blocks.Interfaces.RealInput i0 "Input signal-vector" annotation (
      Placement(transformation(extent={{-120,-70},{-100,-50}}, rotation=0)));


  Modelica.Blocks.Interfaces.RealOutput ia
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Interfaces.RealOutput ib
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealOutput ic
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
protected
  parameter Real pi = Modelica.Constants.pi;

equation

  ia = id*cos(theta)+iq*sin(theta)+i0/sqrt(2);
  ib = id*cos(theta-2*pi/3)+iq*sin(theta-2*pi/3)+i0/sqrt(2);
  ic = id*cos(theta+2*pi/3)+iq*sin(theta+2*pi/3)+i0/sqrt(2);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
        Text(
          extent={{-92,38},{60,-34}},
          lineColor={28,108,200},
          textString="Park"),
        Text(
          extent={{6,70},{126,16}},
          lineColor={28,108,200},
          textString="-1")}),                                    Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Inverse Park transformation (dq0 to ABC)</p>
<p><br>References: Power System Analysis, 3rd edition, Hadi Saadat</p>
</html>"));
end InversePark;
