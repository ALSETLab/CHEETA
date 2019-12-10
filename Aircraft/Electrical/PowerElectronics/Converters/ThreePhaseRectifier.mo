within CHEETA.Aircraft.Electrical.PowerElectronics.Converters;
model ThreePhaseRectifier "3 phase rectifier model"
  Modelica.Electrical.MultiPhase.Ideal.IdealDiode
                   idealDiode1(
    m=m,
    Ron=Ron,
    Goff=Goff,
    Vknee=Vknee)          annotation (Placement(transformation(
        origin={0,20},
        extent={{-10,-10},{10,10}},
        rotation=90)));
  Modelica.Electrical.MultiPhase.Basic.Star
             star1(m=m) annotation (Placement(transformation(
        origin={0,50},
        extent={{-10,-10},{10,10}},
        rotation=90)));
  Modelica.Electrical.MultiPhase.Ideal.IdealDiode
                   idealDiode2(
    m=m,
    Ron=Ron,
    Goff=Goff,
    Vknee=Vknee)          annotation (Placement(transformation(
        origin={0,-20},
        extent={{-10,-10},{10,10}},
        rotation=90)));
  Modelica.Electrical.MultiPhase.Basic.Star
             star2(m=m) annotation (Placement(transformation(
        origin={0,-50},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plug_p1
                                 "Positive polyphase electrical plug with m pins"
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin load_n
    "Negative pin connection for load"
    annotation (Placement(transformation(extent={{-10,-94},{10,-74}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin load_p
    "Positive pin connection for load"
    annotation (Placement(transformation(extent={{-10,74},{10,94}})));
  parameter Integer m=m "Number of phases";
  parameter Modelica.SIunits.Resistance Ron[m]=fill(Ron, m)
    "Closed diode resistance";
  parameter Modelica.SIunits.Conductance Goff[m]=fill(Goff, m)
    "Opened diode conductance";
  parameter Modelica.SIunits.Voltage Vknee[m]=fill(Vknee, m)
    "Threshold voltage";
equation
  connect(idealDiode1.plug_n,star1. plug_p)
    annotation (Line(points={{0,30},{0,40}},   color={0,0,255}));
  connect(idealDiode2.plug_p,star2. plug_p) annotation (Line(points={{0,-30},{0,
          -40}},              color={0,0,255}));
  connect(idealDiode1.plug_p,idealDiode2. plug_n)
    annotation (Line(points={{0,10},{0,-10}},   color={0,0,255}));
  connect(idealDiode1.plug_p, plug_p1)
    annotation (Line(points={{0,10},{0,0},{-80,0}}, color={0,0,255}));
  connect(star2.pin_n, load_n)
    annotation (Line(points={{0,-60},{0,-84}}, color={0,0,255}));
  connect(star1.pin_n, load_p)
    annotation (Line(points={{0,60},{0,84}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-80},
            {80,80}}), graphics={
        Polygon(
          points={{0,20},{2,16},{20,-20},{-22,-20},{0,20}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-28,26},{28,20}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Line(
          points={{0,68},{0,26}},
          color={0,0,0}),
        Line(
          points={{0,-66},{0,-20}},
          color={0,0,0}),
        Rectangle(extent={{-80,80},{80,-80}}, lineColor={28,108,200})}),
                          Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-80,-80},{80,80}})));
end ThreePhaseRectifier;
