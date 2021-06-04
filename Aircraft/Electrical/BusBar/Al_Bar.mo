within CHEETA.Aircraft.Electrical.BusBar;
model Al_Bar
  Cu_CurrentLead cu_CurrentLead(
    l=l,
    k_0=k_0,
    h_L=h_L,
    c_p0=c_p0,
    I_0=I_0,
    A=A) annotation (Placement(transformation(
        extent={{-4,-6},{4,6}},
        rotation=90,
        origin={-22,40})));
  Modelica.Electrical.Analog.Interfaces.PositivePin FuelCell
    "Positive electrical pin" annotation (Placement(transformation(extent={{-60,
            30},{-40,50}}), iconTransformation(extent={{-60,30},{-40,50}})));
  Cu_CurrentLead cu_CurrentLead1(
    l=l,
    k_0=k_0,
    h_L=h_L,
    c_p0=c_p0,
    I_0=I_0,
    A=A) annotation (Placement(transformation(
        extent={{-4,-6},{4,6}},
        rotation=90,
        origin={-20,-40})));
  Modelica.Electrical.Analog.Interfaces.PositivePin Battery
    "Positive electrical pin" annotation (Placement(transformation(extent={{-60,
            -50},{-40,-30}}), iconTransformation(extent={{-60,-50},{-40,-30}})));
  Cu_CurrentLead cu_CurrentLead2(
    l=l,
    k_0=k_0,
    h_L=h_L,
    c_p0=c_p0,
    I_0=I_0,
    A=A) annotation (Placement(transformation(
        extent={{-4,6},{4,-6}},
        rotation=90,
        origin={18,0})));
  Modelica.Electrical.Analog.Interfaces.PositivePin Machine
    "Positive electrical pin" annotation (Placement(transformation(extent={{60,
            -10},{40,10}}), iconTransformation(extent={{60,-10},{40,10}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=1e-10) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,20})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=1e-10) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-16})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a1
    annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
  parameter Real l=0.38 "Lead length";
  parameter Real k_0=600 "Thermal conductivity";
  parameter Real h_L=20.7e3 "Latent heat of liquid media";
  parameter Real c_p0=5.26e3 "Specific heat of media";
  parameter Real I_0=8000 "Carrying current";
  parameter Real A=1 "Current lead area";
equation
  connect(cu_CurrentLead.p1, FuelCell)
    annotation (Line(points={{-29,40},{-50,40}}, color={0,0,255}));
  connect(cu_CurrentLead1.p1, Battery)
    annotation (Line(points={{-27,-40},{-50,-40}}, color={0,0,255}));
  connect(cu_CurrentLead2.p1, Machine)
    annotation (Line(points={{25,-4.44089e-16},{50,0}}, color={0,0,255}));
  connect(cu_CurrentLead.n1, resistor.p) annotation (Line(points={{-15,40},{
          1.77636e-15,40},{1.77636e-15,30}}, color={0,0,255}));
  connect(resistor.n, resistor1.p) annotation (Line(points={{-1.77636e-15,10},{
          1.77636e-15,-6}}, color={0,0,255}));
  connect(cu_CurrentLead2.n1, resistor1.p) annotation (Line(points={{11,
          4.44089e-16},{1.77636e-15,4.44089e-16},{1.77636e-15,-6}}, color={0,0,
          255}));
  connect(cu_CurrentLead1.n1, resistor1.n) annotation (Line(points={{-13,-40},{
          -1.77636e-15,-40},{-1.77636e-15,-26}}, color={0,0,255}));
  connect(cu_CurrentLead.port_a, port_a1)
    annotation (Line(points={{-22,36},{-22,0},{-44,0}}, color={191,0,0}));
  connect(cu_CurrentLead1.port_a, port_a1) annotation (Line(points={{-20,-44},{
          -22,-44},{-22,0},{-44,0}}, color={191,0,0}));
  connect(cu_CurrentLead2.port_a, port_a1) annotation (Line(points={{18,-4},{18,
          -6},{-22,-6},{-22,0},{-44,0}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,
            -100},{40,100}}), graphics={Rectangle(extent={{-40,100},{40,-100}},
            lineColor={28,108,200})}), Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-40,-100},{40,100}})));
end Al_Bar;
