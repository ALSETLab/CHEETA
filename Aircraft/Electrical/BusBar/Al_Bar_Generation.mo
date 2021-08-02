within CHEETA.Aircraft.Electrical.BusBar;
model Al_Bar_Generation
  Cu_CurrentLead cu_CurrentLead(
    l=l,
    k_0=k_0,
    h_L=h_L,
    c_p0=c_p0,
    I_0=I_0,
    A=A) annotation (Placement(transformation(
        extent={{-4,-6},{4,6}},
        rotation=90,
        origin={-22,82})));
  Modelica.Electrical.Analog.Interfaces.PositivePin FuelCell1
    "Positive electrical pin" annotation (Placement(transformation(extent={{-60,
            72},{-40,92}}), iconTransformation(extent={{-60,72},{-40,92}})));
  Cu_CurrentLead cu_CurrentLead1(
    l=l,
    k_0=k_0,
    h_L=h_L,
    c_p0=c_p0,
    I_0=I_0,
    A=A) annotation (Placement(transformation(
        extent={{-4,-6},{4,6}},
        rotation=90,
        origin={-20,34})));
  Modelica.Electrical.Analog.Interfaces.PositivePin FuelCell2
    "Positive electrical pin" annotation (Placement(transformation(extent={{-60,
            24},{-40,44}}), iconTransformation(extent={{-60,24},{-40,44}})));
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
  Modelica.Electrical.Analog.Interfaces.PositivePin HTS
    "Positive electrical pin" annotation (Placement(transformation(extent={{60,
            -10},{40,10}}), iconTransformation(extent={{60,-10},{40,10}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=1e-10) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,62})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=1e-10) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,12})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a1 annotation (
      Placement(transformation(extent={{-50,-10},{-30,10}}), iconTransformation(
          extent={{-50,-10},{-30,10}})));
  parameter Real l=0.38 "Lead length";
  parameter Real k_0=600 "Thermal conductivity";
  parameter Real h_L=20.7e3 "Latent heat of liquid media";
  parameter Real c_p0=5.26e3 "Specific heat of media";
  parameter Real I_0=8000 "Carrying current";
  parameter Real A=1 "Current lead area";
  Cu_CurrentLead cu_CurrentLead3(
    l=l,
    k_0=k_0,
    h_L=h_L,
    c_p0=c_p0,
    I_0=I_0,
    A=A) annotation (Placement(transformation(
        extent={{-4,-6},{4,6}},
        rotation=90,
        origin={-20,-40})));
  Modelica.Electrical.Analog.Interfaces.PositivePin FuelCell3
    "Positive electrical pin" annotation (Placement(transformation(extent={{-60,
            -50},{-40,-30}}), iconTransformation(extent={{-60,-50},{-40,-30}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor2(R=1e-10) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-26})));
  Cu_CurrentLead cu_CurrentLead4(
    l=l,
    k_0=k_0,
    h_L=h_L,
    c_p0=c_p0,
    I_0=I_0,
    A=A) annotation (Placement(transformation(
        extent={{-4,-6},{4,6}},
        rotation=90,
        origin={-20,-78})));
  Modelica.Electrical.Analog.Basic.Resistor resistor3(R=1e-10) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-64})));
  Modelica.Electrical.Analog.Interfaces.PositivePin FuelCell4
    "Positive electrical pin" annotation (Placement(transformation(extent={{-60,
            -92},{-40,-72}}), iconTransformation(extent={{-60,-92},{-40,-72}})));
equation
  connect(cu_CurrentLead.p1, FuelCell1)
    annotation (Line(points={{-29,82},{-50,82}}, color={0,0,255}));
  connect(cu_CurrentLead1.p1, FuelCell2)
    annotation (Line(points={{-27,34},{-50,34}}, color={0,0,255}));
  connect(cu_CurrentLead2.p1, HTS)
    annotation (Line(points={{25,-4.44089e-16},{50,0}}, color={0,0,255}));
  connect(cu_CurrentLead.n1, resistor.p) annotation (Line(points={{-15,82},{
          1.77636e-15,82},{1.77636e-15,72}}, color={0,0,255}));
  connect(cu_CurrentLead2.port_a, port_a1) annotation (Line(points={{18,-4},{18,
          -6},{-22,-6},{-22,0},{-40,0}}, color={191,0,0}));
  connect(cu_CurrentLead3.p1, FuelCell3)
    annotation (Line(points={{-27,-40},{-50,-40}}, color={0,0,255}));
  connect(resistor2.p, resistor1.n) annotation (Line(points={{1.77636e-15,-16},
          {-1.77636e-15,2}}, color={0,0,255}));
  connect(resistor2.n, cu_CurrentLead3.n1) annotation (Line(points={{
          -1.77636e-15,-36},{-1.77636e-15,-40},{-13,-40}}, color={0,0,255}));
  connect(cu_CurrentLead3.port_a, port_a1) annotation (Line(points={{-20,-44},{
          -22,-44},{-22,0},{-40,0}}, color={191,0,0}));
  connect(resistor.n, resistor1.p) annotation (Line(points={{-1.77636e-15,52},{
          1.77636e-15,22}}, color={0,0,255}));
  connect(cu_CurrentLead1.n1, resistor1.p) annotation (Line(points={{-13,34},{
          -1.77636e-15,34},{1.77636e-15,22}}, color={0,0,255}));
  connect(cu_CurrentLead4.p1, FuelCell4) annotation (Line(points={{-27,-78},{
          -38,-78},{-38,-82},{-50,-82}}, color={0,0,255}));
  connect(resistor3.n, cu_CurrentLead4.n1)
    annotation (Line(points={{0,-74},{0,-78},{-13,-78}}, color={0,0,255}));
  connect(cu_CurrentLead4.port_a, port_a1) annotation (Line(points={{-20,-82},{
          -22,-82},{-22,0},{-40,0}}, color={191,0,0}));
  connect(cu_CurrentLead1.port_a, port_a1) annotation (Line(points={{-20,30},{
          -20,-6},{-22,-6},{-22,0},{-40,0}}, color={191,0,0}));
  connect(cu_CurrentLead.port_a, port_a1) annotation (Line(points={{-22,78},{
          -20,78},{-20,-6},{-22,-6},{-22,0},{-40,0}}, color={191,0,0}));
  connect(resistor3.p, resistor2.n) annotation (Line(points={{1.77636e-15,-54},
          {-1.77636e-15,-36}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,
            -100},{40,100}}), graphics={Rectangle(extent={{-40,100},{40,-100}},
            lineColor={28,108,200})}), Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-40,-100},{40,100}})));
end Al_Bar_Generation;
