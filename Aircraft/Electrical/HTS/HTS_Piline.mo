within CHEETA.Aircraft.Electrical.HTS;
model HTS_Piline "HTS line using Stekly equations"
  parameter Modelica.SIunits.Length l "Length of wire";
  parameter Modelica.SIunits.ElectricFieldStrength E_0 = 1e-4 "Reference electric field";
  parameter Real n = 2 "Intrinstic value of the superconductor";
  parameter Real I_c0 = 1 "Reference corner current";
  parameter Real dT = 2 "Change in temperature";
  parameter Modelica.SIunits.Area A = 1 "Area";
  parameter Modelica.SIunits.Resistance R_L "Resistance of the brass connectors";
  //Characteristics of the line
  parameter Modelica.SIunits.Radius a = 2e-6
                                            "Inner radius of co-axial cable";
  parameter Modelica.SIunits.Radius b = 4e-6
                                            "Outer radius of co-axial cable";

  parameter Modelica.SIunits.Permeability mu_r = 1;
  parameter Modelica.SIunits.Permittivity epsilon_r = 1;

  //Constants
  Real pi= Modelica.Constants.pi;
  Modelica.SIunits.PermeabilityOfVacuum mu_0 = 4*pi*10e-7;

  Modelica.SIunits.PermittivityOfVacuum epsilon_0 = 8.854e-12;
  Modelica.SIunits.Permeability mu;
  Modelica.SIunits.Permittivity epsilon;
  //Resistances, inductances, and currents
  Modelica.SIunits.Current I_c "corner current";
  Modelica.SIunits.ElectricFieldStrength E "Electric field";
  Modelica.SIunits.Resistance R_pi;
  Modelica.SIunits.Inductance L_pi;
  Modelica.SIunits.Capacitance C_pi;


  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p             annotation (Placement(
        transformation(extent={{-100,-10},{-80,10}}),iconTransformation(extent={{-100,
            -10},{-80,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation (Placement(
        transformation(extent={{80,-10},{100,10}}),iconTransformation(extent={{80,-10},
            {100,10}})));

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
    annotation (Placement(transformation(extent={{-10,-50},{10,-30}})));

  Modelica.Electrical.Analog.Basic.Resistor resistor(R=R_L)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=R_L)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Modelica.Electrical.Analog.Basic.VariableInductor
                                            inductor(
    Lmin=0,
    IC=IC,
    UIC=UIC)
    annotation (Placement(transformation(extent={{-28,6},{-8,26}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor2
    annotation (Placement(transformation(extent={{-30,10},{-10,-10}})));
  Modelica.Electrical.Analog.Basic.VariableInductor
                                            inductor1(
    Lmin=0,
    IC=IC,
    UIC=UIC)
    annotation (Placement(transformation(extent={{8,6},{28,26}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor3
    annotation (Placement(transformation(extent={{8,10},{28,-10}})));
  Modelica.Electrical.Analog.Basic.VariableCapacitor
                                             capacitor(
    Cmin=0,
    IC=IC,
    UIC=UIC)                                           annotation (Placement(
        transformation(
        extent={{-6,6},{6,-6}},
        rotation=270,
        origin={4,-14})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{0,-30},{8,-22}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=R_pi/2)
    annotation (Placement(transformation(extent={{-52,-34},{-40,-26}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=C_pi)
    annotation (Placement(transformation(extent={{-18,-28},{-8,-20}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=L_pi/2)
    annotation (Placement(transformation(extent={{-54,30},{-44,38}})));
  parameter Boolean UIC=false "Use starting condition";
  parameter Modelica.SIunits.Voltage IC=0 "Initial Value of capacitor";
initial equation
  R_pi = l*E_0*(pin_p.i/I_c)^n;
equation
  mu = mu_0*mu_r;
  epsilon = epsilon_0*epsilon_r;
  port_a.Q_flow = (0.6953+0.001079*dT^4)*A;
  I_c = I_c0 *port_a.T;
  E = E_0 *(pin_p.i/I_c)^n;
  R_pi = l*E_0*(pin_p.i/I_c)^n/pin_p.i;
  L_pi = l*mu/(2*pi) * log(b/a);
  C_pi = l*2*pi*epsilon / (log(b/a));


  connect(pin_p, resistor.p)
    annotation (Line(points={{-90,0},{-60,0}}, color={0,0,255}));
  connect(resistor.n, inductor.p) annotation (Line(points={{-40,0},{-36,0},{-36,
          16},{-28,16}}, color={0,0,255}));
  connect(resistor.n, resistor2.p)
    annotation (Line(points={{-40,0},{-30,0}}, color={0,0,255}));
  connect(inductor.n, inductor1.p)
    annotation (Line(points={{-8,16},{8,16}}, color={0,0,255}));
  connect(inductor1.n, resistor3.n)
    annotation (Line(points={{28,16},{28,0}}, color={0,0,255}));
  connect(resistor1.p, resistor3.n)
    annotation (Line(points={{40,0},{28,0}}, color={0,0,255}));
  connect(capacitor.n, ground.p)
    annotation (Line(points={{4,-20},{4,-22}},   color={0,0,255}));
  connect(resistor2.R, realExpression.y) annotation (Line(points={{-20,-12},{-20,
          -30},{-39.4,-30}}, color={0,0,127}));
  connect(resistor3.R, realExpression.y)
    annotation (Line(points={{18,-12},{18,-30},{-39.4,-30}}, color={0,0,127}));
  connect(resistor3.p, resistor2.n)
    annotation (Line(points={{8,0},{-10,0}}, color={0,0,255}));
  connect(capacitor.p, inductor1.p)
    annotation (Line(points={{4,-8},{4,16},{8,16}}, color={0,0,255}));
  connect(capacitor.p, resistor2.n)
    annotation (Line(points={{4,-8},{4,0},{-10,0}}, color={0,0,255}));
  connect(capacitor.C, realExpression1.y) annotation (Line(points={{-3.2,-14},{-6,
          -14},{-6,-24},{-7.5,-24}}, color={0,0,127}));
  connect(inductor.L, realExpression2.y)
    annotation (Line(points={{-18,28},{-18,34},{-43.5,34}}, color={0,0,127}));
  connect(inductor1.L, realExpression2.y)
    annotation (Line(points={{18,28},{18,34},{-43.5,34}}, color={0,0,127}));
  connect(resistor1.n, pin_n)
    annotation (Line(points={{60,0},{90,0}}, color={0,0,255}));
   annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
           -40},{80,40}}), graphics={
                  Rectangle(
          extent={{-80,40},{80,-40}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),Rectangle(
          extent={{-60,20},{60,-20}},
          lineColor={0,0,255},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),Text(
          extent={{-60,20},{60,-20}},
          lineColor={255,255,0},
          textString="%name")}),                                  Diagram(
         coordinateSystem(preserveAspectRatio=false, extent={{-80,-40},{80,40}})));
end HTS_Piline;
