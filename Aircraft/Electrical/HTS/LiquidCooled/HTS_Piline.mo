within CHEETA.Aircraft.Electrical.HTS.LiquidCooled;
model HTS_Piline "HTS line using Stekly equations"
  parameter Modelica.Units.SI.Length l "Length of wire";
  parameter Modelica.Units.SI.ElectricFieldStrength E_0=1e-4
    "Reference electric field";
  parameter Real n = 2 "Intrinstic value of the superconductor";
  parameter Real I_c0 = 1 "Reference corner current";
  parameter Modelica.Units.SI.Area A=1 "Area";
  parameter Modelica.Units.SI.Area A_cu=1 "Area of copper in wire";
  parameter Modelica.Units.SI.Current I_crit "Critical current";
  parameter Modelica.Units.SI.Temperature T_c=92 "Critical temperature";
  //Losses
  parameter Modelica.Units.SI.Resistance R_L
    "Resistance of the brass connectors";
  parameter Modelica.Units.SI.Power G_d "Extra heat generation due to fault";
  //Characteristics of the line
  parameter Modelica.Units.SI.Radius a=2e-6 "Inner radius of co-axial cable";
  parameter Modelica.Units.SI.Radius b=4e-6 "Outer radius of co-axial cable";

  parameter Modelica.Units.SI.Permeability mu_r=1;
  parameter Modelica.Units.SI.Permittivity epsilon_r=1;
  parameter Modelica.Units.SI.Length P=0.1035 "Perimeter of line";
  parameter Modelica.Units.SI.Frequency f=60 "Frequency of AC system";
  //Constants
  Real pi= Modelica.Constants.pi;
  Modelica.Units.SI.PermeabilityOfVacuum mu_0=4*pi*10e-7;
  Modelica.Units.SI.PermittivityOfVacuum epsilon_0=8.854e-12;
  Modelica.Units.SI.Permeability mu;
  Modelica.Units.SI.Permittivity epsilon;
  Modelica.Units.SI.Resistivity omega=f*2*pi;
  Modelica.Units.SI.Resistivity delta=30;

  //Line heat transfer characeteristics
  Real h "Heat transfer coefficient of surfaces";
  Modelica.Units.SI.Temperature dT "Change in temperature";
  Modelica.Units.SI.Current I_c "corner current";
  Modelica.Units.SI.ElectricFieldStrength E "Electric field";
  Modelica.Units.SI.Power Q;
  Modelica.Units.SI.Power G;
  //Resistances, inductances, and currents
  Modelica.Units.SI.Resistance R_pi;
  Modelica.Units.SI.Resistance R_ac;
  Modelica.Units.SI.Inductance L_pi;
  Modelica.Units.SI.Capacitance C_pi;

  Modelica.Units.SI.Resistivity rho;

  Real x(start=0);
  //Real y(start = 0.07);
  Real z=1;

  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p             annotation (Placement(
        transformation(extent={{-100,-10},{-80,10}}),iconTransformation(extent={{-100,
            -10},{-80,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation (Placement(
        transformation(extent={{80,-10},{100,10}}),iconTransformation(extent={{80,-10},
            {100,10}})));

  Modelica.Electrical.Analog.Basic.Resistor resistor(R=R_L)
    annotation (Placement(transformation(extent={{-70,-6},{-58,6}})));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R=R_L)
    annotation (Placement(transformation(extent={{60,-6},{72,6}})));
  Modelica.Electrical.Analog.Basic.VariableInductor
                                            inductor
    annotation (Placement(transformation(extent={{-46,10},{-34,22}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor2
    annotation (Placement(transformation(extent={{-46,6},{-34,-6}})));
  Modelica.Electrical.Analog.Basic.VariableInductor
                                            inductor1(
    Lmin=Modelica.Constants.eps)
    annotation (Placement(transformation(extent={{-6,10},{6,22}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor3
    annotation (Placement(transformation(extent={{-6,6},{6,-6}})));
  Modelica.Electrical.Analog.Basic.VariableCapacitor
                                             capacitor(
    Cmin=Modelica.Constants.eps)                                            annotation (Placement(
        transformation(
        extent={{-6,6},{6,-6}},
        rotation=270,
        origin={-18,-14})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-22,-30},{-14,-22}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=R_pi)
    annotation (Placement(transformation(extent={{-64,-34},{-52,-26}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=C_pi)
    annotation (Placement(transformation(extent={{-38,-18},{-28,-10}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=L_pi)
    annotation (Placement(transformation(extent={{-54,30},{-44,38}})));

  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor4
    annotation (Placement(transformation(extent={{6,6},{-6,-6}},
        rotation=90,
        origin={-10,-14})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=R_ac)
    annotation (Placement(transformation(extent={{28,-18},{16,-10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
    annotation (Placement(transformation(extent={{-8,-50},{12,-30}})));
initial equation
  R_pi = l*E_0*DymolaModels.Functions.Math.divNoZero((pin_p.i/I_c)^n,pin_p.i);
equation
  mu = mu_0*mu_r;
  epsilon = epsilon_0*epsilon_r;

  I_c = I_c0 *(1-(port_a.T/T_c));
  E = E_0 *(pin_p.i/I_c)^n;
  rho = DymolaModels.Functions.Math.divNoZero(E,(I_c/A));
  L_pi = l*mu/(2*pi) * log(b/a);
  C_pi = l*2*pi*epsilon / (log(b/a));
  R_pi = l*E_0*DymolaModels.Functions.Math.divNoZero((pin_p.i/I_c)^n,pin_p.i);
  R_ac =DymolaModels.Functions.Math.divNoZero( tan(delta),omega)*C_pi;

  dT = DymolaModels.Functions.Math.divNoZero(port_a.T*(rho *I_c^2/(P*A_cu) + G_d),(h));
  x = 0;
  h = smooth(10,noEvent(if dT>=2 then 10*DymolaModels.Functions.Math.divNoZero(-5.787-0.155*dT,1-0.546*dT) else 1000*(0.6953+0.001079*dT^4)));

  port_a.Q_flow = -h*dT*A+G;
  Q = l*(mu_0 * h * I_c^2)/ (3*pi*b) * (I_c0/I_c)^3;

  if noEvent(pin_p.i>I_crit) then
    G = (rho * I_c^2 * 10^3 / A_cu*P) + G_d*A_cu;
  else
    G = 0;
  end if;

  connect(pin_p, resistor.p)
    annotation (Line(points={{-90,0},{-70,0}}, color={0,0,255}));
  connect(resistor.n, resistor2.p)
    annotation (Line(points={{-58,0},{-46,0}}, color={0,0,255}));
  connect(inductor1.n, resistor3.n)
    annotation (Line(points={{6,16},{14,16},{14,0},{6,0}},
                                              color={0,0,255}));
  connect(capacitor.n, ground.p)
    annotation (Line(points={{-18,-20},{-18,-22}},
                                                 color={0,0,255}));
  connect(resistor2.R, realExpression.y) annotation (Line(points={{-40,-7.2},{-40,
          -30},{-51.4,-30}}, color={0,0,127}));
  connect(resistor3.R, realExpression.y)
    annotation (Line(points={{0,-7.2},{0,-30},{-51.4,-30}},  color={0,0,127}));
  connect(resistor3.p, resistor2.n)
    annotation (Line(points={{-6,0},{-34,0}},color={0,0,255}));
  connect(capacitor.p, inductor1.p)
    annotation (Line(points={{-18,-8},{-18,16},{-6,16}},
                                                    color={0,0,255}));
  connect(capacitor.p, resistor2.n)
    annotation (Line(points={{-18,-8},{-18,0},{-34,0}},
                                                    color={0,0,255}));
  connect(capacitor.C, realExpression1.y) annotation (Line(points={{-25.2,-14},{
          -27.5,-14}},               color={0,0,127}));
  connect(inductor.L, realExpression2.y)
    annotation (Line(points={{-40,23.2},{-40,34},{-43.5,34}},
                                                            color={0,0,127}));
  connect(inductor1.L, realExpression2.y)
    annotation (Line(points={{0,23.2},{0,34},{-43.5,34}}, color={0,0,127}));
  connect(resistor1.n, pin_n)
    annotation (Line(points={{72,0},{90,0}}, color={0,0,255}));
  connect(resistor1.p, resistor3.n)
    annotation (Line(points={{60,0},{6,0}}, color={0,0,255}));
  connect(resistor4.p, inductor1.p) annotation (Line(points={{-10,-8},{-10,-4},{
          -18,-4},{-18,16},{-6,16}}, color={0,0,255}));
  connect(resistor4.n, ground.p)
    annotation (Line(points={{-10,-20},{-10,-22},{-18,-22}}, color={0,0,255}));
  connect(realExpression3.y, resistor4.R)
    annotation (Line(points={{15.4,-14},{-2.8,-14}}, color={0,0,127}));
  connect(inductor.p, resistor2.p) annotation (Line(points={{-46,16},{-48,16},{
          -48,0},{-46,0}}, color={0,0,255}));
  connect(inductor.n, inductor1.p)
    annotation (Line(points={{-34,16},{-6,16}}, color={0,0,255}));
   annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-40},
            {80,40}}),     graphics={
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
         coordinateSystem(preserveAspectRatio=false, extent={{-80,-40},{80,40}})),
    Documentation(info="<html>
<p>This transmission line model is a pi-line model. The capactiance, resistance, and inductance of the line depend on the physical dimensions and current of the line. The resistance in parallel to the capacitor is used as a modeled loss.</p>
</html>"),              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-40},
            {80,40}}),     graphics={
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
         coordinateSystem(preserveAspectRatio=false, extent={{-80,-40},{80,40}})),
    Documentation(info="<html>
<p>This transmission line model is a pi-line model. The capactiance, resistance, and inductance of the line depend on the physical dimensions and current of the line. The resistance in parallel to the capacitor is used as a modeled loss.</p>
</html>"),
    experiment(__Dymola_NumberOfIntervals=1000, __Dymola_Algorithm="Dassl"));
end HTS_Piline;
