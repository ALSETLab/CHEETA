within CHEETA.Aircraft.Electrical.HTS;
model HTS_QuasiStationary "HTS line using Stekly equations"
  parameter Modelica.SIunits.Length l "Length of wire";
  parameter Modelica.SIunits.ElectricFieldStrength E_0 = 1e-4 "Reference electric field";
  parameter Real n = 2 "Intrinstic value of the superconductor";
  parameter Real I_c0 = 1 "Reference corner current";
  parameter Modelica.SIunits.Area A = 1 "Area";
  parameter Modelica.SIunits.Area A_cu = 1 "Area of copper in wire";
  parameter Modelica.SIunits.Current I_crit "Critical current";
  parameter Modelica.SIunits.Temp_K T_c = 92 "Critical temperature";
  //Losses
  parameter Modelica.SIunits.Resistance R_L "Resistance of the brass connectors";
  parameter Modelica.SIunits.Power G_d "Extra heat generation due to fault";
  //Characteristics of the line
  parameter Modelica.SIunits.Radius a = 2e-6
                                            "Inner radius of co-axial cable";
  parameter Modelica.SIunits.Radius b = 4e-6
                                            "Outer radius of co-axial cable";

  parameter Modelica.SIunits.Permeability mu_r = 1;
  parameter Modelica.SIunits.Permittivity epsilon_r = 1;
  parameter Modelica.SIunits.Length P = 0.1035 "Perimeter of line";
  parameter Modelica.SIunits.Frequency f = 60 "Frequency of AC system";
  //Constants
  Real pi= Modelica.Constants.pi;
  Modelica.SIunits.PermeabilityOfVacuum mu_0 = 4*pi*10e-7;
  Modelica.SIunits.PermittivityOfVacuum epsilon_0 = 8.854e-12;
  Modelica.SIunits.Permeability mu;
  Modelica.SIunits.Permittivity epsilon;
  Modelica.SIunits.Resistivity omega = f*2*pi;
  Modelica.SIunits.Resistivity delta = 30;

  //Line heat transfer characeteristics
  Real h "Heat transfer coefficient of surfaces";
  Modelica.SIunits.Temp_K dT "Change in temperature";
  Modelica.SIunits.Current I_c "corner current";
  Modelica.SIunits.ElectricFieldStrength E "Electric field";
  Modelica.SIunits.Power Q;
  Modelica.SIunits.Power G;
  //Resistances, inductances, and currents
  Modelica.SIunits.Resistance R_pi;
  Modelica.SIunits.Resistance R_ac;
  Modelica.SIunits.Inductance L_pi;
  Modelica.SIunits.Capacitance C_pi;

  Modelica.SIunits.Resistivity rho;

  Real x(start = 1e-5);
  Real y(start = 0.07);

  Modelica.Blocks.Sources.RealExpression realExpression(y=R_pi)
    annotation (Placement(transformation(extent={{-64,-34},{-52,-26}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=C_pi)
    annotation (Placement(transformation(extent={{-38,-22},{-28,-14}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=L_pi)
    annotation (Placement(transformation(extent={{-54,30},{-44,38}})));

  Modelica.Blocks.Sources.RealExpression realExpression3(y=R_ac)
    annotation (Placement(transformation(extent={{28,-22},{16,-14}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
    annotation (Placement(transformation(extent={{-8,-50},{12,-30}})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Basic.VariableInductor
    inductor annotation (Placement(transformation(extent={{-40,14},{-28,26}})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Basic.VariableInductor
    inductor1 annotation (Placement(transformation(extent={{-6,14},{6,26}})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Resistor resistor_L(
      R_ref=R_L)
    annotation (Placement(transformation(extent={{-66,-6},{-54,6}})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.PositivePin pin_p
    "Positive quasi-static single-phase pin"
    annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Resistor resistor_L1(
      R_ref=R_L) annotation (Placement(transformation(extent={{52,-6},{64,6}})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.NegativePin pin_n
    "Negative quasi-static single-phase pin"
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Basic.VariableResistor
    resistor annotation (Placement(transformation(extent={{-40,6},{-28,-6}})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Basic.VariableResistor
    resistor1 annotation (Placement(transformation(extent={{-6,6},{6,-6}})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Basic.VariableCapacitor
    capacitor annotation (Placement(transformation(
        extent={{-4,4},{4,-4}},
        rotation=270,
        origin={-14,-18})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Basic.VariableResistor
    resistor2 annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-2,-18})));
  Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground
    annotation (Placement(transformation(extent={{-18,-38},{-10,-30}})));
initial equation
  R_pi = l*E_0*DymolaModels.Functions.Math.divNoZero((pin_p.i/I_c)^n,pin_p.i);
equation
  mu = mu_0*mu_r;
  epsilon = epsilon_0*epsilon_r;

  I_c = I_c0 *(1-(port_a.T/T_c));
  E = E_0 *(pin_p.i/I_c)^n;
  rho = DymolaModels.Functions.Math.divNoZero(E,(I_c/A));
  Q = l*(mu_0 * h * I_c^2)/ (3*pi*b) * (I_c0/I_c)^3;
  x = DymolaModels.Functions.Math.divNoZero(port_a.T*(rho *I_c^2/(P*A_cu) + G_d),(2*h));
  y = (0.6953+0.001079*dT^4)*A;

 // if (x>=2) then
   // dT=200;
    //h = 100;
 // else
  //  dT = pre(x);
 //   h = pre(y);
 // end if;
 //when (x>=2) then
   //dT = 200;
   //h = 100;
  //elsewhen (x<2) then
   //dT = pre(x);
   //h = pre(y);
  //end when;

  dT = (if x>=2 then 200 else x);
  h = (if x>=2 then 100 else y);

   //  h = (0.2+0.013301*dT^2)*A;//(0.6953+0.001079*dT^4)*A;
  if (pin_p.i>I_crit) then
    G = (rho * I_c^2 * 10^3 / A_cu*P) + G_d*A_cu;
  else
    G = 0;
  end if;

  port_a.Q_flow = -h*dT;

  R_pi = l*E_0*DymolaModels.Functions.Math.divNoZero((pin_p.i/I_c)^n,pin_p.i);
  L_pi = l*mu/(2*pi) * log(b/a);
  C_pi = l*2*pi*epsilon / (log(b/a));
  R_ac =DymolaModels.Functions.Math.divNoZero( tan(delta),omega)*C_pi;
  connect(inductor1.pin_p, inductor.pin_n)
    annotation (Line(points={{-6,20},{-28,20}}, color={85,170,255}));
  connect(realExpression2.y, inductor.L) annotation (Line(points={{-43.5,34},{-34,
          34},{-34,27.2}}, color={0,0,127}));
  connect(inductor1.L, realExpression2.y)
    annotation (Line(points={{0,27.2},{0,34},{-43.5,34}}, color={0,0,127}));
  connect(resistor_L.pin_p, pin_p)
    annotation (Line(points={{-66,0},{-82,0}}, color={85,170,255}));
  connect(resistor_L1.pin_n, pin_n)
    annotation (Line(points={{64,0},{80,0}}, color={85,170,255}));
  connect(resistor_L.pin_n, resistor.pin_p)
    annotation (Line(points={{-54,0},{-40,0}}, color={85,170,255}));
  connect(resistor.pin_n, resistor1.pin_p)
    annotation (Line(points={{-28,0},{-6,0}}, color={85,170,255}));
  connect(inductor.pin_p, resistor.pin_p) annotation (Line(points={{-40,20},{-50,
          20},{-50,0},{-40,0}}, color={85,170,255}));
  connect(resistor.pin_n, inductor.pin_n) annotation (Line(points={{-28,0},{-20,
          0},{-20,20},{-28,20}}, color={85,170,255}));
  connect(inductor1.pin_n, resistor_L1.pin_p) annotation (Line(points={{6,20},{20,
          20},{20,0},{52,0}}, color={85,170,255}));
  connect(resistor1.pin_n, resistor_L1.pin_p)
    annotation (Line(points={{6,0},{52,0}}, color={85,170,255}));
  connect(resistor.R_ref, realExpression.y) annotation (Line(points={{-34,-7.2},
          {-34,-12},{-48,-12},{-48,-30},{-51.4,-30}}, color={0,0,127}));
  connect(resistor1.R_ref, realExpression.y) annotation (Line(points={{0,-7.2},{
          0,-12},{-48,-12},{-48,-30},{-51.4,-30}}, color={0,0,127}));
  connect(realExpression1.y, capacitor.C)
    annotation (Line(points={{-27.5,-18},{-18.8,-18}}, color={0,0,127}));
  connect(realExpression3.y, resistor2.R_ref)
    annotation (Line(points={{15.4,-18},{2.8,-18}}, color={0,0,127}));
  connect(capacitor.pin_p, resistor1.pin_p)
    annotation (Line(points={{-14,-14},{-14,0},{-6,0}}, color={85,170,255}));
  connect(resistor2.pin_p, resistor1.pin_p) annotation (Line(points={{-2,-14},{-2,
          -8},{-14,-8},{-14,0},{-6,0}}, color={85,170,255}));
  connect(capacitor.pin_n, resistor2.pin_n) annotation (Line(points={{-14,-22},{
          -14,-26},{-2,-26},{-2,-22}}, color={85,170,255}));
  connect(capacitor.pin_n, ground.pin)
    annotation (Line(points={{-14,-22},{-14,-30}}, color={85,170,255}));
  connect(port_a, port_a) annotation (Line(points={{2,-40},{-4,-40},{-4,-40},{2,
          -40}}, color={191,0,0}));
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
end HTS_QuasiStationary;
