within CHEETA.Aircraft.Electrical.HTS.Stekly;
model Stekly_ColdEndCooling "HTS line using Stekly equations"
  parameter Modelica.SIunits.Length l "Length of wire";
  parameter Modelica.SIunits.ElectricFieldStrength E_0 = 1e-4 "Reference electric field";
  parameter Real n = 20 "Intrinstic value of the superconductor";
  parameter Real I_c0 = 1 "Reference corner current";
  parameter Modelica.SIunits.Area A = 1 "Area";
  parameter Modelica.SIunits.Area A_cu = 1 "Area";
  parameter Modelica.SIunits.Length P = 0.1035 "Perimeter of cable";
  parameter Modelica.SIunits.Power G_d "Extra heat generation";
  parameter Modelica.SIunits.Radius R_0 "Wire radius";
  parameter Modelica.SIunits.Thickness h = 1 "Heat transfer coefficient of surfaces";
  parameter Modelica.SIunits.Resistance R "Line resistance";
  parameter Modelica.SIunits.Current I_crit "Critical current";
  parameter Modelica.SIunits.Temp_K T_c = 92 "Critical temperature";
  parameter Real kappa;

  parameter Modelica.SIunits.Heat h_fg = 0.44936 "Latent heat of liquid hydrogen";
  parameter Modelica.SIunits.Density rho_l = 70.85 "Density of liquid";
  parameter Modelica.SIunits.Density rho_v "Density of vapor";
  parameter Modelica.SIunits.Acceleration g = 9.81 "Gravity";
  parameter Modelica.SIunits.SurfaceTension sigma;

  Modelica.SIunits.Current I_c "corner current";
  Modelica.SIunits.ElectricFieldStrength E "Electric field";
  Modelica.SIunits.Power G;
  Real pi = Modelica.Constants.pi;
  Modelica.SIunits.PermeabilityOfVacuum mu_0 = 4*pi*10e-7;
  Modelica.SIunits.Power Q;
  Modelica.SIunits.Power Q_ce;
  Modelica.SIunits.Voltage v_n;
  Real dT  "Change in temperature";
  Modelica.SIunits.Resistivity rho "Resitivity of line";
  Real x;
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p             annotation (Placement(
        transformation(extent={{-100,-10},{-80,10}}),iconTransformation(extent={{-100,
            -10},{-80,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation (Placement(
        transformation(extent={{80,-10},{100,10}}),iconTransformation(extent={{80,-10},
            {100,10}})));

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
    annotation (Placement(transformation(extent={{-10,-50},{10,-30}})));

initial equation

equation
  //h = (k_l*rho_l*c_pl/tau)^(1/2);
  I_c = -I_c0 *(1-port_a.T)/T_c;
  E = E_0 *((pin_p.i/I_c))^n;
  rho = DymolaModels.Functions.Math.divNoZero(E,(I_c/A));
  if noEvent(pin_p.i>I_crit) then
    G = (rho * I_c^2 * 10^3 / A_cu*P) + G_d*A_cu;
  else
    G = 0;
  end if;
  Q = l*(mu_0 * h * I_c^2)/ (3*pi*R_0) * (I_c0/I_c)^3;
  x = rho *h *I_c^2/(P*A_cu);
  Q_ce = port_a.T*sqrt(2*kappa*A_cu*P*h);
  dT = port_a.T*(rho *I_c^2/(P*A_cu) + G_d)/(2*h);
  port_a.Q_flow = (0.6953+0.001079*dT^4)*A*dT;//0.16*h_fg*rho_v*(g*sigma*(rho_l-rho_v)/rho_v^2);
  v_n =  DymolaModels.Functions.Math.divNoZero(Q+G,pin_p.i);
  pin_n.i = pin_p.i;
  pin_n.v = pin_p.v-E*l;

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
end Stekly_ColdEndCooling;
