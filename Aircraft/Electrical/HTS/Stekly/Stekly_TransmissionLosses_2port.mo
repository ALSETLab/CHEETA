within CHEETA.Aircraft.Electrical.HTS.Stekly;
model Stekly_TransmissionLosses_2port
  "HTS line using Stekly equations"
  parameter Modelica.SIunits.Length l "Length of wire";
  parameter Modelica.SIunits.ElectricFieldStrength E_0 = 1e-4 "Reference electric field";
  parameter Real n = 2 "Intrinstic value of the superconductor";
  parameter Real I_c0 = 1 "Reference corner current";
  parameter Modelica.SIunits.Area A = 1 "Area";
  parameter Modelica.SIunits.Area A_cu = 1 "Area";
  parameter Modelica.SIunits.Length P = 0.1035 "Perimeter of cable";
  parameter Modelica.SIunits.Resistivity rho = 2.1e-9 "Resitivity of copper";
  parameter Modelica.SIunits.Power G_d "Extra heat generation";
  parameter Modelica.SIunits.Radius R_0 "Wire radius";
  parameter Real h = 1 "Heat transfer coefficient of surfaces";
  parameter Modelica.SIunits.Resistance R "Brass connector resistance";
  parameter Modelica.SIunits.Current I_crit "Critical current";

  Modelica.SIunits.Current I_c "corner current";
  Modelica.SIunits.ElectricFieldStrength E "Electric field";
  Modelica.SIunits.Power G;
  Real pi = Modelica.Constants.pi;
  Modelica.SIunits.PermeabilityOfVacuum mu_0 = 4*pi*10e-7;
  Modelica.SIunits.Power Q;
  Modelica.SIunits.Voltage v_n;
  Modelica.SIunits.Voltage v_in;
  Real dT "Change in temperature";

  Modelica.Electrical.Analog.Interfaces.PositivePin p_in annotation (Placement(
        transformation(extent={{-100,10},{-80,30}}), iconTransformation(extent={
            {-100,10},{-80,30}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n_out annotation (Placement(
        transformation(extent={{80,-30},{100,-10}}), iconTransformation(extent={
            {80,-30},{100,-10}})));

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
    annotation (Placement(transformation(extent={{-10,-50},{10,-30}})));

  Modelica.Electrical.Analog.Interfaces.PositivePin p_out annotation (Placement(
        transformation(extent={{74,10},{94,30}}), iconTransformation(extent={{80,
            10},{100,30}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n_in annotation (Placement(
        transformation(extent={{-100,-30},{-80,-10}}), iconTransformation(
          extent={{-100,-30},{-80,-10}})));
initial equation

equation
  v_in =p_in.v - n_in.v;

  I_c = I_c0 *port_a.T;
  E =E_0*((p_in.i/I_c))^n;
  if noEvent(p_in.i > I_crit) then
    G = (rho * I_c^2 * 10^3 / A_cu*P) + G_d*A_cu;
  else
    G = 0;
  end if;
  Q = l*(mu_0 * h * I_c^2)/ (3*pi*R_0) * (I_c0/I_c)^3;
  dT = (rho *I_c^2/(P*A_cu) + G_d)/h;
  port_a.Q_flow = (0.6953+0.001079*dT^4)*A;
  v_n =(Q + G)/(p_in.i);
  p_out.i = p_in.i;
  n_out.i = n_in.i;
  p_out.v = v_in - E*l- R*p_out.i;
  n_out.v = 0;

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
end Stekly_TransmissionLosses_2port;
