within CHEETA.Aircraft.Electrical.HTS.Stekly;
model Stekly_TransmissionLosses "HTS line using Stekly equations"
  parameter Modelica.Units.SI.Length l "Length of wire";
  parameter Modelica.Units.SI.ElectricFieldStrength E_0=1e-4
    "Reference electric field";
  parameter Real n = 2 "Intrinstic value of the superconductor";
  parameter Real I_c0 = 1 "Reference corner current";
  parameter Modelica.Units.SI.Area A=1 "Area";
  parameter Modelica.Units.SI.Area A_cu=1 "Area";
  parameter Modelica.Units.SI.Length P=0.1035 "Perimeter of cable";
  parameter Modelica.Units.SI.Power G_d "Extra heat generation";
  parameter Modelica.Units.SI.Radius R_0 "Wire radius";
  parameter Modelica.Units.SI.Resistance R "Line resistance";
  parameter Modelica.Units.SI.Current I_crit "Critical current";
  parameter Modelica.Units.SI.Temperature T_c=92 "Critical temperature";

  Modelica.Units.SI.Thickness h "Heat transfer coefficient of surfaces";
  Modelica.Units.SI.Current I_c "corner current";
  Modelica.Units.SI.ElectricFieldStrength E "Electric field";
  Modelica.Units.SI.Power G;
  Real pi = Modelica.Constants.pi;
  Modelica.Units.SI.PermeabilityOfVacuum mu_0=4*pi*10e-7;
  Modelica.Units.SI.Power Q;
  Modelica.Units.SI.Voltage v_n;
  Real dT  "Change in temperature";
  Modelica.Units.SI.Resistivity rho "Resitivity of line";
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
  I_c = I_c0*port_a.T;//I_c0 *(1-port_a.T)/T_c;
  E = E_0 *DymolaModels.Functions.Math.divNoZero(pin_p.i,(I_c))^n;
  rho = DymolaModels.Functions.Math.divNoZero(E,(I_c/A));
  if noEvent(pin_p.i>I_crit) then
    G = (rho * I_c^2 * 10^3 / A_cu*P) + G_d*A_cu;
  else
    G = 0;
  end if;
  Q = l*(mu_0 * h * I_c^2)/ (3*pi*R_0) *DymolaModels.Functions.Math.divNoZero(I_c0,I_c)^3;
  x = DymolaModels.Functions.Math.divNoZero(port_a.T*(rho *I_c^2/(P*A_cu) + G_d),(2*h));
  if x>2 then
     dT = 2;
  else
     dT = x;
  end if;

  h = (0.6953+0.001079*dT^4)*A;
  port_a.Q_flow = dT*h;
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
end Stekly_TransmissionLosses;
