within CHEETA.Aircraft.Electrical.HTS.Stekly;
model Stekly_CopperLosses "HTS line using Stekly equations"
  parameter Modelica.Units.SI.Length l "Length of wire";
  parameter Modelica.Units.SI.ElectricFieldStrength E_0=1e-4
    "Reference electric field";
parameter Real n = 2 "Intrinstic value of the superconductor";
parameter Real I_c0 = 1 "Reference corner current";
parameter Real dT = 2 "Change in temperature";
  parameter Modelica.Units.SI.Area A=1 "Area";
  parameter Modelica.Units.SI.Length P=0.1035 "Perimeter of cable";
  parameter Modelica.Units.SI.Resistivity rho=2.1e-9 "Resitivity of copper";
  parameter Modelica.Units.SI.Current I_crit "Critical current";
  Modelica.Units.SI.Current I_c "corner current";
  Modelica.Units.SI.ElectricFieldStrength E "Electric field";
  Modelica.Units.SI.Power G;
  Modelica.Units.SI.Voltage x;


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
port_a.Q_flow = (0.6953+0.001079*dT^4)*A;
I_c = I_c0 *port_a.T;
E = E_0 *(pin_p.i/I_c)^n;
pin_n.i = pin_p.i;
G = rho * I_c^2 * 10^3 / P;
x = noEvent(if pin_p.i > I_crit then 0 else pin_p.v);
pin_n.v =x- E*l;
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
end Stekly_CopperLosses;
