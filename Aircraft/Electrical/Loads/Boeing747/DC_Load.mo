within CHEETA.Aircraft.Electrical.Loads.Boeing747;
model DC_Load "DC load interface"
  parameter Integer m=3 "Number of Phases";
  parameter Real R2 = 6.9143e-5 "Secondary Winding Resistance";
  parameter Real L2 = Modelica.Constants.eps "Secondary Winding Inductance";
  parameter Real R1 = 0.0019048 "Primary Winding Resistance";
  parameter Real L1 = Modelica.Constants.eps "Primary Winding Inductance";
  parameter Real Rm = 4761.9 "Magnetization Resistance";
  parameter Real Lm = 12.631 "Magnetization Inductance";
  parameter Real N = 200/(22*sqrt(3)) "Transformer Turn Ratio";
  parameter Real C = 1e-6 "AC Side Shunt Capacitor";
  extends CHEETA.Aircraft.Electrical.Interfaces.Loads;
  Transformers.Yd                                yd
    annotation (Placement(transformation(extent={{-72,-10},{-46,10}})));
  PowerElectronics.Converters.ACDC.Simulink_Averaged_Rectifier simulink_Averaged_Rectifier(P_fixed=
        P_fixed, V_rated=V_rated)
    annotation (Placement(transformation(extent={{-26,-8},{-10,8}})));
  parameter Modelica.SIunits.Power P_fixed;
  parameter Modelica.SIunits.Voltage V_rated;
equation
  connect(simulink_Averaged_Rectifier.AC, yd.Secondary1)
    annotation (Line(points={{-27,0},{-45.6,0}}, color={0,0,255}));
  connect(yd.Primary, AC_in)
    annotation (Line(points={{-72.2,0},{-100,0}}, color={0,0,255}));
end DC_Load;
