within CHEETA.Aircraft.Electrical.FuelCell;
model FuelCell_EquationBased
  parameter Real E_0 = 1239.764;

  parameter Real n = 20.145 "Intrinsic value";
  parameter Modelica.Units.SI.Pressure P_O2=1.6
    "Partial pressure of oxygen";
  parameter Modelica.Units.SI.Pressure P_H2=1.6
    "Partial pressure of hydrogen";
  Modelica.Electrical.Analog.Sources.SignalVoltage   E_cell         annotation (
     Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-48,-22})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            R_act
    annotation (Placement(transformation(extent={{-26,0},{-6,20}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            R_conc
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
  Modelica.Electrical.Analog.Basic.Capacitor C_dl(v(fixed=false),
                                                  C=10.6635)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={0,-12})));
    Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-58,-58},{-38,-38}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                "Positive electrical pin"
    annotation (Placement(transformation(extent={{90,0},{110,20}})));


  Real R = Modelica.Constants.R "Universal gas constant";
  Real F = Modelica.Constants.F "Faraday constant";
  Real alpha = 0.1373;


  Real E_Nernst;
  Real R_act_val;
  Real R_conc_val;
  Modelica.Units.SI.Current I_max=5000 "Maximum current of fuel cell";
  Real z;
  Modelica.Blocks.Sources.RealExpression realExpression(y=E_Nernst)
    annotation (Placement(transformation(extent={{-90,-32},{-70,-12}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=R_act_val)
    annotation (Placement(transformation(extent={{-50,34},{-30,54}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=R_conc_val)
    annotation (Placement(transformation(extent={{-50,50},{-30,70}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
  parameter Modelica.Units.SI.Resistance R_ohm_current=0.02
    "Resistance of current leads";
  Modelica.Electrical.Analog.Basic.Resistor R_ohm(R=R_ohm_current)
    annotation (Placement(transformation(extent={{44,0},{64,20}})));
equation
  E_Nernst = E_0 + R*port_a.T/(n*F) * ln(P_H2 * sqrt(P_O2));
  R_act_val = 1000*(R*port_a.T)/(alpha*n*F) * DymolaModels.Functions.Math.divNoZero(ln(-E_cell.i+ Modelica.Constants.eps),-E_cell.i);
  R_conc_val = -1000*DymolaModels.Functions.Math.divNoZero((R*port_a.T),-E_cell.i*n*F) * ln((-E_cell.i+Modelica.Constants.eps)/I_max);
  port_a.Q_flow = 0;
  z = 0;//R_act_val + R_conc_val + R_ohm_current;
  connect(R_act.p, E_cell.p)
    annotation (Line(points={{-26,10},{-48,10},{-48,-12}},color={0,0,255}));
  connect(R_act.n, R_conc.p)
    annotation (Line(points={{-6,10},{0,10}},color={0,0,255}));
  connect(E_cell.n, ground1.p)
    annotation (Line(points={{-48,-32},{-48,-38}}, color={0,0,255}));
  connect(C_dl.p, E_cell.p) annotation (Line(points={{-10,-12},{-30,-12},{-30,0},
          {-48,0},{-48,-12}},
                            color={0,0,255}));
  connect(E_cell.v, realExpression.y)
    annotation (Line(points={{-60,-22},{-69,-22}}, color={0,0,127}));
  connect(R_act.R, realExpression1.y)
    annotation (Line(points={{-16,22},{-16,44},{-29,44}}, color={0,0,127}));
  connect(realExpression2.y, R_conc.R)
    annotation (Line(points={{-29,60},{10,60},{10,22}},
                                                      color={0,0,127}));
  connect(R_ohm.p, R_conc.n)
    annotation (Line(points={{44,10},{20,10}}, color={0,0,255}));
  connect(C_dl.n, R_conc.n) annotation (Line(points={{10,-12},{32,-12},{32,10},
          {20,10}},
                color={0,0,255}));
  connect(R_ohm.n, p1) annotation (Line(points={{64,10},{100,10}},
                     color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuelCell_EquationBased;
