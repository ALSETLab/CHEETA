within CHEETA.Aircraft.Electrical.FuelCell;
model FuelCell_EquationBased
  parameter Real E_0 = 1239.764;

  parameter Real n = 2 "Intrinsic value";
  parameter Modelica.SIunits.PartialPressure P_O2 = 1.6 "Partial pressure of oxygen";
  parameter Modelica.SIunits.PartialPressure P_H2 = 1.6 "Partial pressure of hydrogen";
  Modelica.Electrical.Analog.Sources.SignalVoltage   E_cell         annotation (
     Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-52,-18})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            R_act
    annotation (Placement(transformation(extent={{-30,4},{-10,24}})));
  Modelica.Electrical.Analog.Basic.VariableResistor
                                            R_conc
    annotation (Placement(transformation(extent={{-4,4},{16,24}})));
  Modelica.Electrical.Analog.Basic.Capacitor C_dl(v(start=239.7634, fixed=false),
                                                  C=10.6635)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-4,-8})));
  Modelica.Electrical.Analog.Basic.Resistor R_ohm(R=0.02)
    annotation (Placement(transformation(extent={{34,4},{54,24}})));
  Modelica.Electrical.Analog.Basic.Ground ground1
    annotation (Placement(transformation(extent={{-62,-54},{-42,-34}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                "Positive electrical pin"
    annotation (Placement(transformation(extent={{90,0},{110,20}})));


  Real R = Modelica.Constants.R "Universal gas constant";
  Real F = Modelica.Constants.F "Faraday constant";
  Real alpha = 0.5;


  Real E_Nernst;
  Real R_act_val;
  Real R_conc_val;
  Modelica.SIunits.Current I_max = 5000 "Maximum current of fuel cell";

  Modelica.Blocks.Sources.RealExpression realExpression(y=E_Nernst)
    annotation (Placement(transformation(extent={{-94,-28},{-74,-8}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=R_act_val)
    annotation (Placement(transformation(extent={{-54,38},{-34,58}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=R_conc_val)
    annotation (Placement(transformation(extent={{-54,54},{-34,74}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
equation
  E_Nernst = E_0 + R*port_a.T/(n*F) * ln(P_H2 * sqrt(P_O2));
  R_act_val = 1000*(R*port_a.T)/(alpha*n*F) * DymolaModels.Functions.Math.divNoZero(ln(-E_cell.i+ Modelica.Constants.eps),-E_cell.i);
  R_conc_val = -1000*DymolaModels.Functions.Math.divNoZero((R*port_a.T),-E_cell.i*n*F) * ln((-E_cell.i+Modelica.Constants.eps)/I_max);
  port_a.Q_flow = 0;
  connect(R_act.p, E_cell.p)
    annotation (Line(points={{-30,14},{-52,14},{-52,-8}}, color={0,0,255}));
  connect(R_act.n, R_conc.p)
    annotation (Line(points={{-10,14},{-4,14}},
                                             color={0,0,255}));
  connect(R_conc.n, R_ohm.p)
    annotation (Line(points={{16,14},{34,14}}, color={0,0,255}));
  connect(E_cell.n, ground1.p)
    annotation (Line(points={{-52,-28},{-52,-34}}, color={0,0,255}));
  connect(C_dl.p, E_cell.p) annotation (Line(points={{-14,-8},{-34,-8},{-34,4},{
          -52,4},{-52,-8}}, color={0,0,255}));
  connect(C_dl.n, R_ohm.p) annotation (Line(points={{6,-8},{24,-8},{24,14},{34,14}},
        color={0,0,255}));
  connect(R_ohm.n, p1) annotation (Line(points={{54,14},{80,14},{80,10},{100,10}},
        color={0,0,255}));
  connect(E_cell.v, realExpression.y)
    annotation (Line(points={{-64,-18},{-73,-18}}, color={0,0,127}));
  connect(R_act.R, realExpression1.y)
    annotation (Line(points={{-20,26},{-20,48},{-33,48}}, color={0,0,127}));
  connect(realExpression2.y, R_conc.R)
    annotation (Line(points={{-33,64},{6,64},{6,26}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuelCell_EquationBased;
