within CHEETA.Aircraft.Electrical.BusBar;
model Cu_CurrentLead

  parameter Real l = 0.38 "Lead length";
  parameter Real k_0 = 600;
  parameter Real h_L = 20.7e3;
  parameter Real c_p0 = 5.26e3;
  parameter Real I_0 = 8000 "Carrying current";
  parameter Real A= 1e-3 "Current lead area";
  Real R_c "Resistance";
  Real m_I "Mass flow rate";
  Real Q_I0 "Power input";
  Real V_0 "Voltage drop along lead";
  Real rho "Resistivity";




  Modelica.Electrical.Analog.Basic.VariableResistor
                                            resistor annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,2})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
    "Positive electrical pin"
    annotation (Placement(transformation(extent={{-10,60},{10,80}}),
        iconTransformation(extent={{-10,60},{10,80}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1
                "Negative electrical pin"
    annotation (Placement(transformation(extent={{-10,-80},{10,-60}}),
        iconTransformation(extent={{-10,-80},{10,-60}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=R_c)
    annotation (Placement(transformation(extent={{38,-8},{18,12}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),
        iconTransformation(extent={{-50,-10},{-30,10}})));
equation
        if port_a.T <50 then
            rho = 2.5e-10;
        else
            rho = ((1.75e-8 - 2.5e-10)/(273-50)) * port_a.T;
        end if;
        port_a.Q_flow=0;
        R_c = rho*p1.i^2*l / A;
        Q_I0 = DymolaModels.Functions.Math.divNoZero(k_0*rho*p1.i^2,(m_I*c_p0));
        V_0 = rho*p1.i*l / A;
        m_I = Q_I0/h_L;


  connect(resistor.p, p1)
    annotation (Line(points={{1.77636e-15,12},{0,70}},
                                             color={0,0,255}));
  connect(resistor.n, n1) annotation (Line(points={{-1.77636e-15,-8},{0,-70}},
        color={0,0,255}));
  connect(resistor.R, realExpression.y)
    annotation (Line(points={{12,2},{17,2}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-60},
            {40,60}}), graphics={Rectangle(extent={{-40,60},{40,-60}},
            lineColor={28,108,200})}),                           Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-40,-60},{40,60}})));
end Cu_CurrentLead;
