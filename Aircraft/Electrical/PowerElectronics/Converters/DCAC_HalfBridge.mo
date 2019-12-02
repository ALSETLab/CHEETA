within CHEETA.Aircraft.Electrical.PowerElectronics.Converters;
model DCAC_HalfBridge "DC AC half bridge converter with PWM input"
  Modelica.Electrical.Analog.Ideal.IdealGTOThyristor Q1(Ron=Ron)
                                                        annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=-90,
        origin={0,30})));
  Modelica.Electrical.Analog.Ideal.IdealDiode D1(Ron=Ron)
                                                 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,30})));
  Modelica.Electrical.Analog.Ideal.IdealGTOThyristor Q4(Ron=Ron)
                                                        annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=-90,
        origin={0,-30})));
  Modelica.Electrical.Analog.Ideal.IdealDiode D4(Ron=Ron)
                                                 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,-30})));
  Modelica.Blocks.Interfaces.BooleanInput Q1gate
    annotation (Placement(transformation(extent={{-102,10},{-80,32}}),
        iconTransformation(extent={{-102,10},{-80,32}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin ac
    annotation (Placement(transformation(extent={{80,-10},{100,10}}),
        iconTransformation(extent={{80,-10},{100,10}})));
  Modelica.Blocks.Interfaces.BooleanInput Q4gate
    annotation (Placement(transformation(extent={{-102,-32},{-80,-10}}),
        iconTransformation(extent={{-102,-32},{-80,-10}})));
  parameter Modelica.SIunits.Resistance Ron=1
    "Forward state-on differential resistance (closed resistance)";
  Modelica.Electrical.Analog.Interfaces.PositivePin ground
    annotation (Placement(transformation(extent={{-100,-70},{-80,-50}}),
        iconTransformation(extent={{-100,-70},{-80,-50}})));
  parameter Modelica.SIunits.Voltage Vd1=600 "Value of constant voltage";
  parameter Modelica.SIunits.Voltage Vd2=600 "Value of constant voltage";
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                "Positive electrical pin" annotation (Placement(transformation(
          extent={{-100,52},{-80,72}}), iconTransformation(extent={{-100,52},{
            -80,72}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort(final T=
        T_heatPort, final Q_flow=-LossPower) if                                                                useHeatPort
    "Conditional heat port"
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
        iconTransformation(extent={{-10,-110},{10,-90}})));
equation
  connect(Q4.p, ac)
    annotation (Line(points={{0,-20},{0,0},{90,0}}, color={0,0,255}));
  connect(Q1gate, Q1.fire)
    annotation (Line(points={{-91,21},{-36,21},{-36,20},{-12,20}},
                                                 color={255,0,255}));
  connect(Q4gate, Q4.fire)
    annotation (Line(points={{-91,-21},{-36,-21},{-36,-40},{-12,-40}},
                                                   color={255,0,255}));
  connect(Q1.n, ac)
    annotation (Line(points={{0,20},{0,0},{90,0}}, color={0,0,255}));
  connect(D1.p, ac) annotation (Line(points={{40,20},{20,20},{20,10},{0,10},{0,
          0},{90,0}},   color={0,0,255}));
  connect(D4.n, ac) annotation (Line(points={{40,-20},{40,-12},{0,-12},{0,0},{
          90,0}},  color={0,0,255}));
  connect(D4.p, Q4.n) annotation (Line(points={{40,-40},{40,-52},{0,-52},{0,-40},
          {-1.77636e-15,-40}},      color={0,0,255}));
  connect(D1.n, Q1.p) annotation (Line(points={{40,40},{40,54},{0,54},{0,40},
          {1.83187e-15,40}}, color={0,0,255}));
  connect(Q1.p, p1)
    annotation (Line(points={{0,40},{0,62},{-90,62}}, color={0,0,255}));
  connect(ground, Q4.n) annotation (Line(points={{-90,-60},{0,-60},{0,-40},{
          -1.77636e-15,-40}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-80},
            {80,80}}), graphics={
        Rectangle(extent={{-80,80},{80,-80}}, lineColor={28,108,200}),
                                Rectangle(
        extent={{-80,-80},{80,80}},
        lineColor={0,0,127},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),
        Line(
          points={{-80,-80},{80,80}},
          color={0,0,127}),
        Rectangle(
          extent={{-40,40},{40,-40}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-20,20},{-20,-20}},
          color={0,0,255}),
        Line(
          points={{-28,20},{-28,-20}},
          color={0,0,255}),
        Line(
          points={{-40,0},{-28,0}},
          color={0,0,255}),
        Line(
          points={{-20,4},{0,24},{0,40}},
          color={0,0,255}),
        Line(
          points={{-20,-4},{0,-24},{0,-40}},
          color={0,0,255}),
        Line(
          points={{-4,-20},{-10,-8},{-16,-14},{-4,-20}},
          color={0,0,255}),
        Line(
          points={{0,-24},{10,-24},{10,24},{0,24}},
          color={0,0,255}),
        Line(
          points={{0,8},{20,8}},
          color={0,0,255}),
        Line(
          points={{10,8},{0,-8},{20,-8},{10,8}},
          color={0,0,255}),
        Text(
          extent={{-100,70},{0,50}},
          lineColor={0,0,127},
          textString="DC"),
        Text(
          extent={{0,-50},{100,-70}},
          lineColor={0,0,127},
          textString="AC")}),  Diagram(coordinateSystem(preserveAspectRatio=
            false, extent={{-80,-80},{80,80}})));
end DCAC_HalfBridge;
