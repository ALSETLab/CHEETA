within CHEETA.Aircraft.Electrical;
package Controls


  model IEEEtype1AVR
    parameter Real T_R "Rate Filter Time Constant";
    parameter Real T_C, T_B "TGR Time Constants";
    parameter Real K_A "Regulator Gain";
    parameter Real T_A "Regulator Time Constant";
    parameter Real K_E "Exciter Gain";
    parameter Real T_E "Exciter Time Constant";
    parameter Real K_F "Rate Feedback Gain";
    parameter Real T_F "Rate Feedback Time Constant";
    parameter Real Vmax "Regulator Maximum Output";
    parameter Real Vmin "Regulator Minimum Output";
    parameter Real Vref "Terminal Reference Voltage (pu)";
    Modelica.Blocks.Interfaces.RealInput Vterm
      annotation (Placement(transformation(extent={{-240,-18},{-200,22}})));
    Modelica.Blocks.Continuous.TransferFunction LeadLag(b={T_C,1}, a={T_B,1},
    initType=Modelica.Blocks.Types.Init.SteadyState,
      x_start={0})
      annotation (Placement(transformation(extent={{-102,-8},{-82,12}})));
    Modelica.Blocks.Continuous.TransferFunction TGR(b={K_F,0}, a={T_F,0},
    initType=Modelica.Blocks.Types.Init.SteadyState)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-58,-40})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=Vmax, uMin=Vmin,
    homotopyType=Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy)
      annotation (Placement(transformation(extent={{-30,-8},{-10,12}})));
    Modelica.Blocks.Math.Feedback feedback1 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={6,2})));
    Modelica.Blocks.Math.Add3 SUM(k2=-1, k3=-1)
      annotation (Placement(transformation(extent={{-134,-8},{-114,12}})));
    Modelica.Blocks.Sources.Constant Reference(k=Vref) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-164,40})));
    Modelica.Blocks.Interfaces.RealOutput Ifd
      annotation (Placement(transformation(extent={{100,-16},{138,22}})));
    Modelica.Blocks.Continuous.TransferFunction Exciter(b={1}, a={T_E,K_E},
    initType=Modelica.Blocks.Types.Init.SteadyState)
      annotation (Placement(transformation(extent={{22,-8},{42,12}})));
    Modelica.Blocks.Continuous.TransferFunction LPF(
    b={1},
    a={T_R},
    initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{-176,-8},{-156,12}})));
    Modelica.Blocks.Continuous.TransferFunction LeadLag1(
    b={K_A},
    a={T_A},
    initType=Modelica.Blocks.Types.Init.SteadyState)
      annotation (Placement(transformation(extent={{-64,-8},{-44,12}})));
  equation
    connect(feedback1.u1, limiter.y)
      annotation (Line(points={{-2,2},{-9,2}}, color={0,0,127}));
    connect(TGR.u, feedback1.u2) annotation (Line(points={{-46,-40},{60,-40},{
          60,-22},{6,-22},{6,-6}},
                             color={0,0,127}));
    connect(SUM.y, LeadLag.u)
      annotation (Line(points={{-113,2},{-104,2}}, color={0,0,127}));
    connect(TGR.y, SUM.u3) annotation (Line(points={{-69,-40},{-142,-40},{-142,
          -6},{-136,-6}},
                        color={0,0,127}));
    connect(Reference.y, SUM.u1) annotation (Line(points={{-153,40},{-146,40},{
          -146,10},{-136,10}},
                            color={0,0,127}));
    connect(Ifd, feedback1.u2) annotation (Line(points={{119,3},{60,3},{60,
            -22},{6,-22},{6,-6}}, color={0,0,127}));
    connect(feedback1.y, Exciter.u)
      annotation (Line(points={{15,2},{20,2}}, color={0,0,127}));
    connect(Exciter.y, feedback1.u2) annotation (Line(points={{43,2},{60,2},{
            60,-22},{6,-22},{6,-6}}, color={0,0,127}));
  connect(SUM.u2, LPF.y)
    annotation (Line(points={{-136,2},{-155,2}}, color={0,0,127}));
  connect(Vterm, LPF.u)
    annotation (Line(points={{-220,2},{-178,2}}, color={0,0,127}));
  connect(LeadLag.y, LeadLag1.u)
    annotation (Line(points={{-81,2},{-66,2}}, color={0,0,127}));
  connect(limiter.u, LeadLag1.y)
    annotation (Line(points={{-32,2},{-43,2}}, color={0,0,127}));
      annotation (Placement(transformation(extent={{66,-20},{106,20}})),
                Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,
            -60},{100,60}}), graphics={  Rectangle(extent={{-200,60},{100,-60}},
              lineColor={28,108,200}), Text(
            extent={{-126,20},{44,-20}},
            lineColor={28,108,200},
            textString="IEEE Type 1")}),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
            extent={{-200,-60},{100,60}})),
    Documentation(info="<html>
<p>IEEE AVR Type 1 model</p>
</html>"));
  end IEEEtype1AVR;

annotation (Documentation(info="<html>
<p>This package contains models for the control system of the electrical system. This would include componets such as:</p>
<ul>
<li>AVR</li>
<li>PSS</li>
</ul>
</html>"));
end Controls;
