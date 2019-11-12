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
      annotation (Placement(transformation(extent={{-240,-20},{-200,20}})));
    Modelica.Blocks.Continuous.TransferFunction LeadLag(b={T_C,1}, a={T_B,1},
    initType=Modelica.Blocks.Types.Init.SteadyState,
      x_start={0})
      annotation (Placement(transformation(extent={{-102,-10},{-82,10}})));
    Modelica.Blocks.Continuous.TransferFunction TGR(b={K_F,0}, a={T_F,0},
    initType=Modelica.Blocks.Types.Init.SteadyState)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-58,-42})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=Vmax, uMin=Vmin,
    homotopyType=Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy)
      annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
    Modelica.Blocks.Math.Feedback feedback1 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={6,0})));
    Modelica.Blocks.Math.Add3 SUM(k2=-1, k3=-1)
      annotation (Placement(transformation(extent={{-134,-10},{-114,10}})));
    Modelica.Blocks.Sources.Constant Reference(k=Vref) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-164,38})));
    Modelica.Blocks.Interfaces.RealOutput Ifd
      annotation (Placement(transformation(extent={{100,-20},{138,18}})));
    Modelica.Blocks.Continuous.TransferFunction Exciter(b={1}, a={T_E,K_E},
    initType=Modelica.Blocks.Types.Init.SteadyState)
      annotation (Placement(transformation(extent={{22,-10},{42,10}})));
    Modelica.Blocks.Continuous.FirstOrder LPF(
    k=1,
    T=T_R,
    initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{-176,-10},{-156,10}})));
    Modelica.Blocks.Continuous.FirstOrder Regulator(
    k=K_A,
    T=T_A,
    initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{-66,-10},{-46,10}})));
  equation
    connect(feedback1.u1, limiter.y)
      annotation (Line(points={{-2,0},{-9,0}}, color={0,0,127}));
    connect(TGR.u, feedback1.u2) annotation (Line(points={{-46,-42},{60,-42},{
          60,-24},{6,-24},{6,-8}},
                             color={0,0,127}));
    connect(SUM.y, LeadLag.u)
      annotation (Line(points={{-113,0},{-104,0}}, color={0,0,127}));
    connect(TGR.y, SUM.u3) annotation (Line(points={{-69,-42},{-142,-42},{-142,
          -8},{-136,-8}},
                        color={0,0,127}));
    connect(Reference.y, SUM.u1) annotation (Line(points={{-153,38},{-146,38},{
          -146,8},{-136,8}},color={0,0,127}));
    connect(Ifd, feedback1.u2) annotation (Line(points={{119,-1},{60,-1},{60,
          -24},{6,-24},{6,-8}},   color={0,0,127}));
    connect(feedback1.y, Exciter.u)
      annotation (Line(points={{15,0},{20,0}}, color={0,0,127}));
    connect(Exciter.y, feedback1.u2) annotation (Line(points={{43,0},{60,0},{60,
          -24},{6,-24},{6,-8}},      color={0,0,127}));
  connect(SUM.u2, LPF.y)
    annotation (Line(points={{-136,0},{-155,0}}, color={0,0,127}));
  connect(Vterm, LPF.u)
    annotation (Line(points={{-220,0},{-178,0}}, color={0,0,127}));
  connect(LeadLag.y, Regulator.u)
    annotation (Line(points={{-81,0},{-68,0}}, color={0,0,127}));
  connect(limiter.u, Regulator.y)
    annotation (Line(points={{-32,0},{-45,0}}, color={0,0,127}));
  connect(Ifd, Ifd)
    annotation (Line(points={{119,-1},{119,-1}}, color={0,0,127}));
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
