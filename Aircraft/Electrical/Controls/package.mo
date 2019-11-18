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
    Modelica.Blocks.Continuous.FirstOrder LPF(k=1, T=T_R,
      initType=Modelica.Blocks.Types.Init.InitialState,
      y_start=0)                                          "Low Pass Filter"
      annotation (Placement(transformation(extent={{-172,-10},{-152,10}})));
    Modelica.Blocks.Continuous.TransferFunction LeadLag(b={T_C,1}, a={T_B,1},
      initType=Modelica.Blocks.Types.Init.SteadyState)
      annotation (Placement(transformation(extent={{-102,-10},{-82,10}})));
    Modelica.Blocks.Continuous.FirstOrder Regulator(k=K_A, T=T_A,
      initType=Modelica.Blocks.Types.Init.InitialState)           "Regulator"
      annotation (Placement(transformation(extent={{-68,-10},{-48,10}})));
    Modelica.Blocks.Continuous.TransferFunction TGR(b={K_F,0}, a={T_F,0},
      initType=Modelica.Blocks.Types.Init.SteadyState)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-60,-40})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=Vmax, uMin=Vmin)
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
          origin={-162,40})));
    Modelica.Blocks.Interfaces.RealOutput Ifd
      annotation (Placement(transformation(extent={{100,-16},{132,16}})));
    Modelica.Blocks.Continuous.TransferFunction Exciter(b={1}, a={T_E,K_E},
      initType=Modelica.Blocks.Types.Init.SteadyState)
      annotation (Placement(transformation(extent={{22,-10},{42,10}})));
  equation
    connect(LPF.u, Vterm)
      annotation (Line(points={{-174,0},{-220,0}}, color={0,0,127}));
    connect(Regulator.u, LeadLag.y)
      annotation (Line(points={{-70,0},{-81,0}}, color={0,0,127}));
    connect(limiter.u, Regulator.y)
      annotation (Line(points={{-32,0},{-47,0}}, color={0,0,127}));
    connect(feedback1.u1, limiter.y)
      annotation (Line(points={{-2,0},{-9,0}}, color={0,0,127}));
    connect(TGR.u, feedback1.u2) annotation (Line(points={{-48,-40},{60,-40},
            {60,-24},{6,-24},{6,-8}},
                             color={0,0,127}));
    connect(SUM.y, LeadLag.u)
      annotation (Line(points={{-113,0},{-104,0}}, color={0,0,127}));
    connect(LPF.y, SUM.u2)
      annotation (Line(points={{-151,0},{-136,0}}, color={0,0,127}));
    connect(TGR.y, SUM.u3) annotation (Line(points={{-71,-40},{-142,-40},{
            -142,-8},{-136,-8}},
                        color={0,0,127}));
    connect(Reference.y, SUM.u1) annotation (Line(points={{-151,40},{-146,40},
            {-146,8},{-136,8}},
                            color={0,0,127}));
    connect(Ifd, feedback1.u2) annotation (Line(points={{116,0},{60,0},{60,
            -24},{6,-24},{6,-8}}, color={0,0,127}));
    connect(feedback1.y, Exciter.u)
      annotation (Line(points={{15,0},{20,0}}, color={0,0,127}));
    connect(Exciter.y, feedback1.u2) annotation (Line(points={{43,0},{60,0},{
            60,-24},{6,-24},{6,-8}}, color={0,0,127}));
      annotation (Placement(transformation(extent={{66,-20},{106,20}})),
                Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,
              -60},{100,60}}), graphics={Rectangle(extent={{-200,60},{100,-60}},
              lineColor={28,108,200}), Text(
            extent={{-126,20},{44,-20}},
            lineColor={28,108,200},
            textString="IEEE Type 1")}),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
            extent={{-200,-60},{100,60}})));
  end IEEEtype1AVR;

annotation (Documentation(info="<html>
<p>This package contains models for the control system of the electrical system. This would include componets such as:</p>
<ul>
<li>AVR</li>
<li>PSS</li>
</ul>
</html>"));
end Controls;
