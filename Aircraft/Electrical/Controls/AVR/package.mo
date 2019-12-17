within CHEETA.Aircraft.Electrical.Controls;
package AVR

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
      annotation (Placement(transformation(extent={{-240,-20},{-200,20}}),
        iconTransformation(extent={{-240,-20},{-200,20}})));
    Modelica.Blocks.Continuous.FirstOrder LPF(k=1, T=T_R,
      initType=Modelica.Blocks.Types.Init.SteadyState,
      y_start=0)                                          "Low Pass Filter"
      annotation (Placement(transformation(extent={{-172,-8},{-152,12}})));
    Modelica.Blocks.Continuous.TransferFunction LeadLag(b={T_C,1}, a={T_B,1},
      initType=Modelica.Blocks.Types.Init.NoInit,
      x_start={0})
      annotation (Placement(transformation(extent={{-102,-8},{-82,12}})));
    Modelica.Blocks.Continuous.FirstOrder Regulator(k=K_A, T=T_A,
      initType=Modelica.Blocks.Types.Init.NoInit)                 "Regulator"
      annotation (Placement(transformation(extent={{-68,-8},{-48,12}})));
    Modelica.Blocks.Continuous.TransferFunction TGR(b={K_F,0}, a={T_F,1})
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-64,-68})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=Vmax, uMin=Vmin)
      annotation (Placement(transformation(extent={{-30,-8},{-10,12}})));
    Modelica.Blocks.Math.Add3 SUM(k2=-1, k3=-1)
      annotation (Placement(transformation(extent={{-134,-8},{-114,12}})));
    Modelica.Blocks.Sources.Constant Reference(k=Vref) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-162,50})));
    Modelica.Blocks.Interfaces.RealOutput Ifd
      annotation (Placement(transformation(extent={{100,-20},{138,18}}),
        iconTransformation(extent={{100,-20},{138,18}})));
    Modelica.Blocks.Math.Gain gain(k=K_E) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={34,-44})));
    Modelica.Blocks.Math.Add3 add3_1(k2=-1, k3=-1)
      annotation (Placement(transformation(extent={{16,-16},{36,4}})));
    Modelica.Blocks.Math.Gain gain1(k=1/T_E) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={56,-6})));
    Modelica.Blocks.Continuous.Integrator integrator
      annotation (Placement(transformation(extent={{74,-16},{94,4}})));
  equation
    connect(LPF.u, Vterm)
      annotation (Line(points={{-174,2},{-198,2},{-198,0},{-220,0}},
                                                   color={0,0,127}));
    connect(Regulator.u, LeadLag.y)
      annotation (Line(points={{-70,2},{-81,2}}, color={0,0,127}));
    connect(limiter.u, Regulator.y)
      annotation (Line(points={{-32,2},{-47,2}}, color={0,0,127}));
    connect(SUM.y, LeadLag.u)
      annotation (Line(points={{-113,2},{-104,2}}, color={0,0,127}));
    connect(LPF.y, SUM.u2)
      annotation (Line(points={{-151,2},{-136,2}}, color={0,0,127}));
    connect(TGR.y, SUM.u3) annotation (Line(points={{-75,-68},{-142,-68},{
            -142,-6},{-136,-6}},
                        color={0,0,127}));
    connect(Reference.y, SUM.u1) annotation (Line(points={{-151,50},{-146,50},
            {-146,10},{-136,10}},
                            color={0,0,127}));
    connect(add3_1.u1, limiter.y)
      annotation (Line(points={{14,2},{-9,2}}, color={0,0,127}));
    connect(gain.y, add3_1.u3) annotation (Line(points={{23,-44},{-2,-44},{-2,
            -14},{14,-14}}, color={0,0,127}));
    connect(gain1.u, add3_1.y)
      annotation (Line(points={{44,-6},{37,-6}}, color={0,0,127}));
    connect(integrator.u, gain1.y)
      annotation (Line(points={{72,-6},{67,-6}}, color={0,0,127}));
    connect(gain.u, integrator.y)
      annotation (Line(points={{46,-44},{95,-44},{95,-6}}, color={0,0,127}));
    connect(add3_1.u2, integrator.y) annotation (Line(points={{14,-6},{10,-6},
            {10,-28},{95,-28},{95,-6}}, color={0,0,127}));
    connect(TGR.u, integrator.y) annotation (Line(points={{-52,-68},{96,-68},
            {96,-44},{95,-44},{95,-6}}, color={0,0,127}));
    connect(integrator.y, Ifd) annotation (Line(points={{95,-6},{98,-6},{98,-1},
          {119,-1}},  color={0,0,127}));
      annotation (Placement(transformation(extent={{66,-20},{106,20}})),
                Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},
              {100,100}})), Diagram(coordinateSystem(preserveAspectRatio=false,
            extent={{-200,-100},{100,100}})));
  end IEEEtype1AVR;
end AVR;
