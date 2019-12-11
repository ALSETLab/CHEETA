within CHEETA.Aircraft.Electrical.Controls.AVR;
model AVR_TypeI
  extends CHEETA.Aircraft.Electrical.Interfaces.Controller;
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
  Modelica.Blocks.Continuous.FirstOrder LPF(
    k=1,
    T=T_R,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    y_start=0)                                          "Low Pass Filter"
    annotation (Placement(transformation(extent={{-194,-2},{-174,18}})));
  Modelica.Blocks.Continuous.TransferFunction LeadLag(
    b={T_C,1},
    a={T_B,1},
    initType=Modelica.Blocks.Types.Init.NoInit,
    x_start={0})
    annotation (Placement(transformation(extent={{-124,-2},{-104,18}})));
  Modelica.Blocks.Continuous.FirstOrder Regulator(
    k=K_A,
    T=T_A,
    initType=Modelica.Blocks.Types.Init.NoInit)                 "Regulator"
    annotation (Placement(transformation(extent={{-90,-2},{-70,18}})));
  Modelica.Blocks.Continuous.TransferFunction TGR(b={K_F,0}, a={T_F,1})
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-86,-62})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax=Vmax, uMin=Vmin)
    annotation (Placement(transformation(extent={{-52,-2},{-32,18}})));
  Modelica.Blocks.Math.Add3 SUM(k2=-1, k3=-1)
    annotation (Placement(transformation(extent={{-156,-2},{-136,18}})));
  Modelica.Blocks.Math.Gain gain(k=K_E) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={12,-38})));
  Modelica.Blocks.Math.Add3 add3_1(k2=-1, k3=-1)
    annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
  Modelica.Blocks.Math.Gain gain1(k=1/T_E) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={34,0})));
  Modelica.Blocks.Continuous.Integrator integrator
    annotation (Placement(transformation(extent={{52,-10},{72,10}})));
equation
  connect(Regulator.u,LeadLag. y)
    annotation (Line(points={{-92,8},{-103,8}},color={0,0,127}));
  connect(limiter.u,Regulator. y)
    annotation (Line(points={{-54,8},{-69,8}}, color={0,0,127}));
  connect(SUM.y,LeadLag. u)
    annotation (Line(points={{-135,8},{-126,8}}, color={0,0,127}));
  connect(LPF.y,SUM. u2)
    annotation (Line(points={{-173,8},{-158,8}}, color={0,0,127}));
  connect(TGR.y,SUM. u3) annotation (Line(points={{-97,-62},{-164,-62},{-164,0},
          {-158,0}},  color={0,0,127}));
  connect(add3_1.u1,limiter. y)
    annotation (Line(points={{-8,8},{-31,8}},color={0,0,127}));
  connect(gain.y,add3_1. u3) annotation (Line(points={{1,-38},{-24,-38},{-24,-8},
          {-8,-8}},       color={0,0,127}));
  connect(gain1.u,add3_1. y)
    annotation (Line(points={{22,0},{15,0}},   color={0,0,127}));
  connect(integrator.u,gain1. y)
    annotation (Line(points={{50,0},{45,0}},   color={0,0,127}));
  connect(gain.u,integrator. y)
    annotation (Line(points={{24,-38},{73,-38},{73,0}},  color={0,0,127}));
  connect(add3_1.u2,integrator. y) annotation (Line(points={{-8,0},{-12,0},{-12,
          -22},{73,-22},{73,0}},      color={0,0,127}));
  connect(TGR.u,integrator. y) annotation (Line(points={{-74,-62},{74,-62},{74,-38},
          {73,-38},{73,0}},           color={0,0,127}));
  connect(integrator.y, Actuation)
    annotation (Line(points={{73,0},{110,0}}, color={0,0,127}));
  connect(SUM.u1, Reference) annotation (Line(points={{-158,16},{-164,16},{-164,
          40},{-280,40}}, color={0,0,127}));
  connect(LPF.u, Measurement) annotation (Line(points={{-196,8},{-228,8},{-228,-40},
          {-280,-40}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-280,-100},{100,100}})), Icon(
        coordinateSystem(extent={{-280,-100},{100,100}})));
end AVR_TypeI;
