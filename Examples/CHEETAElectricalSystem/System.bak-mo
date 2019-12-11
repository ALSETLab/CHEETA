within CHEETA.Examples.CHEETAElectricalSystem;
model System
  Aircraft.Electrical.FuelCell.SimplifiedFuelCell simplifiedFuelCell(R=100, L=
        1e3,
    V=500)   annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={-86,0})));
  Modelica.Electrical.PowerConverters.DCDC.ChopperStepUp dcdc annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L=1e-6)
    annotation (Placement(transformation(extent={{-24,-4},{-4,16}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
      constantDutyCycle=0.5, f=10)
    annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L=1e-6)
    annotation (Placement(transformation(extent={{-24,-16},{-4,4}})));
  Modelica.Electrical.Machines.BasicMachines.AsynchronousInductionMachines.AIM_SquirrelCage
    aimc(
    p=8,
    fsNominal=300,
    TsOperational(displayUnit="K") = 20,
    Rs=1e-6,
    TsRef(displayUnit="K") = 20,
    Lssigma=0.041,
    Lm=21.76e-6,
    Lrsigma=6.69e-6,
    Rr=1e-6,
    TrRef(displayUnit="K") = 20,
    TrOperational(displayUnit="K") = 20)
    annotation (Placement(transformation(extent={{56,-10},{76,10}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox
                                 terminalBox(terminalConnection="Y")
    annotation (Placement(transformation(extent={{56,6},{76,26}})));
  Modelica.Electrical.PowerConverters.DCAC.MultiPhase2Level inverter1(
      useHeatPort=false, m=3)
    annotation (Placement(transformation(extent={{16,-10},{36,10}})));
  Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM signalPWM[3](
      constantDutyCycle=0.5, each f=10)           annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        origin={26,-40})));
  Modelica.Blocks.Sources.Constant const(k=1)
    annotation (Placement(transformation(extent={{-14,-50},{6,-30}})));
equation
  connect(simplifiedFuelCell.pin_p, dcdc.dc_p1) annotation (Line(points={{-79,4},
          {-72,4},{-72,6},{-60,6}}, color={0,0,255}));
  connect(simplifiedFuelCell.n1, dcdc.dc_n1) annotation (Line(points={{-79,-4},
          {-70,-4},{-70,-6},{-60,-6}}, color={0,0,255}));
  connect(dcdc.dc_p2, inductor.p)
    annotation (Line(points={{-40,6},{-24,6}}, color={0,0,255}));
  connect(dcdc.fire_p, pwm.fire)
    annotation (Line(points={{-56,-12},{-56,-19}}, color={255,0,255}));
  connect(inductor1.p, dcdc.dc_n2)
    annotation (Line(points={{-24,-6},{-40,-6}}, color={0,0,255}));
  connect(terminalBox.plug_sp, aimc.plug_sp)
    annotation (Line(points={{72,10},{72,10}}, color={0,0,255}));
  connect(terminalBox.plug_sn, aimc.plug_sn)
    annotation (Line(points={{60,10},{60,10}}, color={0,0,255}));
  connect(signalPWM.fire, inverter1.fire_p)
    annotation (Line(points={{20,-29},{20,-12}}, color={255,0,255}));
  connect(signalPWM.notFire, inverter1.fire_n)
    annotation (Line(points={{32,-29},{32,-12}}, color={255,0,255}));
  connect(inverter1.dc_n, inductor1.n)
    annotation (Line(points={{16,-6},{-4,-6}}, color={0,0,255}));
  connect(inverter1.dc_p, inductor.n)
    annotation (Line(points={{16,6},{-4,6}}, color={0,0,255}));
  connect(inverter1.ac, terminalBox.plugSupply) annotation (Line(points={{36,0},
          {48,0},{48,18},{66,18},{66,12}}, color={0,0,255}));
  connect(const.y, signalPWM[1].dutyCycle)
    annotation (Line(points={{7,-40},{14,-40}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The architecture of the CHEETA electrical system is shown below:</p>
<p><br><img src=\"modelica://CHEETA/Images/Electrical/CHEETASystem.PNG\"/></p>
</html>"),
    experiment(StopTime=1));
end System;
