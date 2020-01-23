within CHEETA.Aircraft.Electrical.Machines;
model AIMC_SquirrelCage
  import SI = Modelica.SIunits;
  import Modelica.SIunits.Conversions.*;
  parameter Integer m= 3 "Number of phases";

  Modelica.Blocks.Interfaces.RealInput v1
    "Voltage between pin p and n (= p.v - n.v) as input signal" annotation (
      Placement(transformation(extent={{-114,-20},{-74,20}}),
        iconTransformation(extent={{-94,0},{-74,20}})));
  parameter Real wref = 4000;
  Modelica.Electrical.Machines.BasicMachines.AsynchronousInductionMachines.AIM_SquirrelCage
    aimc(
    p=aimcData.p,
    fsNominal=aimcData.fsNominal,
    Rs=aimcData.Rs,
    TsRef=aimcData.TsRef,
    alpha20s(displayUnit="1/K") = aimcData.alpha20s,
    Lszero=aimcData.Lszero,
    Lssigma=aimcData.Lssigma,
    Jr=aimcData.Jr,
    Js=aimcData.Js,
    frictionParameters=aimcData.frictionParameters,
    statorCoreParameters=aimcData.statorCoreParameters,
    strayLoadParameters=aimcData.strayLoadParameters,
    Lm=aimcData.Lm,
    Lrsigma=aimcData.Lrsigma,
    Rr=aimcData.Rr,
    TrRef=aimcData.TrRef,
    TsOperational=293.15,
    alpha20r=aimcData.alpha20r,
    TrOperational=293.15) annotation (Placement(transformation(extent={{-10,-48},
            {10,-28}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox
                                 terminalBox(terminalConnection="Y")
    annotation (Placement(transformation(extent={{-10,-32},{10,-12}})));
  Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor
                                         currentQuasiRMSSensor
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={0,0})));
  Modelica.Electrical.MultiPhase.Sources.SignalVoltage signalVoltage(final m=m)
                 annotation (Placement(transformation(
        origin={0,32},
        extent={{10,10},{-10,-10}},
        rotation=270)));
  Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) annotation (
      Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=270,
        origin={0,60})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        origin={0,86},
        extent={{-10,-10},{10,10}},
        rotation=180)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange1
                                                           "Shaft"
    annotation (Placement(transformation(extent={{82,-10},{102,10}})));
  Blocks.Routing.RealExtend realExtend
    annotation (Placement(transformation(extent={{-62,-10},{-42,10}})));
  parameter Records.NotionalPowerSystem.AIM_SquirrelCageData aimcData
    annotation (Placement(transformation(extent={{40,30},{60,50}})));
protected
  parameter Real rpm=from_rpm(wref) "Reference speed of the generator";
equation
  connect(signalVoltage.plug_p, currentQuasiRMSSensor.plug_p) annotation (Line(
        points={{-1.77636e-15,22},{0,22},{0,10},{1.77636e-15,10}}, color={0,0,255}));
  connect(terminalBox.plug_sn, aimc.plug_sn)
    annotation (Line(points={{-6,-28},{-6,-28}}, color={0,0,255}));
  connect(terminalBox.plug_sp, aimc.plug_sp)
    annotation (Line(points={{6,-28},{6,-28}}, color={0,0,255}));
  connect(currentQuasiRMSSensor.plug_n, terminalBox.plugSupply) annotation (
      Line(points={{-1.77636e-15,-10},{-1.77636e-15,-18},{0,-18},{0,-26}},
        color={0,0,255}));
  connect(signalVoltage.plug_n, star.plug_p) annotation (Line(points={{1.77636e-15,
          42},{0,42},{0,50}}, color={0,0,255}));
  connect(star.pin_n, ground.p) annotation (Line(points={{1.83187e-15,70},{0,70},
          {0,76}}, color={0,0,255}));
  connect(aimc.flange, flange1) annotation (Line(points={{10,-38},{26,-38},{26,
          0},{92,0}},
                    color={0,0,0}));
  connect(v1, realExtend.u)
    annotation (Line(points={{-94,0},{-64,0}},  color={0,0,127}));
  connect(realExtend.y, signalVoltage.v) annotation (Line(points={{-41,0},{
          -33.5,0},{-33.5,32},{-12,32}},
                                   color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
            {100,100}}), graphics={
        Rectangle(
          extent={{-72,68},{48,-52}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,128,255}),
        Rectangle(
          extent={{-72,68},{-92,-52}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={128,128,128}),
        Rectangle(
          extent={{48,18},{68,-2}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={95,95,95}),
        Rectangle(
          extent={{-72,78},{8,58}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-82,-82},{-72,-82},{-42,-12},{8,-12},{38,-82},{48,-82},{48,
              -92},{-82,-92},{-82,-82}},
          fillPattern=FillPattern.Solid)}),
                          Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end AIMC_SquirrelCage;
