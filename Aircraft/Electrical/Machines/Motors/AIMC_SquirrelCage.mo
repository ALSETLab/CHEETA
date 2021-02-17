within CHEETA.Aircraft.Electrical.Machines.Motors;
model AIMC_SquirrelCage
  import      Modelica.Units.SI;
  import Modelica.Units.Conversions.*;
  parameter Integer m= 3 "Number of phases";

  parameter Real wref = 4000;
  Modelica.Electrical.Machines.BasicMachines.InductionMachines.IM_SquirrelCage
    aimc(
    p=aimcData.p,
    fsNominal=aimcData.fs_nom,
    Rs=aimcData.Rs,
    TsRef=aimcData.Ts_ref,
    alpha20s(displayUnit="1/K") = aimcData.alpha20s,
    Lszero=aimcData.Ls_zero,
    Lssigma=aimcData.Ls_sigma,
    Jr=aimcData.Jr,
    Js=aimcData.Js,
    frictionParameters=aimcData.frictionParameters,
    statorCoreParameters=aimcData.statorCoreParameters,
    strayLoadParameters=aimcData.strayLoadParameters,
    Lm=aimcData.Lm,
    Lrsigma=aimcData.Lr_sigma,
    Rr=aimcData.Rr,
    TrRef=aimcData.Tr_ref,
    TsOperational=293.15,
    alpha20r=aimcData.alpha20r,
    TrOperational=293.15)
    annotation (Placement(transformation(extent={{-10,-48},{10,-28}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox
                                 terminalBox(terminalConnection="Y")
    annotation (Placement(transformation(extent={{-10,-32},{10,-12}})));
  Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor
                                         currentQuasiRMSSensor
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={0,0})));
  Modelica.Electrical.Polyphase.Sources.SignalVoltage signalVoltage(final m=m)
    annotation (Placement(transformation(
        origin={0,32},
        extent={{10,10},{-10,-10}},
        rotation=270)));
  Modelica.Electrical.Polyphase.Basic.Star star(final m=m) annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
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
    annotation (Placement(transformation(extent={{-44,22},{-24,42}})));
  parameter Records.Sample_AIM_Machine_10kW                  aimcData
    annotation (Placement(transformation(extent={{40,30},{60,50}})));
  Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor
    annotation (Placement(transformation(extent={{-82,-10},{-62,10}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                           "pin to be measured"
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
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
  connect(realExtend.y, signalVoltage.v) annotation (Line(points={{-23,32},{-12,
          32}},                    color={0,0,127}));
  connect(realExtend.u, potentialSensor.phi) annotation (Line(points={{-46,32},
          {-54,32},{-54,0},{-61,0}}, color={0,0,127}));
  connect(potentialSensor.p, p1)
    annotation (Line(points={{-82,0},{-100,0}}, color={0,0,255}));
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
