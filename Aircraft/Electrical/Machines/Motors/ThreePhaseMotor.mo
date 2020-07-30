within CHEETA.Aircraft.Electrical.Machines.Motors;
model ThreePhaseMotor
  import SI = Modelica.SIunits;
  import Modelica.SIunits.Conversions.*;
  parameter Integer m= 3 "Number of phases";

  parameter Real wref = 4000;
  Modelica.Electrical.Machines.BasicMachines.AsynchronousInductionMachines.AIM_SquirrelCage
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
    TrOperational=293.15) annotation (Placement(transformation(extent={{-10,-48},
            {10,-28}})));
  Modelica.Electrical.Machines.Utilities.TerminalBox
                                 terminalBox(terminalConnection="Y")
    annotation (Placement(transformation(extent={{-10,-32},{10,-12}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange1
                                                           "Shaft"
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Records.Boeing.Boeing747                         aimcData
    annotation (Placement(transformation(extent={{40,30},{60,50}})));
  Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plug_p1
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
protected
  parameter Real rpm=from_rpm(wref) "Reference speed of the generator";
equation
  connect(terminalBox.plug_sn, aimc.plug_sn)
    annotation (Line(points={{-6,-28},{-6,-28}}, color={0,0,255}));
  connect(terminalBox.plug_sp, aimc.plug_sp)
    annotation (Line(points={{6,-28},{6,-28}}, color={0,0,255}));
  connect(aimc.flange, flange1) annotation (Line(points={{10,-38},{26,-38},{26,
          0},{100,0}},
                    color={0,0,0}));
  connect(terminalBox.plugSupply, plug_p1)
    annotation (Line(points={{0,-26},{0,0},{-100,0}}, color={0,0,255}));
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
end ThreePhaseMotor;
