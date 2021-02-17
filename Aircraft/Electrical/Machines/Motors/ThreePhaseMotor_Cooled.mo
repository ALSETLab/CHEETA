within CHEETA.Aircraft.Electrical.Machines.Motors;
model ThreePhaseMotor_Cooled "3 phase motor with cooling"
  import      Modelica.Units.SI;
  import Modelica.Units.Conversions.*;
  parameter Integer m= 3 "Number of phases";

  parameter Real wref = 4000;
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange1
                                                           "Shaft"
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Electrical.Polyphase.Interfaces.PositivePlug plug_p1
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  ElectrifiedPowertrains.ElectricMachines.PSM.ElectroMechanical.Linear
    linearPSM(redeclare
      ElectrifiedPowertrains.ElectricMachines.PSM.ElectroMechanical.Records.Data.Linear.Automotive_100kW
      data, useThermalPort=true)
            annotation (Placement(transformation(extent={{-10,10},{10,-10}})));
  ElectrifiedPowertrains.ElectricDrives.Interfaces.Bus electricDriveBus1
    annotation (Placement(transformation(extent={{-10,90},{10,110}})));
              ElectrifiedPowertrains.ElectricMachines.PSM.Thermal.ThreeMassTEFC
                                                                         machineThermal(
      redeclare
      ElectrifiedPowertrains.ElectricMachines.PSM.Thermal.Records.Data.ForcedCoolingThreeMassEstimation2.Industrial_550W
      data)
    annotation (Placement(transformation(extent={{-4,-44},{16,-64}})));
  replaceable Modelica.Thermal.FluidHeatFlow.Media.Air_30degC coolingMedium constrainedby
    Modelica.Thermal.FluidHeatFlow.Media.Air_30degC
                                                annotation (Placement(transformation(extent={{-78,62},
            {-66,74}})));
  Modelica.Thermal.FluidHeatFlow.Interfaces.FlowPort_a flowPort_a1
    annotation (Placement(transformation(extent={{-90,-110},{-70,-90}})));
  Modelica.Thermal.FluidHeatFlow.Interfaces.FlowPort_b flowPort_b1
    annotation (Placement(transformation(extent={{70,-110},{90,-90}})));
protected
  parameter Real rpm=from_rpm(wref) "Reference speed of the generator";
equation
  connect(plug_p1, linearPSM.plug_p)
    annotation (Line(points={{-100,0},{-10,0}}, color={0,0,255}));
  connect(flange1, linearPSM.flange)
    annotation (Line(points={{100,0},{10,0}}, color={0,0,0}));
  connect(linearPSM.electricDriveBus, electricDriveBus1) annotation (Line(
      points={{0,10},{0,100}},
      color={0,86,166},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(linearPSM.thermalPort, machineThermal.thermalPort)
    annotation (Line(points={{6,-10},{6,-44}}, color={191,0,0}));
  connect(machineThermal.flange, linearPSM.flange)
    annotation (Line(points={{16,-54},{36,-54},{36,0},{10,0}}, color={0,0,0}));
  connect(machineThermal.flowPort_a, flowPort_a1)
    annotation (Line(points={{-2,-64},{-2,-100},{-80,-100}}, color={255,0,0}));
  connect(machineThermal.flowPort_b, flowPort_b1)
    annotation (Line(points={{14,-64},{14,-100},{80,-100}}, color={255,0,0}));
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
end ThreePhaseMotor_Cooled;
