within ;
package CHEETA "CHEETA Project"
  package Examples

  end Examples;

  package Aircraft
    package Electrical
      package EnergyManagementSystem "Models for the EMS included here"
      end EnergyManagementSystem;

      package HTS "Models for the high temperature superconductor"

        model HTS_firstorder "First order model for HTS cable"
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end HTS_firstorder;
      end HTS;

      package CB "Circuit breaker model"
        model Breaker_simple
          "Simple circuit breaker with time or signal control"
          parameter Boolean enableTrigger=false "=true, if external tigger signal is used"
            annotation (Evaluate=true, choices(checkBox=true));
          parameter Modelica.SIunits.Time t_o=Modelica.Constants.inf "Opening time"
            annotation (Dialog(enable=not enableTrigger));
          parameter Boolean rc_enabled=false "Enable reclosure" annotation (
            Evaluate=true,
            choices(checkBox=true),
            Dialog(enable=not enableTrigger));
          parameter Modelica.SIunits.Time t_rc=Modelica.Constants.inf "Reclosing time"
             annotation (Dialog(enable=not enableTrigger and rc_enabled));
          Modelica.Blocks.Interfaces.BooleanInput Trigger if enableTrigger annotation (Placement(transformation(
                extent={{-20,-20},{20,20}},
                rotation=270,
                origin={0,120})));

          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
            annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
            annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        protected
          Modelica.Blocks.Interfaces.BooleanOutput Open "Help variable to indicate open circuit breaker" annotation (Placement(transformation(extent={{-4,-10},{16,10}})));

        equation
          if not enableTrigger then
            if not rc_enabled and time >= t_o then
              Open = true;
            elseif rc_enabled and time >= t_o and time < t_rc then
              Open = true;
            else
              Open = false;
            end if;
          end if;

          if Open then
            pin_p.i = 0;
            pin_n.i = 0;
          else
            pin_p.v = pin_n.v;
            pin_p.i = -pin_p.i;
          end if;

          connect(Trigger, Open) annotation (Line(points={{0,120},{0,60},{0,0},{6,0}}, color={255,0,255}));
          annotation (
            Icon(graphics={Rectangle(
                  extent={{-40,40},{40,-40}},
                  lineThickness=0.5,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  pattern=LinePattern.None), Ellipse(
                  extent={{-100,100},{100,-100}},
                  lineColor={28,108,200},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Line(points={{-90,2},{-44,2}}, color={0,0,255}),
                Ellipse(extent={{-44,6},{-36,-2}}, lineColor={0,0,255}),
                Line(points={{-37,4},{40,42}}, color={0,0,255}),
                Line(points={{40,2},{90,2}}, color={0,0,255}),
                Line(points={{40,22},{40,2}}, color={0,0,255}),
                Text(
                  extent={{-150,90},{150,50}},
                  textString="%name",
                  lineColor={0,0,255})}),
            Documentation(info="<html>
<p>This is an <b>opening</b> circuit breaker which can either be parametrised with an opening and closing time or controlled via an external trigger. If the external trigger is active (i.e.,  <span style=\"font-family: monospace;\">Trigger=true)</span> then the circuit breaker is open.</p>
</html>"));
        end Breaker_simple;
      end CB;

      package FuelCell
      end FuelCell;

      package Battery
        model Battery "Battery with SOC output"
          extends Modelon.Electrical.EnergyStorage.Interfaces.Base;
          extends Modelon.Thermal.HeatTransfer.Interfaces.ConditionalHeatPort;

          replaceable
            Modelon.Electrical.EnergyStorage.Components.BatteryPackEMFModShepherd
            stackVoltage(final SOC_start=SOC_start) constrainedby
            Modelon.Electrical.EnergyStorage.Components.EMFInterface(SOC_start=SOC_start) annotation (choicesAllMatching=true, Placement(transformation(
                extent={{30,30},{-10,-10}},
                origin={30,-10})));
          Modelica.Electrical.Analog.Basic.Resistor internalResistance(R=R_int,
            useHeatPort=enable_heatport,
            T_ref=T0,
            alpha=0,
            T=T0)
            annotation (Placement(transformation(extent={{0,-20},{-40,20}})));
          Modelica.Blocks.Interfaces.RealOutput SOC "State of Charge" annotation (
                Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  origin={110,0})));

          parameter Real V_min=0
            "Minimum allowable terminal voltage (simulation terminated if below)";

          Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
            annotation (Placement(transformation(extent={{-20,-60},{-40,-40}})));
          Modelon.Blocks.Terminate terminate(message=
                "Battery voltage to low, this may be due to a too high current")
            annotation (Placement(transformation(extent={{10,-80},{30,-60}})));
          Modelica.Blocks.Logical.LessThreshold voltageLimit(threshold=V_min)
            annotation (Placement(transformation(extent={{-20,-80},{0,-60}})));

          final parameter Modelica.SIunits.Mass m_battery = stackVoltage.ns*stackVoltage.np*stackVoltage.cellInfo.m_cell
            "Battery mass, calculated from cell mass";

          final parameter Modelica.SIunits.Resistance R_int = (stackVoltage.ns/stackVoltage.np)*stackVoltage.cellInfo.R_cell
            "Battery internal resistance, calculated from cell resistance and number of series/parallel cells";

          parameter Real SOC_start=0.6 "Initial SOC";
        equation
          connect(internalResistance.p, stackVoltage.n)
                                              annotation (Line(
              points={{0,0},{20,0}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(voltageSensor.p, stackVoltage.p) annotation (Line(
              points={{-20,-50},{60,-50},{60,0}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(voltageSensor.n, internalResistance.n) annotation (Line(
              points={{-40,-50},{-60,-50},{-60,0},{-40,0}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(voltageLimit.y,terminate. u) annotation (Line(
              points={{1,-70},{9,-70}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(voltageLimit.u,voltageSensor. v) annotation (Line(
              points={{-22,-70},{-30,-70},{-30,-61}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(stackVoltage.SoC, SOC) annotation (Line(
              points={{40,-14},{40,-20},{80,-20},{80,0},{110,0}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(pin_p, stackVoltage.p) annotation (Line(
              points={{40,100},{40,60},{60,60},{60,0}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(internalResistance.n, pin_n) annotation (Line(
              points={{-40,0},{-60,0},{-60,60},{-40,60},{-40,100}},
              color={0,0,255},
              smooth=Smooth.None));

          connect(internalResistance.heatPort, heatPort) annotation (Line(
              points={{-20,-20},{-20,-30},{100,-30},{100,-100}},
              color={191,0,0},
              smooth=Smooth.None));
         annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}})),                Icon(coordinateSystem(
                  preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
                graphics={
                Rectangle(
                  extent={{-80,60},{80,-80}},
                  lineColor={95,95,95},
                  fillPattern=FillPattern.Solid,
                  fillColor={215,215,215}),
                Line(
                  points={{-50,90},{-50,60}},
                  color={95,95,95},
                  smooth=Smooth.None),
                Line(
                  points={{50,90},{50,60}},
                  color={95,95,95},
                  smooth=Smooth.None),
                Text(
                  extent={{90,60},{50,100}},
                  lineColor={95,95,95},
                  textString="+"),
                Text(
                  extent={{-50,60},{-90,100}},
                  lineColor={95,95,95},
                  textString="-"),
                Line(
                  points={{-60,-58},{-60,22}},
                  color={95,95,95},
                  smooth=Smooth.None),
                Line(
                  points={{-40,-58},{-40,22}},
                  color={95,95,95},
                  smooth=Smooth.None),
                Line(
                  points={{-20,-58},{-20,22}},
                  color={95,95,95},
                  smooth=Smooth.None),
                Line(
                  points={{0,-58},{0,22}},
                  color={95,95,95},
                  smooth=Smooth.None),
                Line(
                  points={{20,-58},{20,22}},
                  color={95,95,95},
                  smooth=Smooth.None),
                Line(
                  points={{40,-58},{40,22}},
                  color={95,95,95},
                  smooth=Smooth.None),
                Line(
                  points={{60,-58},{60,22}},
                  color={95,95,95},
                  smooth=Smooth.None)}),
            Documentation(info="<html>
<p>The Battery model uses BatteryPackEMF to determine SoC-dependent or charge-dependent voltage. The total internal resistance is modeled as a series resistance, the value of which is determined by the cell internal resistance given by the cellInfo parameter, as well as the number and arrangement (series/parallel) of cells in the battery pack. </p>
<p><br>The user can select between three different EMF voltage models, Proportional, Shepherd and Extended Shepherd. The discharge voltage and SoC curves are visible below.</p>
<p><br>Voltage as a function of time (SoC)</p>
<p><br><img src=\"modelica://Modelon/Resources/Images/Electrical/EnergyStorage/VoltagePlot.png\"/></p>
<p><br>StateOf Charge during a discharge cycle.</p>
<p><br><img src=\"modelica://Modelon/Resources/Images/Electrical/EnergyStorage/SoC.png\"/></p>
<p><br>Note that SoC can take values below 0 in the proportional model. This model will be removed in the future.</p>
<p>A voltage sensor is used to monitor terminal voltage. Simulation is terminated if the terminal voltage falls below a given value. </p>
</html>"));
        end Battery;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-80,60},{80,-80}},
                lineColor={95,95,95},
                fillPattern=FillPattern.Solid,
                fillColor={215,215,215}),
              Line(
                points={{-50,90},{-50,60}},
                color={95,95,95},
                smooth=Smooth.None),
              Line(
                points={{50,90},{50,60}},
                color={95,95,95},
                smooth=Smooth.None),
              Text(
                extent={{90,60},{50,100}},
                lineColor={95,95,95},
                textString="+"),
              Text(
                extent={{-50,60},{-90,100}},
                lineColor={95,95,95},
                textString="-"),
              Line(
                points={{-60,-58},{-60,22}},
                color={95,95,95},
                smooth=Smooth.None),
              Line(
                points={{-40,-58},{-40,22}},
                color={95,95,95},
                smooth=Smooth.None),
              Line(
                points={{-20,-58},{-20,22}},
                color={95,95,95},
                smooth=Smooth.None),
              Line(
                points={{0,-58},{0,22}},
                color={95,95,95},
                smooth=Smooth.None),
              Line(
                points={{20,-58},{20,22}},
                color={95,95,95},
                smooth=Smooth.None),
              Line(
                points={{40,-58},{40,22}},
                color={95,95,95},
                smooth=Smooth.None),
              Line(
                points={{60,-58},{60,22}},
                color={95,95,95},
                smooth=Smooth.None)}));
      end Battery;

      package Interfaces
        connector PwPin
          "Connector for electrical blocks treating voltage and current as complex variables"
          Real vr "Real part of the voltage";
          Real vi "Imaginary part of the voltage";
          flow Real ir "Real part of the current";
          flow Real ii "Imaginary part of the current";
          annotation (
            Icon(graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={0,0,255},
                  fillPattern=FillPattern.Solid)}),
            Diagram(graphics={Text(
                  extent={{-100,160},{100,120}},
                  lineColor={0,0,255},
                  textString="%name"),Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={0,0,255},
                  fillPattern=FillPattern.Solid)}),
            Documentation);
        end PwPin;
        annotation (Icon(graphics={
              Rectangle(
                lineColor={200,200,200},
                fillColor={248,248,248},
                fillPattern=FillPattern.HorizontalCylinder,
                extent={{-100.0,-100.0},{100.0,100.0}},
                radius=25.0),
              Rectangle(
                lineColor={128,128,128},
                extent={{-100.0,-100.0},{100.0,100.0}},
                radius=25.0),
              Polygon(origin={20.0,0.0},
                lineColor={64,64,64},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                points={{-10.0,70.0},{10.0,70.0},{40.0,20.0},{80.0,20.0},{80.0,-20.0},{40.0,-20.0},{10.0,-70.0},{-10.0,-70.0}}),
              Polygon(fillColor={102,102,102},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-100.0,20.0},{-60.0,20.0},{-30.0,70.0},{-10.0,70.0},{-10.0,-70.0},{-30.0,-70.0},{-60.0,-20.0},{-100.0,-20.0}})}));
      end Interfaces;
    end Electrical;

    package Mechanical
      package Cooling "Models for the cooling system"
      end Cooling;
    end Mechanical;

    package Thermal
      package Cooling
      end Cooling;
    end Thermal;

    package Fluids
      package LH2_Storage "Models and components for the LH2 storage"
      end LH2_Storage;
    end Fluids;

    package Airframes
      package Interfaces

      end Interfaces;

      package Wings
      end Wings;

      package Fuselages
      end Fuselages;

      package Tails
      end Tails;

      package LandingGears
      end LandingGears;

      package Brakes
      end Brakes;

      package Aerodynamics
      end Aerodynamics;
    end Airframes;
  end Aircraft;

  package Architecture
    model Aircraft_Overview
      Electrical.CB.CircuitBreaker CB
        annotation (Placement(transformation(extent={{-50,-24},{-30,-4}})));
      Electrical.HTS.HTS HTS
        annotation (Placement(transformation(extent={{-14,-18},{6,2}})));
      Electrical.CB.CircuitBreaker CB1
        annotation (Placement(transformation(extent={{28,-24},{48,-4}})));
      Electrical.FuelCell.FuelCell fuelCell
        annotation (Placement(transformation(extent={{-132,-18},{-112,2}})));
      Mechanical.Cooling.FanCooling Fan
        annotation (Placement(transformation(extent={{6,40},{-14,60}})));
      Fluids.LH2_Storage.LH2 LH2
        annotation (Placement(transformation(extent={{-14,12},{6,32}})));
      Electrical.FuelCell.Battery battery
        annotation (Placement(transformation(extent={{-112,28},{-132,8}})));
      Electrical.EnergyManagementSystem.EMS EMS
        annotation (Placement(transformation(extent={{-88,-20},{-68,0}})));
      Electrical.MasterControls.MasterControl masterControl
        annotation (Placement(transformation(extent={{-104,36},{-84,56}})));
      Electrical.MasterControls.Pilot pilot
        annotation (Placement(transformation(extent={{-136,56},{-116,76}})));
      Electrical.MasterControls.Cabin cabin
        annotation (Placement(transformation(extent={{-164,36},{-144,56}})));
      inner replaceable AircraftDynamics.Atmospheres.Interfaces.Base atmosphere
        annotation (Placement(transformation(extent={{-76,58},{-36,78}})));
    equation
      connect(CB.pin_n, HTS.pin_p)
        annotation (Line(points={{-30,-14},{-22,-14},{-22,-9.8},{-14,-9.8}},
                                                   color={0,0,255}));
      connect(Fan.port_a, HTS.port_b) annotation (Line(points={{6,46},{14,46},{
              14,0},{-14,0}}, color={191,0,0}));
      connect(LH2.port_b, HTS.port_b) annotation (Line(points={{6,26},{14,26},{
              14,0},{-14,0}}, color={191,0,0}));
      connect(LH2.port_a, HTS.port_a) annotation (Line(points={{-14,26},{-22,26},
              {-22,-5.2},{-14,-5.2}},
                                  color={191,0,0}));
      connect(Fan.port_b, HTS.port_a) annotation (Line(points={{-14,46},{-22,46},
              {-22,-5.2},{-14,-5.2}},
                                  color={191,0,0}));
      connect(Fan.pin_p, CB1.pin_n) annotation (Line(points={{6,56},{58,56},{58,
              -14},{48,-14}},
                          color={0,0,255}));
      connect(Fan.pin_n, CB1.pin_p) annotation (Line(points={{-14,50},{-22,50},
              {-22,64},{64,64},{64,-28},{22,-28},{22,-8},{28,-8}},
                                                                 color={0,0,255}));
      connect(battery.port_b, fuelCell.port_a) annotation (Line(points={{-111,
              10.8},{-134,10.8},{-134,-5.2},{-132,-5.2}},
                                                  color={191,0,0}));
      connect(battery.pin_n, fuelCell.pin_p) annotation (Line(points={{-111,
              25.4},{-136,25.4},{-136,-13.8},{-132,-13.8}},
                                            color={0,0,255}));
      connect(battery.pin_p, fuelCell.pin_n) annotation (Line(points={{-111,
              20.4},{-108,20.4},{-108,-16.4},{-132,-16.4}},
                                            color={0,0,255}));
      connect(CB.pin_p, EMS.pin_n)
        annotation (Line(points={{-50,-8},{-60,-8},{-60,-13},{-68,-13}},
                                                   color={0,0,255}));
      connect(EMS.pin_p, fuelCell.pin_n)
        annotation (Line(points={{-88.2,-13},{-102,-13},{-102,-16.4},{-132,
              -16.4}},                              color={0,0,255}));
      connect(EMS.port_b, HTS.port_a) annotation (Line(points={{-68,-6},{-56,-6},
              {-56,-5.2},{-14,-5.2}},
                                  color={191,0,0}));
      connect(masterControl.y, EMS.u)
        annotation (Line(points={{-83,46},{-78,46},{-78,2}}, color={0,0,127}));
      connect(pilot.y, masterControl.userInput) annotation (Line(points={{-115,
              66},{-94,66},{-94,58}}, color={0,0,127}));
      connect(masterControl.Cabin, cabin.y)
        annotation (Line(points={{-106,46},{-143,46}}, color={0,0,127}));
      connect(EMS.y, cabin.u) annotation (Line(points={{-78,-21},{-78,-24},{
              -174,-24},{-174,46},{-166,46}}, color={0,0,127}));
      connect(fuelCell.port_b, battery.port_a) annotation (Line(points={{-132,0},
              {-132,6},{-111,6},{-111,15.4}}, color={191,0,0}));
      connect(fuelCell.port_b1, EMS.port_a) annotation (Line(points={{-112,-10},
              {-100,-10},{-100,-6},{-88,-6}}, color={191,0,0}));
      connect(fuelCell.pin_n1, fuelCell.pin_n) annotation (Line(points={{-112,
              -6},{-108,-6},{-108,-16.4},{-132,-16.4}}, color={0,0,255}));
      connect(HTS.pin_n, CB1.pin_n1) annotation (Line(points={{-14,-15.4},{18,
              -15.4},{18,-20},{28,-20}}, color={0,0,255}));
      connect(CB.pin_n1, fuelCell.pin_n) annotation (Line(points={{-50,-20},{
              -56,-20},{-56,-34},{-100,-34},{-100,-16.4},{-132,-16.4}}, color={
              0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
                -100},{100,100}})),                                  Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-240,-100},{
                100,100}})));
    end Aircraft_Overview;

    model Aircraft_Overview2
      Electrical.CB.CircuitBreaker CB
        annotation (Placement(transformation(extent={{-50,-24},{-30,-4}})));
      Electrical.HTS.HTS HTS
        annotation (Placement(transformation(extent={{-14,-18},{6,2}})));
      Electrical.CB.CircuitBreaker CB1
        annotation (Placement(transformation(extent={{28,-24},{48,-4}})));
      Electrical.FuelCell.FuelCell fuelCell
        annotation (Placement(transformation(extent={{-132,-18},{-112,2}})));
      Mechanical.Cooling.FanCooling Fan
        annotation (Placement(transformation(extent={{6,40},{-14,60}})));
      Fluids.LH2_Storage.LH2 LH2
        annotation (Placement(transformation(extent={{-14,12},{6,32}})));
      Electrical.EnergyManagementSystem.EMS EMS
        annotation (Placement(transformation(extent={{-88,-20},{-68,0}})));
      Electrical.MasterControls.MasterControl masterControl
        annotation (Placement(transformation(extent={{-104,36},{-84,56}})));
      Electrical.MasterControls.Pilot pilot
        annotation (Placement(transformation(extent={{-136,56},{-116,76}})));
      Electrical.MasterControls.Cabin cabin
        annotation (Placement(transformation(extent={{-164,36},{-144,56}})));
      inner replaceable AircraftDynamics.Atmospheres.Interfaces.Base atmosphere
        annotation (Placement(transformation(extent={{-76,58},{-36,78}})));
      Aircraft.Electrical.Battery.Battery battery
        annotation (Placement(transformation(extent={{-132,26},{-112,6}})));
    equation
      connect(CB.pin_n, HTS.pin_p)
        annotation (Line(points={{-30,-14},{-22,-14},{-22,-9.8},{-14,-9.8}},
                                                   color={0,0,255}));
      connect(Fan.port_a, HTS.port_b) annotation (Line(points={{6,46},{14,46},{
              14,0},{-14,0}}, color={191,0,0}));
      connect(LH2.port_b, HTS.port_b) annotation (Line(points={{6,26},{14,26},{
              14,0},{-14,0}}, color={191,0,0}));
      connect(LH2.port_a, HTS.port_a) annotation (Line(points={{-14,26},{-22,26},
              {-22,-5.2},{-14,-5.2}},
                                  color={191,0,0}));
      connect(Fan.port_b, HTS.port_a) annotation (Line(points={{-14,46},{-22,46},
              {-22,-5.2},{-14,-5.2}},
                                  color={191,0,0}));
      connect(Fan.pin_p, CB1.pin_n) annotation (Line(points={{6,56},{58,56},{58,
              -14},{48,-14}},
                          color={0,0,255}));
      connect(Fan.pin_n, CB1.pin_p) annotation (Line(points={{-14,50},{-22,50},
              {-22,64},{64,64},{64,-28},{22,-28},{22,-8},{28,-8}},
                                                                 color={0,0,255}));
      connect(CB.pin_p, EMS.pin_n)
        annotation (Line(points={{-50,-8},{-60,-8},{-60,-13},{-68,-13}},
                                                   color={0,0,255}));
      connect(EMS.pin_p, fuelCell.pin_n)
        annotation (Line(points={{-88.2,-13},{-102,-13},{-102,-16.4},{-132,
              -16.4}},                              color={0,0,255}));
      connect(EMS.port_b, HTS.port_a) annotation (Line(points={{-68,-6},{-56,-6},
              {-56,-5.2},{-14,-5.2}},
                                  color={191,0,0}));
      connect(masterControl.y, EMS.u)
        annotation (Line(points={{-83,46},{-78,46},{-78,2}}, color={0,0,127}));
      connect(pilot.y, masterControl.userInput) annotation (Line(points={{-115,
              66},{-94,66},{-94,58}}, color={0,0,127}));
      connect(masterControl.Cabin, cabin.y)
        annotation (Line(points={{-106,46},{-143,46}}, color={0,0,127}));
      connect(EMS.y, cabin.u) annotation (Line(points={{-78,-21},{-78,-24},{
              -174,-24},{-174,46},{-166,46}}, color={0,0,127}));
      connect(fuelCell.port_b1, EMS.port_a) annotation (Line(points={{-112,-10},
              {-100,-10},{-100,-6},{-88,-6}}, color={191,0,0}));
      connect(fuelCell.pin_n1, fuelCell.pin_n) annotation (Line(points={{-112,
              -6},{-108,-6},{-108,-16.4},{-132,-16.4}}, color={0,0,255}));
      connect(HTS.pin_n, CB1.pin_n1) annotation (Line(points={{-14,-15.4},{18,
              -15.4},{18,-20},{28,-20}}, color={0,0,255}));
      connect(CB.pin_n1, fuelCell.pin_n) annotation (Line(points={{-50,-20},{
              -56,-20},{-56,-34},{-100,-34},{-100,-16.4},{-132,-16.4}}, color={
              0,0,255}));
      connect(battery.pin_n, fuelCell.pin_p) annotation (Line(points={{-126,6},
              {-142,6},{-142,-13.8},{-132,-13.8}}, color={0,0,255}));
      connect(battery.pin_p, fuelCell.pin_n) annotation (Line(points={{-118,6},
              {-108,6},{-108,-16.4},{-132,-16.4}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
                -100},{100,100}})),                                  Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-240,-100},{
                100,100}})));
    end Aircraft_Overview2;

    package Electrical
      package EnergyManagementSystem "Models for the EMS included here"

        model EMS
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
            annotation (Placement(transformation(extent={{90,-40},{110,-20}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
              Placement(transformation(extent={{-112,-40},{-92,-20}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
            annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b
            annotation (Placement(transformation(extent={{90,30},{110,50}})));
          Modelica.Blocks.Interfaces.RealInput u annotation (Placement(
                transformation(
                extent={{-20,-20},{20,20}},
                rotation=270,
                origin={0,120})));
          Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(
                transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={0,-110})));
          EMSControl eMSControl annotation (Placement(transformation(
                extent={{-16,-15},{16,15}},
                rotation=270,
                origin={1,-68})));
          Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor
            temperatureSensor annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={4,-4})));
          Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heatFlowSensor
            annotation (Placement(transformation(extent={{10,30},{30,50}})));
          Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
            annotation (Placement(transformation(extent={{-18,-26},{-38,-6}})));
          Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
            annotation (Placement(transformation(extent={{-52,-40},{-32,-20}})));
        equation
          connect(y, eMSControl.y) annotation (Line(points={{0,-110},{0,-85.6},
                  {0.4,-85.6}}, color={0,0,127}));
          connect(port_a, heatFlowSensor.port_a)
            annotation (Line(points={{-100,40},{10,40}}, color={191,0,0}));
          connect(port_b, heatFlowSensor.port_b)
            annotation (Line(points={{100,40},{30,40}}, color={191,0,0}));
          connect(eMSControl.Temperature_in, heatFlowSensor.Q_flow) annotation (
             Line(points={{13,-48.8},{12,-48.8},{12,-26},{20,-26},{20,30}},
                color={0,0,127}));
          connect(temperatureSensor.port, heatFlowSensor.port_a)
            annotation (Line(points={{4,6},{4,40},{10,40}}, color={191,0,0}));
          connect(eMSControl.Q_flow, temperatureSensor.T) annotation (Line(
                points={{4.9,-48.8},{4,-48.8},{4,-14}}, color={0,0,127}));
          connect(pin_p, voltageSensor.n) annotation (Line(points={{-102,-30},{
                  -58,-30},{-58,-16},{-38,-16}}, color={0,0,255}));
          connect(pin_n, voltageSensor.p) annotation (Line(points={{100,-30},{
                  54,-30},{54,-16},{-18,-16}}, color={0,0,255}));
          connect(eMSControl.V_in, voltageSensor.v) annotation (Line(points={{
                  -2.9,-48.8},{-2,-48.8},{-2,-38},{-28,-38},{-28,-27}}, color={
                  0,0,127}));
          connect(pin_p, pin_p)
            annotation (Line(points={{-102,-30},{-102,-30}}, color={0,0,255}));
          connect(eMSControl.I_in, currentSensor.i) annotation (Line(points={{
                  -11,-48.8},{-12,-48.8},{-12,-44},{-42,-44},{-42,-41}}, color=
                  {0,0,127}));
          connect(currentSensor.n, voltageSensor.p) annotation (Line(points={{
                  -32,-30},{54,-30},{54,-16},{-18,-16}}, color={0,0,255}));
          connect(pin_p, currentSensor.p)
            annotation (Line(points={{-102,-30},{-52,-30}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={28,108,200},
                  fillPattern=FillPattern.Solid), Text(
                  extent={{-96,38},{104,-94}},
                  lineColor={255,255,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid,
                  textString="EMS
")}),         Diagram(coordinateSystem(preserveAspectRatio=false)));
        end EMS;

        model EMSControl
          Modelica.Blocks.Interfaces.RealInput V_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,-46},{-100,-6}})));
          Modelica.Blocks.Interfaces.RealInput I_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
          Modelica.Blocks.Interfaces.RealInput Temperature_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
          Modelica.Blocks.Interfaces.RealInput Q_flow "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,6},{-100,46}})));
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-14},{120,6}})));
        equation
          connect(y, y)
            annotation (Line(points={{110,-4},{110,-4}}, color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end EMSControl;
      end EnergyManagementSystem;

      package HTS "Models for the high temperature superconductor"

        model HTS
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
            annotation (Placement(transformation(extent={{-110,18},{-90,38}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b
            annotation (Placement(transformation(extent={{-110,70},{-90,90}})));
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
            annotation (Placement(transformation(extent={{-110,-84},{-90,-64}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
              Placement(transformation(extent={{-110,-28},{-90,-8}})));
          Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heatFlowSensor
            annotation (Placement(transformation(extent={{-68,50},{-48,70}})));
          Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor
            temperatureSensor
            annotation (Placement(transformation(extent={{-68,18},{-48,38}})));
          Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
            annotation (Placement(transformation(extent={{-54,-8},{-34,-28}})));
          Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
            annotation (Placement(transformation(extent={{-54,-48},{-34,-68}})));
          HTSControls hTSControls
            annotation (Placement(transformation(extent={{8,-4},{44,30}})));
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n1
            annotation (Placement(transformation(extent={{90,4},{110,24}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b1
            annotation (Placement(transformation(extent={{90,-32},{110,-12}})));
        equation
          connect(port_a, heatFlowSensor.port_a) annotation (Line(points={{-100,
                  28},{-80,28},{-80,60},{-68,60}}, color={191,0,0}));
          connect(port_b, heatFlowSensor.port_b) annotation (Line(points={{-100,
                  80},{-40,80},{-40,60},{-48,60}}, color={191,0,0}));
          connect(temperatureSensor.port, heatFlowSensor.port_a) annotation (
              Line(points={{-68,28},{-80,28},{-80,60},{-68,60}}, color={191,0,0}));
          connect(pin_p, voltageSensor.p)
            annotation (Line(points={{-100,-18},{-54,-18}}, color={0,0,255}));
          connect(pin_n, voltageSensor.n) annotation (Line(points={{-100,-74},{
                  -24,-74},{-24,-18},{-34,-18}}, color={0,0,255}));
          connect(currentSensor.p, voltageSensor.p) annotation (Line(points={{
                  -54,-58},{-64,-58},{-64,-18},{-54,-18}}, color={0,0,255}));
          connect(currentSensor.n, voltageSensor.n) annotation (Line(points={{
                  -34,-58},{-24,-58},{-24,-18},{-34,-18}}, color={0,0,255}));
          connect(hTSControls.Temperature_in, heatFlowSensor.Q_flow)
            annotation (Line(points={{4.4,26.6},{-18,26.6},{-18,40},{-58,40},{
                  -58,50}}, color={0,0,127}));
          connect(temperatureSensor.T, hTSControls.Q_flow) annotation (Line(
                points={{-48,28},{-22,28},{-22,17.42},{4.4,17.42}}, color={0,0,
                  127}));
          connect(hTSControls.V_in, voltageSensor.v) annotation (Line(points={{
                  4.4,8.58},{-44,8.58},{-44,-7}}, color={0,0,127}));
          connect(hTSControls.I_in, currentSensor.i) annotation (Line(points={{
                  4.4,-0.6},{-14,-0.6},{-14,-36},{-44,-36},{-44,-47}}, color={0,
                  0,127}));
          connect(hTSControls.port_a, port_b1) annotation (Line(points={{44,6.2},
                  {64,6.2},{64,-22},{100,-22}}, color={191,0,0}));
          connect(hTSControls.pin_p, pin_n1) annotation (Line(points={{44,19.8},
                  {64,19.8},{64,14},{100,14}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={28,108,200},
                  fillPattern=FillPattern.Solid), Text(
                  extent={{-84,34},{80,-30}},
                  lineColor={255,255,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid,
                  textString="HTS")}),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end HTS;

        model HTSControls
          Modelica.Blocks.Interfaces.RealInput V_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,-46},{-100,-6}})));
          Modelica.Blocks.Interfaces.RealInput I_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
          Modelica.Blocks.Interfaces.RealInput Temperature_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
          Modelica.Blocks.Interfaces.RealInput Q_flow "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,6},{-100,46}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
            annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
              Placement(transformation(extent={{90,30},{110,50}})));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end HTSControls;
      end HTS;

      package CB "Circuit breaker model"

        model CircuitBreaker
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
            annotation (Placement(transformation(extent={{90,-10},{110,10}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
              Placement(transformation(extent={{-110,50},{-90,70}})));
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n1
            annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
          CBControls cBControls
            annotation (Placement(transformation(extent={{20,-10},{40,10}})));
          Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={-34,6})));
          Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={-58,-20})));
        equation
          connect(pin_n, cBControls.pin_p)
            annotation (Line(points={{100,0},{40,0}}, color={0,0,255}));
          connect(pin_p, voltageSensor.n) annotation (Line(points={{-100,60},{
                  -34,60},{-34,16}}, color={0,0,255}));
          connect(pin_n1, voltageSensor.p) annotation (Line(points={{-100,-60},
                  {-34,-60},{-34,-4}}, color={0,0,255}));
          connect(cBControls.V_in, voltageSensor.v)
            annotation (Line(points={{18,6},{-23,6}}, color={0,0,127}));
          connect(pin_n1, currentSensor.p) annotation (Line(points={{-100,-60},
                  {-58,-60},{-58,-30}}, color={0,0,255}));
          connect(pin_p, currentSensor.n) annotation (Line(points={{-100,60},{
                  -58,60},{-58,-10}}, color={0,0,255}));
          connect(currentSensor.i, cBControls.I_in) annotation (Line(points={{
                  -47,-20},{-14,-20},{-14,-6},{18,-6}}, color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={28,108,200},
                  fillPattern=FillPattern.Solid), Text(
                  extent={{-98,46},{92,-46}},
                  lineColor={255,255,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid,
                  textString="Circuit 
Breaker")}),  Diagram(coordinateSystem(preserveAspectRatio=false)));
        end CircuitBreaker;

        model CBControls
          Modelica.Blocks.Interfaces.RealInput V_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
          Modelica.Blocks.Interfaces.RealInput I_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
              Placement(transformation(extent={{90,-10},{110,10}})));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end CBControls;
      end CB;

      package FuelCell
        model FuelCell
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
            annotation (Placement(transformation(extent={{-110,18},{-90,38}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b
            annotation (Placement(transformation(extent={{-110,70},{-90,90}})));
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
            annotation (Placement(transformation(extent={{-110,-94},{-90,-74}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
              Placement(transformation(extent={{-110,-68},{-90,-48}})));
          FuelCellControls fuelCellControls
            annotation (Placement(transformation(extent={{-8,-8},{26,22}})));
          Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heatFlowSensor
            annotation (Placement(transformation(extent={{-68,50},{-48,70}})));
          Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor
            temperatureSensor
            annotation (Placement(transformation(extent={{-68,18},{-48,38}})));
          Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
            annotation (Placement(transformation(extent={{-54,-8},{-34,-28}})));
          Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
            annotation (Placement(transformation(extent={{-54,-48},{-34,-68}})));
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n1
            annotation (Placement(transformation(extent={{90,10},{110,30}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b1
            annotation (Placement(transformation(extent={{90,-30},{110,-10}})));
        equation
          connect(port_a, heatFlowSensor.port_a) annotation (Line(points={{-100,
                  28},{-80,28},{-80,60},{-68,60}}, color={191,0,0}));
          connect(port_b, heatFlowSensor.port_b) annotation (Line(points={{-100,
                  80},{-40,80},{-40,60},{-48,60}}, color={191,0,0}));
          connect(temperatureSensor.port, heatFlowSensor.port_a) annotation (
              Line(points={{-68,28},{-80,28},{-80,60},{-68,60}}, color={191,0,0}));
          connect(fuelCellControls.Temperature_in, heatFlowSensor.Q_flow)
            annotation (Line(points={{-11.4,19},{-36,19},{-36,44},{-58,44},{-58,
                  50}}, color={0,0,127}));
          connect(fuelCellControls.Q_flow, temperatureSensor.T) annotation (
              Line(points={{-11.4,10.9},{-42,10.9},{-42,28},{-48,28}}, color={0,
                  0,127}));
          connect(pin_p, voltageSensor.p) annotation (Line(points={{-100,-58},{
                  -64,-58},{-64,-18},{-54,-18}}, color={0,0,255}));
          connect(pin_n, voltageSensor.n) annotation (Line(points={{-100,-84},{
                  -24,-84},{-24,-18},{-34,-18}}, color={0,0,255}));
          connect(voltageSensor.v, fuelCellControls.V_in) annotation (Line(
                points={{-44,-7},{-44,3.1},{-11.4,3.1}}, color={0,0,127}));
          connect(currentSensor.p, voltageSensor.p) annotation (Line(points={{
                  -54,-58},{-64,-58},{-64,-18},{-54,-18}}, color={0,0,255}));
          connect(currentSensor.n, voltageSensor.n) annotation (Line(points={{
                  -34,-58},{-24,-58},{-24,-18},{-34,-18}}, color={0,0,255}));
          connect(currentSensor.i, fuelCellControls.I_in) annotation (Line(
                points={{-44,-47},{-44,-34},{-20,-34},{-20,-5},{-11.4,-5}},
                color={0,0,127}));
          connect(fuelCellControls.pin_p, pin_n1) annotation (Line(points={{26,
                  13},{64,13},{64,20},{100,20}}, color={0,0,255}));
          connect(fuelCellControls.port_a, port_b1) annotation (Line(points={{
                  26,1},{60,1},{60,-20},{100,-20}}, color={191,0,0}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={28,108,200},
                  fillPattern=FillPattern.Solid), Text(
                  extent={{-64,24},{70,-22}},
                  lineColor={255,255,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid,
                  textString="Fuel Cell")}),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end FuelCell;

        model Battery
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
            annotation (Placement(transformation(extent={{-120,16},{-100,36}}),
                iconTransformation(extent={{-120,16},{-100,36}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b
            annotation (Placement(transformation(extent={{-120,62},{-100,82}}),
                iconTransformation(extent={{-120,62},{-100,82}})));
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
            annotation (Placement(transformation(extent={{-120,-84},{-100,-64}}),
                iconTransformation(extent={{-120,-84},{-100,-64}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
              Placement(transformation(extent={{-120,-34},{-100,-14}}),
                iconTransformation(extent={{-120,-34},{-100,-14}})));
          BatteryControls batteryControls
            annotation (Placement(transformation(extent={{-2,-2},{18,18}})));
          Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heatFlowSensor
            annotation (Placement(transformation(extent={{-68,50},{-48,70}})));
          Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor
            temperatureSensor
            annotation (Placement(transformation(extent={{-68,18},{-48,38}})));
          Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
            annotation (Placement(transformation(extent={{-54,-8},{-34,-28}})));
          Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
            annotation (Placement(transformation(extent={{-54,-48},{-34,-68}})));
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n1
            annotation (Placement(transformation(extent={{100,40},{120,60}}),
                iconTransformation(extent={{100,40},{120,60}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b1
            annotation (Placement(transformation(extent={{100,-60},{120,-40}}),
                iconTransformation(extent={{100,-60},{120,-40}})));
        equation
          connect(port_a, heatFlowSensor.port_a) annotation (Line(points={{-110,
                  26},{-80,26},{-80,60},{-68,60}}, color={191,0,0}));
          connect(port_b, heatFlowSensor.port_b) annotation (Line(points={{-110,
                  72},{-34,72},{-34,60},{-48,60}}, color={191,0,0}));
          connect(temperatureSensor.port, heatFlowSensor.port_a) annotation (
              Line(points={{-68,28},{-80,28},{-80,60},{-68,60}}, color={191,0,0}));
          connect(pin_p, voltageSensor.p) annotation (Line(points={{-110,-24},{
                  -78,-24},{-78,-18},{-54,-18}}, color={0,0,255}));
          connect(pin_n, voltageSensor.n) annotation (Line(points={{-110,-74},{
                  -24,-74},{-24,-18},{-34,-18}}, color={0,0,255}));
          connect(currentSensor.p, voltageSensor.p) annotation (Line(points={{
                  -54,-58},{-64,-58},{-64,-18},{-54,-18}}, color={0,0,255}));
          connect(currentSensor.n, voltageSensor.n) annotation (Line(points={{
                  -34,-58},{-24,-58},{-24,-18},{-34,-18}}, color={0,0,255}));
          connect(batteryControls.pin_p, pin_n1) annotation (Line(points={{18,
                  12},{58,12},{58,50},{110,50}}, color={0,0,255}));
          connect(batteryControls.port_a, port_b1) annotation (Line(points={{18,
                  4},{60,4},{60,-50},{110,-50}}, color={191,0,0}));
          connect(voltageSensor.v, batteryControls.V_in) annotation (Line(
                points={{-44,-7},{-44,5.4},{-4,5.4}}, color={0,0,127}));
          connect(batteryControls.I_in, currentSensor.i) annotation (Line(
                points={{-4,0},{-10,0},{-10,-24},{-16,-24},{-16,-34},{-44,-34},
                  {-44,-47}}, color={0,0,127}));
          connect(temperatureSensor.T, batteryControls.Q_flow) annotation (Line(
                points={{-48,28},{-28,28},{-28,10.6},{-4,10.6}}, color={0,0,127}));
          connect(heatFlowSensor.Q_flow, batteryControls.Temperature_in)
            annotation (Line(points={{-58,50},{-58,44},{-24,44},{-24,16},{-4,16}},
                color={0,0,127}));
          connect(pin_n, pin_n) annotation (Line(points={{-110,-74},{-98,-74},{
                  -98,-74},{-110,-74}}, color={0,0,255}));
          connect(pin_p, pin_p) annotation (Line(points={{-110,-24},{-104,-24},
                  {-104,-42},{-104,-42},{-104,-24},{-110,-24}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={28,108,200},
                  fillPattern=FillPattern.Solid), Text(
                  extent={{-62,32},{64,-22}},
                  lineColor={255,255,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid,
                  textString="Battery")}),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end Battery;

        model FuelCellControls
          Modelica.Blocks.Interfaces.RealInput V_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,-46},{-100,-6}})));
          Modelica.Blocks.Interfaces.RealInput I_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
          Modelica.Blocks.Interfaces.RealInput Temperature_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
          Modelica.Blocks.Interfaces.RealInput Q_flow "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,6},{-100,46}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
            annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
              Placement(transformation(extent={{90,30},{110,50}})));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end FuelCellControls;

        model BatteryControls
          Modelica.Blocks.Interfaces.RealInput V_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,-46},{-100,-6}})));
          Modelica.Blocks.Interfaces.RealInput I_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
          Modelica.Blocks.Interfaces.RealInput Temperature_in "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
          Modelica.Blocks.Interfaces.RealInput Q_flow "Imaginary Part"
            annotation (Placement(transformation(extent={{-140,6},{-100,46}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
            annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
              Placement(transformation(extent={{90,30},{110,50}})));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end BatteryControls;
      end FuelCell;

      package MasterControls
        model MasterControl
          Modelica.Blocks.Interfaces.RealInput userInput annotation (Placement(
                transformation(
                extent={{-20,-20},{20,20}},
                rotation=270,
                origin={0,120})));
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          Modelica.Blocks.Interfaces.RealInput Cabin annotation (Placement(
                transformation(
                extent={{-20,-20},{20,20}},
                rotation=0,
                origin={-120,0})));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid), Text(
                  extent={{-100,38},{102,-36}},
                  lineColor={255,255,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid,
                  textString="Master
Control")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
        end MasterControl;

        model Pilot
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                graphics={
                Ellipse(
                  extent={{-26,72},{24,30}},
                  lineColor={0,0,0},
                  lineThickness=1),
                Line(points={{-8,48}}, color={28,108,200}),
                Line(
                  points={{0,30},{0,-26},{0,-24}},
                  color={0,0,0},
                  thickness=1),
                Line(
                  points={{0,-26},{-20,-56}},
                  color={0,0,0},
                  thickness=1),
                Line(
                  points={{6,10},{-24,-10}},
                  color={0,0,0},
                  origin={10,-32},
                  rotation=90,
                  thickness=1),
                Line(
                  points={{0,8},{26,22}},
                  color={0,0,0},
                  thickness=1),
                Line(
                  points={{0,8},{-28,22}},
                  color={0,0,0},
                  thickness=1),
                Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,255})}),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end Pilot;

        model Environment
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
                  preserveAspectRatio=false)));
        end Environment;

        model Cabin
          Modelica.Blocks.Interfaces.RealInput u annotation (Placement(
                transformation(extent={{-140,-20},{-100,20}})));
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          CabinControls cabinControls
            annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
        equation
          connect(u, cabinControls.u)
            annotation (Line(points={{-120,0},{-10,0}}, color={0,0,127}));
          connect(y, cabinControls.y)
            annotation (Line(points={{110,0},{13,0}}, color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid), Text(
                  extent={{-60,20},{50,-16}},
                  lineColor={255,255,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid,
                  textString="Cabin")}), Diagram(coordinateSystem(
                  preserveAspectRatio=false), graphics={Text(
                  extent={{-66,28},{60,-26}},
                  lineColor={255,255,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid,
                  textString="Cabin")}));
        end Cabin;

        model CabinControls
          Modelica.Blocks.Interfaces.RealInput u annotation (Placement(
                transformation(extent={{-140,-20},{-100,20}})));
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false), graphics={
                  Text(
                  extent={{-66,28},{60,-26}},
                  lineColor={255,255,255},
                  fillColor={179,31,36},
                  fillPattern=FillPattern.Solid,
                  textString="Cabin")}));
        end CabinControls;
      end MasterControls;
    end Electrical;

    package Mechanical
      package Cooling "Models for the cooling system"
        model FanCooling
          Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a
            annotation (Placement(transformation(extent={{-110,-90},{-90,-70}})));
          Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b
            annotation (Placement(transformation(extent={{90,30},{110,50}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
            annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b
            annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
            annotation (Placement(transformation(extent={{90,-10},{110,10}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
              Placement(transformation(extent={{-110,50},{-90,70}})));
          FanControls fanControls
            annotation (Placement(transformation(extent={{16,-18},{62,24}})));
          Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
            annotation (Placement(transformation(
                extent={{-10,10},{10,-10}},
                rotation=270,
                origin={-74,14})));
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n1
            annotation (Placement(transformation(extent={{-114,-10},{-94,10}})));
          Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor
            temperatureSensor annotation (Placement(transformation(extent={{-32,
                    -30},{-12,-10}})));
          Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
            annotation (Placement(transformation(extent={{-12,-70},{8,-50}})));
          Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
            annotation (Placement(transformation(
                extent={{-10,10},{10,-10}},
                rotation=270,
                origin={-54,30})));
        equation
          connect(pin_n, fanControls.pin_p) annotation (Line(points={{100,0},{
                  80,0},{80,15.6},{62.92,15.6}}, color={0,0,255}));
          connect(port_b, fanControls.port_a) annotation (Line(points={{100,-40},
                  {80,-40},{80,-9.6},{62.92,-9.6}}, color={191,0,0}));
          connect(fanControls.flange_a, flange_b) annotation (Line(points={{39,
                  24},{39,40},{100,40}}, color={0,0,0}));
          connect(pin_p, voltageSensor.p) annotation (Line(points={{-100,60},{
                  -74,60},{-74,24}}, color={0,0,255}));
          connect(pin_n1, voltageSensor.n) annotation (Line(points={{-104,0},{
                  -74,0},{-74,4}}, color={0,0,255}));
          connect(voltageSensor.v, fanControls.u) annotation (Line(points={{-63,
                  14},{-24,14},{-24,3},{11.4,3}}, color={0,0,127}));
          connect(port_a, temperatureSensor.port) annotation (Line(points={{
                  -100,-40},{-66,-40},{-66,-20},{-32,-20}}, color={191,0,0}));
          connect(fanControls.u1, temperatureSensor.T) annotation (Line(points=
                  {{11.4,-9.6},{0,-9.6},{0,-20},{-12,-20}}, color={0,0,127}));
          connect(flange_a, speedSensor.flange) annotation (Line(points={{-100,
                  -80},{-42,-80},{-42,-60},{-12,-60}}, color={0,0,0}));
          connect(fanControls.u2, speedSensor.w) annotation (Line(points={{39,
                  -22.2},{40,-22.2},{40,-60},{9,-60}}, color={0,0,127}));
          connect(pin_p, currentSensor.p) annotation (Line(points={{-100,60},{
                  -54,60},{-54,40}}, color={0,0,255}));
          connect(currentSensor.i, fanControls.u3) annotation (Line(points={{
                  -43,30},{-18,30},{-18,15.6},{11.4,15.6}}, color={0,0,127}));
          connect(currentSensor.n, voltageSensor.n) annotation (Line(points={{
                  -54,20},{-54,0},{-74,0},{-74,4}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={0,140,72},
                  fillPattern=FillPattern.Solid), Text(
                  extent={{-70,30},{68,-28}},
                  lineColor={255,255,255},
                  fillColor={0,140,72},
                  fillPattern=FillPattern.Solid,
                  textString="Fan")}),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end FanCooling;

        model FanControls
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
            annotation (Placement(transformation(extent={{94,-70},{114,-50}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
              Placement(transformation(extent={{94,50},{114,70}})));
          Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a
            annotation (Placement(transformation(extent={{-10,90},{10,110}})));
          Modelica.Blocks.Interfaces.RealInput u annotation (Placement(
                transformation(extent={{-140,-20},{-100,20}})));
          Modelica.Blocks.Interfaces.RealInput u1 annotation (Placement(
                transformation(extent={{-140,-80},{-100,-40}})));
          Modelica.Blocks.Interfaces.RealInput u2 annotation (Placement(
                transformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={0,-120})));
          Modelica.Blocks.Interfaces.RealInput u3 annotation (Placement(
                transformation(extent={{-140,40},{-100,80}})));
        equation
          connect(pin_p, pin_p)
            annotation (Line(points={{104,60},{104,60}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end FanControls;
      end Cooling;
    end Mechanical;

    package Thermal
      package Cooling
      end Cooling;
    end Thermal;

    package Fluids
      package LH2_Storage "Models and components for the LH2 storage"

        model LH2
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
            annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b
            annotation (Placement(transformation(extent={{90,30},{110,50}})));
          Modelica.Fluid.Interfaces.FluidPort_a port_a1 annotation (Placement(
                transformation(extent={{-110,-50},{-90,-30}})));
          Modelica.Fluid.Interfaces.FluidPort_b port_b1
            annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
          LH2Controls lH2Controls
            annotation (Placement(transformation(extent={{-6,-20},{14,0}})));
          Modelica.Fluid.Sensors.Temperature temperature
            annotation (Placement(transformation(extent={{-88,-26},{-68,-6}})));
          Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heatFlowSensor
            annotation (Placement(transformation(extent={{-42,30},{-22,50}})));
          Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor
            temperatureSensor
            annotation (Placement(transformation(extent={{-66,2},{-46,22}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b2
            annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        equation
          connect(port_a1, temperature.port) annotation (Line(points={{-100,-40},
                  {-78,-40},{-78,-26}}, color={0,127,255}));
          connect(lH2Controls.u1, temperature.T)
            annotation (Line(points={{-8,-16},{-71,-16}}, color={0,0,127}));
          connect(lH2Controls.port_b1, port_b1) annotation (Line(points={{14,
                  -14},{56,-14},{56,-40},{100,-40}}, color={0,127,255}));
          connect(port_a, heatFlowSensor.port_a)
            annotation (Line(points={{-100,40},{-42,40}}, color={191,0,0}));
          connect(heatFlowSensor.port_b, port_b)
            annotation (Line(points={{-22,40},{100,40}}, color={191,0,0}));
          connect(heatFlowSensor.Q_flow, lH2Controls.u3) annotation (Line(
                points={{-32,30},{-32,-4},{-8,-4}}, color={0,0,127}));
          connect(temperatureSensor.port, heatFlowSensor.port_a) annotation (
              Line(points={{-66,12},{-76,12},{-76,40},{-42,40}}, color={191,0,0}));
          connect(temperatureSensor.T, lH2Controls.u) annotation (Line(points={
                  {-46,12},{-38,12},{-38,-10},{-8,-10}}, color={0,0,127}));
          connect(port_b2, lH2Controls.port_a) annotation (Line(points={{100,0},
                  {58,0},{58,-5.2},{14.4,-5.2}}, color={191,0,0}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={170,85,255},
                  fillPattern=FillPattern.Solid), Text(
                  extent={{-102,44},{98,-32}},
                  lineColor={255,255,255},
                  fillColor={170,213,255},
                  fillPattern=FillPattern.Solid,
                  textString="LH2 
Storage")}),  Diagram(coordinateSystem(preserveAspectRatio=false)));
        end LH2;

        model LH2Controls
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
            annotation (Placement(transformation(extent={{94,38},{114,58}})));
          Modelica.Blocks.Interfaces.RealInput u annotation (Placement(
                transformation(extent={{-140,-20},{-100,20}})));
          Modelica.Blocks.Interfaces.RealInput u1 annotation (Placement(
                transformation(extent={{-140,-80},{-100,-40}})));
          Modelica.Blocks.Interfaces.RealInput u3 annotation (Placement(
                transformation(extent={{-140,40},{-100,80}})));
          Modelica.Fluid.Interfaces.FluidPort_b port_b1
            annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end LH2Controls;
      end LH2_Storage;
    end Fluids;


  end Architecture;

  package Working_Models
    package Battery

      model Battery
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
          annotation (Placement(transformation(extent={{-56,48},{-36,68}})));
        Modelica.Blocks.Sources.Constant const(k=1)
          annotation (Placement(transformation(extent={{-80,70},{-60,90}})));
        Modelon.Electrical.EnergyStorage.Battery battery1
          annotation (Placement(transformation(extent={{-58,22},{-38,42}})));
      equation
        connect(signalVoltage.v, const.y) annotation (Line(points={{-46,70},{
                -46,80},{-59,80}}, color={0,0,127}));
        connect(signalVoltage.p, battery1.pin_n) annotation (Line(points={{-56,
                58},{-54,58},{-54,42},{-52,42}}, color={0,0,255}));
        connect(signalVoltage.n, battery1.pin_p) annotation (Line(points={{-36,
                58},{-40,58},{-40,42},{-44,42}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Battery;
    end Battery;
  end Working_Models;
  annotation (uses(Modelica(version="3.2.3"), AircraftDynamics(version="1.1"),
      Modelon(version="3.3"),
      OpenIPSL(version="2.0.0-dev"),
      Complex(version="3.2.3")));
end CHEETA;
