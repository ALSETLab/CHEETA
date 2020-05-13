within ;
package CHEETA_Thermal
  package Utilities
    model Fan "Simple fan model"
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J)
        annotation (Placement(transformation(extent={{18,-10},{38,10}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a1
                        "Flange of left shaft" annotation (Placement(transformation(
              extent={{-60,-10},{-40,10}}), iconTransformation(extent={{-60,-10},{
                -40,10}})));
      parameter Modelica.SIunits.Inertia J=1 "Moment of inertia of the fan blades";
      parameter Modelica.SIunits.Angle deltaPhi=0
        "Fixed rotation of left flange with respect to right flange";
    equation
      connect(flange_a1, inertia.flange_a)
        annotation (Line(points={{-50,0},{18,0}}, color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-40},
                {40,40}}), graphics={
            Rectangle(extent={{-40,40},{40,-40}}, lineColor={28,108,200}),
            Polygon(
              points={{0,-4},{4,16},{6,26},{0,32},{-6,26},{-4,16},{0,-4}},
              lineColor={28,108,200},
              smooth=Smooth.Bezier,
              fillColor={175,175,175},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{0,-18},{4,2},{6,12},{0,18},{-6,12},{-4,2},{0,-18}},
              lineColor={28,108,200},
              smooth=Smooth.Bezier,
              origin={-14,0},
              rotation=90,
              fillColor={175,175,175},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{0,-18},{4,2},{6,12},{0,18},{-6,12},{-4,2},{0,-18}},
              lineColor={28,108,200},
              smooth=Smooth.Bezier,
              origin={14,0},
              rotation=270,
              fillColor={175,175,175},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{0,-18},{4,2},{6,12},{0,18},{-6,12},{-4,2},{0,-18}},
              lineColor={28,108,200},
              smooth=Smooth.Bezier,
              origin={0,-14},
              rotation=180,
              fillColor={175,175,175},
              fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-40,-40},{40,40}})));
    end Fan;

    model DiscretizedLiquidPipe
      "Discretizable, liquid type fluid pipe with arbitrary flow cross section"
      extends
        DassaultSystemes.Fluid.FlowVolumes.Templates.BaseDiscretizedLiquidVolume(
        final Vs=fill(V*N_parallel/N, N),
        final ls_FM=fill(sum(ls)/(N_FM), N_FM),
        redeclare replaceable package Medium =
          DassaultSystemes.Media.IncompressibleLiquid.PropyleneGlycol47                                      constrainedby
          DassaultSystemes.Media.Templates.Incompressible.TableBased);
      import DassaultSystemes.Fluid.Types.InitType;
      //Geometry
      parameter Integer N_parallel(min=1) = 1 "Number of parallel identical channels" annotation(Dialog(group="Geometry"));
      //Geometry data for single channel
      parameter Modelica.SIunits.Length L "Length"
        annotation (Dialog(group="Geometry"));
      parameter Modelica.SIunits.Area A "Flow channel cross section"
        annotation (Dialog(group="Geometry"));
      parameter Modelica.SIunits.Length perimeter "Flow channel perimeter"
        annotation (Dialog(group="Geometry"));
      final parameter Modelica.SIunits.Diameter D_h=4*A/perimeter
        "Flow channel hydraulic diameter" annotation (Dialog(group="Geometry"));
      parameter Modelica.SIunits.Area A_ht "Flow channel heat transfer surface"
        annotation (Dialog(group="Geometry"));
      parameter Modelica.SIunits.Volume V=A*L "Flow channel fluid volume"
        annotation (Dialog(group="Geometry"));
      // @i //parameter SI.Height H "Elevation of port_b above port_a" annotation(Dialog(group="Geometry"));
      // @i //final parameter SI.Height[N_FM] hs = fill(H/N_FM,N_FM) "Elevation of volume element outlet above inlet";
      final parameter Modelica.SIunits.Length[N] ls=fill(L/N, N)
        "Lengths of the volume elements";
      //Model parameters
      // @i #### For now leave out the momentum flow terms ####
      // @i parameter Boolean useMomentumFlows = false "Include the momentum flow terms in the momentum balance"
      // @i annotation(Dialog(tab="Settings"),Evaluate=true);
      //Mass flow rates and pressure drop
      Medium.ThermodynamicState[N_FM] states_aFM "Upstream fluid state records for the pressure drop model";
      Medium.ThermodynamicState[N_FM] states_bFM "Reverse flow upstream fluid state records for the pressure drop model";

      Modelica.SIunits.AbsolutePressure[N_FM + 1] ps_FM
        "Pressures at the flow model boundaries";
      Modelica.SIunits.PressureDifference[N_FM] dps_f(start=fill((p_aInit -
            p_bInit)/N_FM, N_FM)) "Friction pressure drop";
      Modelica.SIunits.PressureDifference[N_FM] dps_stat
        "Static head pressure drop";

      Modelica.SIunits.Velocity[N_FM + 1] vs_FM
        "Flow velocities at the flow volume boundaries";
      //Fluid properties
      Medium.ThermodynamicState state_a "Fluid state outside port_a";
      Medium.ThermodynamicState state_b "Fluid state outside port_b";

      replaceable model HT =
        DassaultSystemes.Fluid.HeatTransfer.PipeHeatTransfer.ConstAlpha                      constrainedby
        DassaultSystemes.Fluid.HeatTransfer.PipeHeatTransfer.BasePipeHeatTransfer
      "Heat transfer model" annotation(choicesAllMatching=true,Dialog(enable=useHeatTransfer));
      HT[N] heatTransfer(
        redeclare each final package Medium=Medium,
        final A_flow=fill(A,N),
        final D_h=fill(D_h,N),
        final l=fill(L,N),
        state=mediums.state,
        m_flow={0.5*(m_flows[i]+m_flows[i+1])/N_parallel for i in 1:N})
        annotation(Placement(transformation(extent={{-60,-60},{-40,-40}})));
      replaceable model DP =
        DassaultSystemes.Fluid.FlowFriction.PipeFriction.LinearNominalOpPoint                      constrainedby
        DassaultSystemes.Fluid.FlowFriction.PipeFriction.BasePipeFriction
        "Pressure drop model" annotation(choicesAllMatching=true);
      DP[N_FM] pressureDrop(
        redeclare each final package Medium=Medium,
        each final dp_nominal=dp_nominal/N_FM,
        each final m_flowNominal=m_flowNominal,
        each final d_nominal=d_nominal,
        each final allowFlowReversal=allowFlowReversal,
        final A=fill(A,N_FM),
        final l=ls_FM,
        each final L=L,
        final D_h=fill(D_h,N_FM),
        final state_a=states_aFM,
        final state_b=states_bFM,
        final m_flow=m_flowsFM/N_parallel) annotation(Placement(transformation(extent={{-60,-20},{-40,0}})));
    equation

        assert(not
                  ((not allowFlowReversal and port_a.m_flow < 0 and time > 0)),"\n WARNING!: allowFlowReversal = false and port_a.m_flow < 0 in component \"" + getInstanceName()
        + "\"\n", AssertionLevel.warning);

        if N > 1 then
        //N_FM = N-1
        ps_FM = mediums.p; // ps_FM has N_FM+1 = N elements
      else
        //N_FM = N
        //ps_FM has N+1 elements
        ps_FM[1] = port_a.p;
        ps_FM[2] = mediums[1].p;
      end if;
      dps_f = pressureDrop.dp_f;
      dps_stat = zeros(N_FM);
      if N > 1 then
        vs_FM = {m_flows[i]/N_parallel/mediums[i].d/A for i in 1:N_FM+1};
      else
        vs_FM = {m_flows[i]/N_parallel/mediums[1].d/A for i in 1:N_FM+1};
      end if;
      Fs_pa = N_parallel*A.*ps_FM[1:N_FM];
      Fs_pb = N_parallel*A.*ps_FM[2:N_FM+1];
      Fs_g = N_parallel*A.*dps_stat;
      Fs_f = N_parallel*A.*dps_f;
      // @i if useMomentumFlows then
      // @i   Ia_flows = Medium.density(states_aFM).*vs_FM[1:N_FM].*vs_FM[1:N_FM].*A*N_parallel;
      // @i   Ib_flows = Medium.density(states_bFM).*vs_FM[2:N_FM+1].*vs_FM[2:N_FM+1].*A*N_parallel;
      // @i else
      Ia_flows = zeros(N_FM);
      Ib_flows = zeros(N_FM);
      // @i end if;
      state_a = Medium.setState_phX(port_a.p,inStream(port_a.h_outflow),inStream(port_a.Xi_outflow));
      state_b = Medium.setState_phX(port_b.p,inStream(port_b.h_outflow),inStream(port_b.Xi_outflow));
      if N > 1 then
        //N_FM = N-1
        states_aFM = mediums[1:N-1].state;
        states_bFM = mediums[2:N].state;
      else
        //N_FM = N
        states_aFM[1] = state_a;
        //states_aFM[2:N_FM] = mediums[1:N-1].state;
        states_bFM[1] = mediums[1].state;
      end if;
      Qb_flows = heatTransfer.alpha.*fill(A_ht/N,N).*(heatPortAdapter.T-mediums.T)*N_parallel;

      annotation(Dialog(tab="Initialization",enable=useHeatTransfer),
                  Documentation(info="<html>
<p>
Model of a fluid flow control volume with arbitrary but constant cross section
and discretizable along the direction of the flow. The component is designed for 
use with incompressible liquid media models. Heat transfer across the fluid domain 
boundary perpendicular to the direction of the flow path is optional. If useHeatTransfer
is activated, the user can select a heat transfer model for the pipe wall. Different 
pressure drop and, if applicable, heat transfer correlations can be selected by 
redeclaration of the <code>DP</code> and <code>HT</code> components. Energy and 
mass balances are discretized into <code>N</code> elements. The number of discrete
momentum balances is <code>N-1</code> using a staggered grid approach. A special case
is <code>N</code>&nbsp;=&nbsp;1, where the number of momentum balance elements is
also <code>N</code>. The momentum balance is then located upstream and the pressure
at <code>port_b</code> represents the pressure inside the fluid volume. The energy 
balances are always dynamic, the dynamic momentum balance may be replaced by a 
static formulation. The formulation of the mass balance is static to allow decoupling 
of mass and energy balance to eliminate nonlinear algebraic loops with incompressible 
(independent of pressure) fluid property models. 
</p>
</html>"));
    end DiscretizedLiquidPipe;

    model TorqueFOC
      "Linear torque commanded AIM drive, field-oriented control, continuous inverter"
      import ElectrifiedPowertrains;

      // Interface
      extends
        ElectrifiedPowertrains.ElectricDrives.Interfaces.ElectroMechanicTorqueCommanded;

      // Parameter
      extends
        ElectrifiedPowertrains.ElectricDrives.Interfaces.Parameters.Interfaces;

      // Icon
      extends ElectrifiedPowertrains.Common.Icons.ElectricMachines.Drive;
      extends
        ElectrifiedPowertrains.Common.Icons.ElectricMachines.SubIcons.RotorSquirrel;
      extends
        ElectrifiedPowertrains.Common.Icons.Control.SubIcons.TorqueCommanded_RightCorner;

      replaceable ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Interfaces.TorqueInVoltageOut controller annotation (
        choicesAllMatching,
        Dialog(group="Model Selection"),
        Placement(transformation(extent={{-80,-10},{-60,10}})));
      replaceable ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.Interfaces.Averaged modulationMethod
        annotation (choicesAllMatching, Dialog(group="Model Selection"), Placement(transformation(extent={{-40,-10},{-20,10}})));
      replaceable ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Interfaces.ThermalBase inverter(
        final useThermalPort=useThermalPort)
        annotation (choicesAllMatching, Dialog(group="Model Selection"), Placement(transformation(extent={{0,-10},{20,10}})));
      replaceable ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Interfaces.ThreePhase machine(
        final useThermalPort=useThermalPort,
        final useSupport=useSupport)
        annotation (
        choicesAllMatching,
        Dialog(group="Model Selection"),
        Placement(transformation(extent={{60,-10},{80,10}})));

      Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortAIMC thermalPortMachine if useThermalPort
        annotation (Placement(transformation(extent={{50,90},{70,110}})));
      ElectrifiedPowertrains.PowerElectronics.Inverters.Interfaces.ThermalPortInverter thermalPortInverter if useThermalPort
        annotation (Placement(transformation(extent={{-70,90},{-50,110}})));
    equation
      connect(machine.plug_p, inverter.plug)
        annotation (Line(
          points={{60,0},{20,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(inverter.pin_p, pin_p)
        annotation (Line(
          points={{0,6},{-10,6},{-10,60},{-100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(inverter.pin_n, pin_n)
        annotation (Line(
          points={{0,-6},{-10,-6},{-10,-60},{-100,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(machine.flange, flange) annotation (Line(
          points={{80,0},{100,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(modulationMethod.phaseVoltages, controller.actuatingVoltages)
        annotation (Line(
          points={{-42,0},{-59,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(modulationMethod.normalizedPhaseVoltages, inverter.normalizedPhaseVoltages)
        annotation (Line(
          points={{-19,0},{-2,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(controller.desiredTorque, desiredTorque)
        annotation (Line(
          points={{-82,0},{-90,0},{-90,40},{0,40},{0,120}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(controller.electricDriveBus, machine.electricDriveBus) annotation (Line(
          points={{-70,-10},{-70,-20},{70,-20},{70,-10}},
          color={0,86,166},
          thickness=0.5,
          smooth=Smooth.None));
      connect(inverter.electricDriveBus, machine.electricDriveBus) annotation (Line(
          points={{10,-10},{10,-20},{70,-20},{70,-10}},
          color={0,86,166},
          thickness=0.5,
          smooth=Smooth.None));
      connect(modulationMethod.electricDriveBus, machine.electricDriveBus) annotation (Line(
          points={{-30,-10},{-30,-20},{70,-20},{70,-10}},
          color={0,86,166},
          thickness=0.5,
          smooth=Smooth.None));
      connect(modulationMethod.electricDriveBus, electricDriveBus) annotation (Line(
          points={{-30,-10},{-30,-20},{0,-20},{0,-100}},
          color={0,86,166},
          thickness=0.5,
          smooth=Smooth.None));
      connect(machine.support, support) annotation (Line(points={{80,-10},{80,-100},{100,-100}}, color={0,0,0}));
      connect(thermalPortMachine, machine.thermalPort)
        annotation (Line(points={{60,100},{60,80},{76,80},{76,10},{76,10}}, color={199,0,0}));
      connect(inverter.thermalPortInverter, thermalPortInverter)
        annotation (Line(points={{10,10},{10,10},{10,60},{10,80},{-60,80},{-60,100}}, color={199,0,0}));
      annotation (
        defaultComponentName="torqueFOC_AIM",
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
                                   graphics={Rectangle(extent={{34,6},{46,-6}},
                lineColor={175,175,175})}),   Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics),
        Documentation(info="<html>
<p>Torque controlled (field oriented) physical machine model for a flexible combination of sub-components. It is intended for averaged physical simulations with different levels of detail corresponding to the selected models. The inverter models are always averaged, due to the internal connections in the model. Check the documentation of the selected models as documentation.</p>
</html>"));
    end TorqueFOC;

    package Media_Utilities
      extends Modelica.Icons.Package;
      import SI = Modelica.SIunits;

      package UserGuide "User's Guide"
        extends Modelica.Icons.Information;
        class Overview "Overview"
          extends Modelica.Icons.Information;
          annotation (Documentation(info="<html>
<p>The <b>ExternalMedia</b> library provides a framework for interfacing external codes computing fluid properties to Modelica.Media-compatible component models. The library has been designed with two main goals: maximizing the efficiency of the code, while minimizing the amount of extra code required to interface existing external codes to the library.</p>
<p>The library provides medium packages covering pure fluids models, possibly two-phase, which are 100&percnt; compatible with the <a href=\"modelica://Modelica.Media.Interfaces.PartialTwoPhaseMedium\">Modelica.Media.Interfaces.PartialTwoPhaseMedium</a> interface. </p>
<p>Two external sofwares for fluid property computation are currently suppored by the ExternalMedia library:</p>
<ul>
<li><a href=\"http://www.fluidprop.com\">FluidProp</a>, formerly developed at TU Delft and currently devloped and maintained by Asimptote</li>
<li><a href=\"http://coolprop.org\">CoolProp</a>, developed at the University of Liege and at the Technical University of Denmark (DTU)</li>
</ul>
</html>"));
        end Overview;

        package Usage "Using the ExternalMedia Library"
          extends Modelica.Icons.Information;
          class FluidProp "FluidProp medium models"
            extends Modelica.Icons.Information;
            annotation (Documentation(info="<html>
<p>Pure (or pseudo-pure) medium models from all the libraries in FluidProp can be accessed by extending the <a href=\"modelica://ExternalMedia.Media.FluidPropMedium\">ExternalMedia.Media.FluidPropMedium</a> package. You need to download and install FluidProp on your computer for these models to work: ExternalMedia accesses them through a COM interface.</p>
<p>Set libraryName to &QUOT;FluidProp.RefProp&QUOT;, &QUOT;FluidProp.StanMix&QUOT;, &QUOT;FluidProp.TPSI&QUOT;, &QUOT;FluidProp.IF97&QUOT;, or &QUOT;FluidProp.GasMix&QUOT; (only single-component), depending on the specific library you need to use. Set substanceNames to a single-element string array containing the name of the specific medium, as specified by the FluidProp documentation. Set mediumName to a string that describes the medium (this only used for documentation purposes but has no effect in selecting the medium model).</p>
<p>See <a href=\"modelica://ExternalMedia.Examples\">ExternalMedia.Examples</a> for examples.</p>
<p>Please note that the medium models IF97 and GasMix are already available natively in Modelica.Media as <a href=\"modelica://Modelica.Media.Water.StandardWater\">Water.StandardWater</a> and <a href=\"modelica://Modelica.Media.IdealGases.MixtureGases\">IdealGases.MixtureGases</a>, and are included here for comparison purposes. It is recommended to use the Modelica.Media models instead, since they are much faster to compute. </p>
</html>"));
          end FluidProp;

          class CoolProp "CoolProp medium models"
            extends Modelica.Icons.Information;
            annotation (Documentation(info="<html>
<p>Pure (or pseudo-pure) medium models in CoolProp can be accessed by extending the <a href=\"modelica://ExternalMedia.Media.FluidPropMedium\">ExternalMedia.Media.CoolPropMedium</a> package.</p>
<p>Set substanceNames to a single-element string array containing the name of the specific medium, as specified by the CoolProp documentation. Set mediumName to a string that describes the medium (this only used for documentation purposes but has no effect in selecting the medium model).</p>
<p>See <a href=\"modelica://ExternalMedia.Examples\">ExternalMedia.Examples</a> for examples.</p>
</html>"));
          end CoolProp;
        end Usage;

        class Contact "Contact information"
          extends Modelica.Icons.Contact;
          annotation (Documentation(info="<html>
<p>For suggestions and enquiries regarding the library development and the support of more tools, operating systems and external codes, please contact the main developer:</p>
<p>Francesco Casella
<br>Dipartimento di Elettronica, Informazione e Bioingegneria
<br>Politecnico di Milano
<br>Via Ponzio 34/5
<br>I-20133 Milano ITALY<br>
<a href=\"mailto:francesco.casella@polimi.it\">francesco.casella@polimi.it</a></p>
<p>Submit bug reports to <a href=\"https://trac.modelica.org/Modelica/newticket?component=_ExternalMedia\">
https://trac.modelica.org/Modelica/newticket?component=_ExternalMedia</p>
</html>"));
        end Contact;
        annotation(DocumentationClass = true);
      end UserGuide;

      package Common "Package with common definitions"
        extends Modelica.Icons.Package;
        type InputChoice = enumeration(
            dT "(d,T) as inputs",
            hs "(h,s) as inputs",
            ph "(p,h) as inputs",
            ps "(p,s) as inputs",
            pT "(p,T) as inputs");
        type InputChoiceMixture = enumeration(
            dTX "(d,T,X) as inputs",
            hsX "(h,s,X) as inputs",
            phX "(p,h,X) as inputs",
            psX "(p,s,X) as inputs",
            pTX "(p,T,X) as inputs");
        type InputChoiceIncompressible = enumeration(
            ph "(p,h) as inputs",
            pT "(p,T) as inputs",
            phX "(p,h,X) as inputs",
            pTX "(p,T,X) as inputs");
        function XtoName "A function to convert concentration to substance name"
          extends Modelica.Icons.Function;
          input String substanceName = "";
          input Real[:] composition = {0.0};
          input String delimiter = "|";
          input Boolean debug = false;
          output String result;
        protected
          Integer nextIndex;
          Integer inLength;
          String name;
          String rest;
        algorithm
          if noEvent(size(composition,1) <= 0) then
            assert(not debug, "You are passing an empty composition vector, returning name only: "+substanceName, level = AssertionLevel.warning);
            result :=substanceName;
          else
            assert(noEvent(size(composition,1)==1), "Your mixture has more than two components, ignoring all but the first element.", level = AssertionLevel.warning);
            inLength  := Modelica.Utilities.Strings.length(substanceName);
            nextIndex := Modelica.Utilities.Strings.find(substanceName, delimiter);
            if noEvent(nextIndex<2) then
              // Assuming there are no special options
              name   := substanceName;
              rest   := "";
            else
              name   := Modelica.Utilities.Strings.substring(substanceName, 1, nextIndex-1);
              rest   := Modelica.Utilities.Strings.substring(substanceName, nextIndex, inLength);
            end if;
            if noEvent(noEvent(composition[1]<=0) or noEvent(composition[1]>=1)) then
              result := substanceName;
            else
              result := name + "-" + String(composition[1]) + rest;
            end if;
          end if;
          if noEvent(debug) then
            Modelica.Utilities.Streams.print(result+" --- "+substanceName);
          end if;
        end XtoName;

        function CheckCoolPropOptions
          "A function to extract and check the options passed to CoolProp"
          extends Modelica.Icons.Function;
          input String substance = "";
          input Boolean debug = false;
          output String result;

        protected
          Integer nextIndex;
          Integer intVal;
          Integer length;
          String name;
          String rest;
          // used to process the option
          String option;
          String value;
          // gather all valid options
          String options;
          // accept these inputs and set the default parameters
          String[:] allowedOptions = {
            "calc_transport",
            "enable_TTSE",
            "enable_BICUBIC",
            "enable_EXTTP",
            "twophase_derivsmoothing_xend",
            "rho_smoothing_xend",
            "debug"};
          String[:] defaultOptions = {
            "1",
            "0",
            "0",
            "1",
            "0.0",
            "0.0",
            "0"};
          // predefined delimiters
          String delimiter1 = "|";
          String delimiter2 = "=";

        algorithm
          if noEvent(debug) then
            Modelica.Utilities.Streams.print("input  = " + substance);
          end if;

          name := substance;

          for i in 1:size(allowedOptions,1) loop
            nextIndex := Modelica.Utilities.Strings.find(name, allowedOptions[i]);     // 0 if not found
            if nextIndex==0 then // not found
              name := name+delimiter1+allowedOptions[i]+delimiter2+defaultOptions[i];
            end if;
          end for;

          nextIndex := Modelica.Utilities.Strings.find(name, delimiter1);     // 0 if not found
          if nextIndex > 0 then
            // separate fluid name and options
            length  := Modelica.Utilities.Strings.length(name);
            rest    := Modelica.Utilities.Strings.substring(name, nextIndex+1, length);
            name    := Modelica.Utilities.Strings.substring(name, 1, nextIndex-1);
            options := "";

            while (nextIndex > 0) loop
              nextIndex := Modelica.Utilities.Strings.find(rest, delimiter1);     // 0 if not found
              if nextIndex > 0 then
                option  := Modelica.Utilities.Strings.substring(rest, 1, nextIndex-1);
                length  := Modelica.Utilities.Strings.length(rest);
                rest    := Modelica.Utilities.Strings.substring(rest, nextIndex+1, length);
              else
                option  := rest;
              end if;
              // now option contains enable_TTSE=1 or enable_TTSE
              intVal    := Modelica.Utilities.Strings.find(option, delimiter2);     // 0 if not found
              if intVal > 0 then // found "="
                length  := Modelica.Utilities.Strings.length(option);
                value   := Modelica.Utilities.Strings.substring(option, intVal+1, length);
                option  := Modelica.Utilities.Strings.substring(option, 1, intVal-1);
              else  // enable option by default
                value   := "1";
              end if;
              // now option contains only enable_TTSE
              intVal :=1;
              for i in 1:size(allowedOptions,1) loop
                if Modelica.Utilities.Strings.compare(option,allowedOptions[i])==Modelica.Utilities.Types.Compare.Equal then
                  intVal := intVal - 1;
                end if;
              end for;
              if intVal <> 0 then
                assert(false, "Your option (" + option + ") is unknown.");
              else
                options := options+delimiter1+option+delimiter2+value;
              end if;
            end while;
          else
            // Assuming there are no special options
            name   := substance;
            options:= "";
          end if;

          result := name+options;
          if noEvent(debug) then
            Modelica.Utilities.Streams.print("output = " + result);
          end if;
        end CheckCoolPropOptions;
      end Common;

      package Media "Medium packages compatible with Modelica.Media"
        extends Modelica.Icons.Package;
        package FluidPropMedium "Medium package accessing the FluidProp solver"
          extends BaseClasses.ExternalTwoPhaseMedium;
          redeclare replaceable function setBubbleState
            "Set the thermodynamic state on the bubble line"
            extends Modelica.Icons.Function;
            input SaturationProperties sat "saturation point";
            input FixedPhase phase = 0 "phase flag";
            output ThermodynamicState state "complete thermodynamic state info";
            // Standard definition
            external "C" TwoPhaseMedium_setBubbleState_C_impl(sat, phase, state, mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                  "modelica://CHEETA_Thermal/Resources/Include",                                                                                               LibraryDirectory=
                  "modelica://CHEETA_Thermal/Resources/Library");
            annotation(Inline = true);
          end setBubbleState;

          redeclare replaceable function setDewState
            "Set the thermodynamic state on the dew line"
            extends Modelica.Icons.Function;
            input SaturationProperties sat "saturation point";
            input FixedPhase phase = 0 "phase flag";
            output ThermodynamicState state "complete thermodynamic state info";
            // Standard definition
            external "C" TwoPhaseMedium_setDewState_C_impl(sat, phase, state, mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                  "modelica://CHEETA_Thermal/Resources/Include",                                                                                               LibraryDirectory=
                  "modelica://CHEETA_Thermal/Resources/Library");
            annotation(Inline = true);
          end setDewState;

          redeclare function bubbleEntropy "Return bubble point specific entropy"
            input SaturationProperties sat "saturation property record";
            output SI.SpecificEntropy sl "boiling curve specific entropy";
          algorithm
            sl := specificEntropy(setBubbleState(sat));
          end bubbleEntropy;

          redeclare function dewEntropy "Return dew point specific entropy"
            input SaturationProperties sat "saturation property record";
            output SI.SpecificEntropy sv "dew curve specific entropy";
          algorithm
            sv := specificEntropy(setDewState(sat));
          end dewEntropy;

          redeclare function surfaceTension
            extends Modelica.Icons.Function;
            input SaturationProperties sat "saturation property record";
            output SurfaceTension sigma "Surface tension sigma in the two phase region";
          algorithm
            assert(false, "The FluidProp solver does not provide surface tension");
          end surfaceTension;
        end FluidPropMedium;

        package CoolPropMedium "Medium package accessing the CoolProp solver"
          extends BaseClasses.ExternalTwoPhaseMedium(
            final libraryName = "CoolProp",
            final substanceName = Media_Utilities.Common.CheckCoolPropOptions(
                                                                            substanceNames[1],debug=false));

          redeclare replaceable function isentropicEnthalpy
            input AbsolutePressure p_downstream "downstream pressure";
            input ThermodynamicState refState "reference state for entropy";
            output SpecificEnthalpy h_is "Isentropic enthalpy";
          protected
            SpecificEntropy s_ideal;
            ThermodynamicState state_ideal;
          algorithm
            s_ideal := specificEntropy(refState);
            state_ideal := setState_psX(p_downstream, s_ideal);
            h_is := specificEnthalpy(state_ideal);
          end isentropicEnthalpy;

          redeclare replaceable function setBubbleState
            "Set the thermodynamic state on the bubble line"
            extends Modelica.Icons.Function;
            input SaturationProperties sat "saturation point";
            input FixedPhase phase=0 "phase flag";
            output ThermodynamicState state "complete thermodynamic state info";
            // Standard definition
          external"C" TwoPhaseMedium_setBubbleState_C_impl(
                sat,
                phase,
                state,
                mediumName,
                libraryName,
                substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                  "modelica://CHEETA_Thermal/Resources/Include",                                                                                               LibraryDirectory=
                  "modelica://CHEETA_Thermal/Resources/Library");
            annotation (Inline=true);
          end setBubbleState;

          redeclare replaceable function setDewState
            "Set the thermodynamic state on the dew line"
            extends Modelica.Icons.Function;
            input SaturationProperties sat "saturation point";
            input FixedPhase phase=0 "phase flag";
            output ThermodynamicState state "complete thermodynamic state info";
            // Standard definition
          external"C" TwoPhaseMedium_setDewState_C_impl(
                sat,
                phase,
                state,
                mediumName,
                libraryName,
                substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                  "modelica://CHEETA_Thermal/Resources/Include",                                                                                               LibraryDirectory=
                  "modelica://CHEETA_Thermal/Resources/Library");
            annotation (Inline=true);
          end setDewState;

          redeclare function bubbleEntropy "Return bubble point specific entropy"
            input SaturationProperties sat "saturation property record";
            output SI.SpecificEntropy sl "boiling curve specific entropy";
          algorithm
            sl := specificEntropy(setBubbleState(sat));
          end bubbleEntropy;

          redeclare function dewEntropy "Return dew point specific entropy"
            input SaturationProperties sat "saturation property record";
            output SI.SpecificEntropy sv "dew curve specific entropy";
          algorithm
            sv := specificEntropy(setDewState(sat));
          end dewEntropy;

          redeclare function surfaceTension
            extends Modelica.Icons.Function;
            input SaturationProperties sat "saturation property record";
            output SurfaceTension sigma "Surface tension sigma in the two phase region";
          algorithm
            assert(false, "The CoolProp solver does not provide surface tension");
          end surfaceTension;

        end CoolPropMedium;

        partial package IncompressibleCoolPropMedium
          "External incompressible medium with up to two components using CoolProp"
          extends Modelica.Media.Interfaces.PartialMedium(
            mediumName =  "ExternalMedium",
            singleState = true,
            reducedX =    true);
          import CHEETA_Thermal.Utilities.Media_Utilities.Common.InputChoiceIncompressible;
          constant String libraryName = "CoolProp"
            "Name of the external fluid property computation library";
          constant String substanceName = Media_Utilities.Common.CheckCoolPropOptions(
                                                                                    substanceNames[1],debug=false)
            "Only one substance can be specified, predefined mixture in CoolProp";
          redeclare record extends FluidConstants "external fluid constants"
            MolarMass molarMass "molecular mass";
            Temperature criticalTemperature "critical temperature";
            AbsolutePressure criticalPressure "critical pressure";
            MolarVolume criticalMolarVolume "critical molar Volume";
          end FluidConstants;
          constant InputChoiceIncompressible inputChoice=InputChoiceIncompressible.pTX
            "Default choice of input variables for property computations, incompressibles are in p,T";
          redeclare replaceable record ThermodynamicState =
          Media_Utilities.Media.BaseClasses.ExternalTwoPhaseMedium.ThermodynamicState;

          redeclare replaceable model extends BaseProperties(
            p(stateSelect = if preferredMediumStates and
                               (basePropertiesInputChoice == InputChoiceIncompressible.phX or
                                basePropertiesInputChoice == InputChoiceIncompressible.pTX or
                                basePropertiesInputChoice == InputChoiceIncompressible.ph or
                                basePropertiesInputChoice == InputChoiceIncompressible.pT) then
                                    StateSelect.prefer else StateSelect.default),
            T(stateSelect = if preferredMediumStates and
                               (basePropertiesInputChoice == InputChoiceIncompressible.pTX or
                                basePropertiesInputChoice == InputChoiceIncompressible.pT) then
                                 StateSelect.prefer else StateSelect.default),
            h(stateSelect = if preferredMediumStates and
                               (basePropertiesInputChoice == InputChoiceIncompressible.phX or
                                basePropertiesInputChoice == InputChoiceIncompressible.ph) then
                                 StateSelect.prefer else StateSelect.default))
            import CHEETA_Thermal.Utilities.Media_Utilities.Common.InputChoiceIncompressible;
            parameter InputChoiceIncompressible basePropertiesInputChoice=inputChoice
              "Choice of input variables for property computations";
            Integer phaseInput
              "Phase input for property computation functions, 2 for two-phase, 1 for one-phase, 0 if not known";
            Integer phaseOutput
              "Phase output for medium, 2 for two-phase, 1 for one-phase";
            SpecificEntropy s "Specific entropy";
            //SaturationProperties sat "saturation property record";
          equation
            phaseInput = 1 "Force one-phase property computation";
            R  = Modelica.Constants.small "Gas constant (of mixture if applicable)";
            MM = 0.001 "Molar mass (of mixture or single fluid)";
            if (basePropertiesInputChoice == InputChoiceIncompressible.phX or
                basePropertiesInputChoice == InputChoiceIncompressible.ph) then
              state = setState_phX(p, h, Xi, phaseInput);
              d = density_phX(p, h, Xi, phaseInput);
              s = specificEntropy_phX(p, h, Xi, phaseInput);
              T = temperature_phX(p, h, Xi, phaseInput);
            elseif (basePropertiesInputChoice == InputChoiceIncompressible.pTX or
                    basePropertiesInputChoice == InputChoiceIncompressible.pT) then
              state = setState_pTX(p, T, Xi, phaseInput);
              d = density_pTX(p, T, Xi, phaseInput);
              h = specificEnthalpy_pTX(p, T, Xi, phaseInput);
              s = specificEntropy_pTX(p, T, Xi, phaseInput);
            end if;
            // Compute the internal energy
            u = h - p/d;
            // Compute the saturation properties record
            //sat = setSat_p_state(state);
            // No phase boundary crossing
            phaseOutput = 1;
          end BaseProperties;

          replaceable function setState_ph
            "Return thermodynamic state record from p and h"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "pressure";
            input SpecificEnthalpy h "specific enthalpy";
            input Integer phase = 1 "2 for two-phase, 1 for one-phase, 0 if not known";
            output ThermodynamicState state;
          protected
            String name;
          algorithm
          state := setState_ph_library(p, h, phase, substanceName);
          end setState_ph;

          redeclare replaceable function setState_phX
            "Return thermodynamic state record from p and h"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "pressure";
            input SpecificEnthalpy h "specific enthalpy";
            input MassFraction X[nX] "Mass fractions";
            input Integer phase = 1 "2 for two-phase, 1 for one-phase, 0 if not known";
            output ThermodynamicState state;
          protected
            String name;
          algorithm
            name := Media_Utilities.Common.XtoName(substanceName, X);
          state := setState_ph_library(p, h, phase, name);
          end setState_phX;

          function setState_ph_library "Return thermodynamic state record from p and h"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "pressure";
            input SpecificEnthalpy h "specific enthalpy";
            input Integer phase = 1 "2 for two-phase, 1 for one-phase, 0 if not known";
            input String name "name and mass fractions";
            output ThermodynamicState state;
          external "C" TwoPhaseMedium_setState_ph_C_impl(p, h, phase, state, mediumName, libraryName, name)
            annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                  "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                  "modelica://CHEETA_Thermal/Resources/Library");
          end setState_ph_library;

          replaceable function setState_pT
            "Return thermodynamic state record from p and T"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "pressure";
            input Temperature T "temperature";
            input Integer phase = 1 "2 for two-phase, 1 for one-phase, 0 if not known";
            output ThermodynamicState state;
          protected
            String name;
          algorithm
          state := setState_pT_library(p, T, phase, substanceName);
          end setState_pT;

          redeclare replaceable function setState_pTX
            "Return thermodynamic state record from p and T"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "pressure";
            input Temperature T "temperature";
            input MassFraction X[:] "Mass fractions";
            input Integer phase = 1 "2 for two-phase, 1 for one-phase, 0 if not known";
            output ThermodynamicState state;
          protected
            String name;
          algorithm
            name := Media_Utilities.Common.XtoName(substanceName, X);
          state := setState_pT_library(p, T, phase, name);
          end setState_pTX;

          function setState_pT_library "Return thermodynamic state record from p and T"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "pressure";
            input Temperature T "temperature";
            input Integer phase = 1 "2 for two-phase, 1 for one-phase, 0 if not known";
            input String name "name and mass fractions";
            output ThermodynamicState state;
          external "C" TwoPhaseMedium_setState_pT_C_impl(p, T, state, mediumName, libraryName, name)
            annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                  "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                  "modelica://CHEETA_Thermal/Resources/Library");
          end setState_pT_library;

          redeclare replaceable function setState_psX
            "Return thermodynamic state record from p and s"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "pressure";
            input SpecificEntropy s "specific entropy";
            input MassFraction X[nX] "Mass fractions";
            input Integer phase = 1 "2 for two-phase, 1 for one-phase, 0 if not known";
            output ThermodynamicState state;
          protected
            String in1 = Media_Utilities.Common.XtoName(
                                                      substanceName,X);
            //assert(false, "Incompressibles only support pT and ph as inputs!", level=AssertionLevel.error);
          external "C" TwoPhaseMedium_setState_ps_C_impl(p, s, phase, state, mediumName, libraryName, in1)
            annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                  "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                  "modelica://CHEETA_Thermal/Resources/Library");
          end setState_psX;

          redeclare function density_phX "returns density for given p and h"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input SpecificEnthalpy h "Enthalpy";
            input MassFraction X[nX] "Mass fractions";
            input Integer phase=1 "2 for two-phase, 1 for one-phase, 0 if not known";
          //input ThermodynamicState state;
            output Density d "density";
          algorithm
            d := density_phX_state(p=p, h=h, X=X, state=setState_phX(p=p, h=h, X=X, phase=phase));
          annotation (
            Inline=true);
          end density_phX;

          function density_phX_state "returns density for given p and h"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input SpecificEnthalpy h "Enthalpy";
            input MassFraction X[nX] "Mass fractions";
            input ThermodynamicState state;
            output Density d "density";
          algorithm
            d := density(state);
          annotation (
            Inline=false,
            LateInline=true,
            derivative(noDerivative=state,noDerivative=X)=density_phX_der);
          end density_phX_state;

          replaceable function density_phX_der "Total derivative of density_ph"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input SpecificEnthalpy h "Specific enthalpy";
            input MassFraction X[nX] "Mass fractions";
            input ThermodynamicState state;
            input Real p_der "time derivative of pressure";
            input Real h_der "time derivative of specific enthalpy";
            output Real d_der "time derivative of density";
          algorithm
            d_der := p_der*density_derp_h(state=state)
                   + h_der*density_derh_p(state=state);
          annotation (Inline=true);
          end density_phX_der;

          redeclare replaceable function extends density_derh_p
            "Return derivative of density wrt enthalpy at constant pressure from state"
            // Standard definition
          algorithm
            ddhp := state.ddhp;
            annotation(Inline = true);
          end density_derh_p;

          redeclare replaceable function extends density_derp_h
            "Return derivative of density wrt pressure at constant enthalpy from state"
            // Standard definition
          algorithm
            ddph := state.ddph;
            annotation(Inline = true);
          end density_derp_h;

          redeclare function temperature_phX "returns temperature for given p and h"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input SpecificEnthalpy h "Enthalpy";
            input MassFraction X[nX] "Mass fractions";
            input Integer phase=1 "2 for two-phase, 1 for one-phase, 0 if not known";
            output Temperature T "Temperature";
          algorithm
            T := temperature_phX_state(p=p, h=h, X=X, state=setState_phX(p=p, h=h, X=X, phase=phase));
          annotation (
            Inline=true,
            inverse(h=specificEnthalpy_pTX(p=p, T=T, X=X, phase=phase)));
          end temperature_phX;

          function temperature_phX_state "returns temperature for given p and h"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input SpecificEnthalpy h "Enthalpy";
            input MassFraction X[nX] "Mass fractions";
            input ThermodynamicState state;
            output Temperature T "Temperature";
          algorithm
            T := temperature(state);
          annotation (
            Inline=false,
            LateInline=true,
            inverse(h=specificEnthalpy_pTX_state(p=p, T=T, X=X, state=state)));
          end temperature_phX_state;

            function specificEntropy_phX "returns specific entropy for a given p and h"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input SpecificEnthalpy h "Specific Enthalpy";
            input MassFraction X[nX] "Mass fractions";
            input Integer phase=1 "2 for two-phase, 1 for one-phase, 0 if not known";
            output SpecificEntropy s "Specific Entropy";
            algorithm
            s := specificEntropy_phX_state(p=p, h=h, X=X, state=setState_phX(p=p, h=h, X=X, phase=phase));
            annotation (
            Inline=true);
            end specificEntropy_phX;

          function specificEntropy_phX_state
            "returns specific entropy for a given p and h"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input SpecificEnthalpy h "Specific Enthalpy";
            input MassFraction X[nX] "Mass fractions";
            input ThermodynamicState state;
            output SpecificEntropy s "Specific Entropy";
          algorithm
            s := specificEntropy(state);
          annotation (
            Inline=false,
            LateInline=true,
            derivative(noDerivative=state,noDerivative=X)=specificEntropy_phX_der);
          end specificEntropy_phX_state;

          function specificEntropy_phX_der "time derivative of specificEntropy_phX"
            extends Modelica.Icons.Function;
            input AbsolutePressure p;
            input SpecificEnthalpy h;
            input MassFraction X[nX] "Mass fractions";
            input ThermodynamicState state;
            input Real p_der "time derivative of pressure";
            input Real h_der "time derivative of specific enthalpy";
            output Real s_der "time derivative of specific entropy";
          algorithm
            s_der := p_der*(-1.0/(state.d*state.T))
                   + h_der*( 1.0/state.T);
          annotation (
            Inline=true);
          end specificEntropy_phX_der;

          redeclare function density_pTX "Return density from p and T"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input Temperature T "Temperature";
            input MassFraction X[nX] "Mass fractions";
            input Integer phase=1 "2 for two-phase, 1 for one-phase, 0 if not known";
            output Density d "Density";
          algorithm
            d := density_pTX_state(p=p, T=T, X=X, state=setState_pTX(p=p, T=T, X=X, phase=phase));
          annotation (
            Inline=true);
          end density_pTX;

          function density_pTX_state
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input Temperature T "Temperature";
            input MassFraction X[nX] "Mass fractions";
            input ThermodynamicState state;
            output Density d "Density";
          algorithm
            d := density(state);
          annotation (
            Inline=false,
            LateInline=true);
          end density_pTX_state;

          redeclare function specificEnthalpy_pTX
            "returns specific enthalpy for given p and T"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input Temperature T "Temperature";
            input MassFraction X[nX] "Mass fractions";
            input Integer phase=1 "2 for two-phase, 1 for one-phase, 0 if not known";
            output SpecificEnthalpy h "specific enthalpy";
          algorithm
            h := specificEnthalpy_pTX_state(p=p, T=T, X=X, state=setState_pTX(p=p, T=T, X=X, phase=phase));
          annotation (
            Inline=true,
            inverse(T=temperature_phX(p=p, h=h, X=X, phase=phase)));
          end specificEnthalpy_pTX;

          function specificEnthalpy_pTX_state
            "returns specific enthalpy for given p and T"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input Temperature T "Temperature";
            input MassFraction X[nX] "Mass fractions";
            input ThermodynamicState state;
            output SpecificEnthalpy h "specific enthalpy";
          algorithm
            h := specificEnthalpy(state);
          annotation (
            Inline=false,
            LateInline=true,
            inverse(T=temperature_phX_state(p=p, h=h, X=X, state=state)));
          end specificEnthalpy_pTX_state;

          redeclare function specificEntropy_pTX
            "returns specific entropy for a given p and T"
          extends Modelica.Icons.Function;
          input AbsolutePressure p "Pressure";
          input Temperature T "Temperature";
          input MassFraction X[nX] "Mass fractions";
          input Integer phase=1 "2 for two-phase, 1 for one-phase, 0 if not known";
          output SpecificEntropy s "Specific Entropy";
          algorithm
          s := specificEntropy_pTX_state(p=p, T=T, X=X, state=setState_pTX(p=p, T=T, X=X, phase=phase));
            annotation (
              Inline=true);
          end specificEntropy_pTX;

          function specificEntropy_pTX_state
            "returns specific entropy for a given p and T"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input Temperature T "Temperature";
            input MassFraction X[nX] "Mass fractions";
            input ThermodynamicState state;
            output SpecificEntropy s "Specific Entropy";
          algorithm
            s := specificEntropy(state);
          annotation (
            Inline=false,
            LateInline=true);
          end specificEntropy_pTX_state;

          redeclare replaceable function extends density "Return density from state"
            // Standard definition
          algorithm
            d := state.d;
            annotation(Inline = true);
          end density;

          redeclare replaceable function extends pressure "Return pressure from state"
            // Standard definition
          algorithm
            p := state.p;
            /*  // If special definition in "C"
  external "C" p=  TwoPhaseMedium_pressure_(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
            annotation(Inline = true);
          end pressure;

          redeclare replaceable function extends specificEnthalpy
            "Return specific enthalpy from state"
            // Standard definition
          algorithm
            h := state.h;
            annotation(Inline = true);
          end specificEnthalpy;

          redeclare replaceable function extends specificEntropy
            "Return specific entropy from state"
            // Standard definition
          algorithm
            s := state.s;
            annotation(Inline = true);
          end specificEntropy;

          redeclare replaceable function extends temperature
            "Return temperature from state"
            // Standard definition
          algorithm
            T := state.T;
            annotation(Inline = true);
          end temperature;

          redeclare function extends prandtlNumber
            /*  // If special definition in "C"
  external "C" T=  TwoPhaseMedium_prandtlNumber_(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
            annotation(Inline = true);
          end prandtlNumber;

          redeclare replaceable function extends velocityOfSound
            "Return velocity of sound from state"
            // Standard definition
          algorithm
            a := state.a;
            /*  // If special definition in "C"
  external "C" a=  TwoPhaseMedium_velocityOfSound_(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
            annotation(Inline = true);
          end velocityOfSound;

          redeclare replaceable function extends specificHeatCapacityCp
            "Return specific heat capacity cp from state"
            // Standard definition
          algorithm
            cp := state.cp;
            /*  // If special definition in "C"
  external "C" cp=  TwoPhaseMedium_specificHeatCapacityCp_(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
            annotation(Inline = true);
          end specificHeatCapacityCp;

          redeclare replaceable function extends specificHeatCapacityCv
            "Return specific heat capacity cv from state"
            // Standard definition
          algorithm
            cv := state.cv;
            /*  // If special definition in "C"
  external "C" cv=  TwoPhaseMedium_specificHeatCapacityCv_(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
            annotation(Inline = true);
          end specificHeatCapacityCv;

          redeclare replaceable function extends dynamicViscosity
            "Return dynamic viscosity from state"
            // Standard definition
          algorithm
            eta := state.eta;
            /*  // If special definition in "C"
  external "C" eta=  TwoPhaseMedium_dynamicViscosity_(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
            annotation(Inline = true);
          end dynamicViscosity;

          redeclare replaceable function extends thermalConductivity
            "Return thermal conductivity from state"
            // Standard definition
          algorithm
            lambda := state.lambda;
            /*  // If special definition in "C"
  external "C" lambda=  TwoPhaseMedium_thermalConductivity_(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
            annotation(Inline = true);
          end thermalConductivity;
        end IncompressibleCoolPropMedium;

        package TestMedium "Simple water medium model for debugging and testing"
          extends BaseClasses.ExternalTwoPhaseMedium(
            mediumName = "TestMedium",
            libraryName = "TestMedium",
            ThermoStates = Modelica.Media.Interfaces.Choices.IndependentVariables.pT);
        end TestMedium;

        package BaseClasses "Base classes for external media packages"
          extends Modelica.Icons.BasesPackage;
          package ExternalTwoPhaseMedium "Generic external two phase medium package"
            extends Modelica.Media.Interfaces.PartialTwoPhaseMedium(
              singleState = false,
              onePhase = false,
              smoothModel = false,
              fluidConstants = {externalFluidConstants});
            import CHEETA_Thermal.Utilities.Media_Utilities.Common.InputChoice;
            // mediumName is declared here instead of in the extends clause
            // to break a circular dependency in redeclaration that OpenModelica
            // cannot yet handle
            constant String mediumName="unusablePartialMedium" "Name of the medium";
            constant String libraryName = "UnusableExternalMedium"
              "Name of the external fluid property computation library";
            constant String substanceName = substanceNames[1]
              "Only one substance can be specified";
            constant FluidConstants externalFluidConstants = FluidConstants(
              iupacName = "unknown",
              casRegistryNumber = "unknown",
              chemicalFormula = "unknown",
              structureFormula = "unknown",
              molarMass = getMolarMass(),
              criticalTemperature = getCriticalTemperature(),
              criticalPressure = getCriticalPressure(),
              criticalMolarVolume = getCriticalMolarVolume(),
              acentricFactor = 0,
              triplePointTemperature = 280.0,
              triplePointPressure = 500.0,
              meltingPoint = 280,
              normalBoilingPoint = 380.0,
              dipoleMoment = 2.0);

            constant InputChoice inputChoice=InputChoice.ph
              "Default choice of input variables for property computations";
            redeclare replaceable record ThermodynamicState
              // Fields in ASCII lexicographical order to work in Dymola
              Temperature T "temperature";
              VelocityOfSound a "velocity of sound";
              Modelica.SIunits.CubicExpansionCoefficient beta
                "isobaric expansion coefficient";
              SpecificHeatCapacity cp "specific heat capacity cp";
              SpecificHeatCapacity cv "specific heat capacity cv";
              Density d "density";
              DerDensityByEnthalpy ddhp
                "derivative of density wrt enthalpy at constant pressure";
              DerDensityByPressure ddph
                "derivative of density wrt pressure at constant enthalpy";
              DynamicViscosity eta "dynamic viscosity";
              SpecificEnthalpy h "specific enthalpy";
              Modelica.SIunits.Compressibility kappa "compressibility";
              ThermalConductivity lambda "thermal conductivity";
              AbsolutePressure p "pressure";
              FixedPhase phase(min=0, max=2)
                "phase flag: 2 for two-phase, 1 for one-phase";
              SpecificEntropy s "specific entropy";
            end ThermodynamicState;

            redeclare record SaturationProperties
              // Fields in ASCII lexicographical order to work in Dymola
              Temperature Tsat "saturation temperature";
              Real dTp "derivative of Ts wrt pressure";
              DerDensityByPressure ddldp "derivative of dls wrt pressure";
              DerDensityByPressure ddvdp "derivative of dvs wrt pressure";
              DerEnthalpyByPressure dhldp "derivative of hls wrt pressure";
              DerEnthalpyByPressure dhvdp "derivative of hvs wrt pressure";
              Density dl "density at bubble line (for pressure ps)";
              Density dv "density at dew line (for pressure ps)";
              SpecificEnthalpy hl "specific enthalpy at bubble line (for pressure ps)";
              SpecificEnthalpy hv "specific enthalpy at dew line (for pressure ps)";
              AbsolutePressure psat "saturation pressure";
              SurfaceTension sigma "surface tension";
              SpecificEntropy sl "specific entropy at bubble line (for pressure ps)";
              SpecificEntropy sv "specific entropy at dew line (for pressure ps)";
            end SaturationProperties;

            redeclare replaceable model extends BaseProperties(
              p(stateSelect = if preferredMediumStates and
                                 (basePropertiesInputChoice == InputChoice.ph or
                                  basePropertiesInputChoice == InputChoice.pT or
                                  basePropertiesInputChoice == InputChoice.ps) then
                                      StateSelect.prefer else StateSelect.default),
              T(stateSelect = if preferredMediumStates and
                                 (basePropertiesInputChoice == InputChoice.pT or
                                  basePropertiesInputChoice == InputChoice.dT) then
                                   StateSelect.prefer else StateSelect.default),
              h(stateSelect = if preferredMediumStates and
                                 (basePropertiesInputChoice == InputChoice.hs or
                                  basePropertiesInputChoice == InputChoice.ph) then
                                   StateSelect.prefer else StateSelect.default),
              d(stateSelect = if preferredMediumStates and
                                 basePropertiesInputChoice == InputChoice.dT then
                                   StateSelect.prefer else StateSelect.default))
              import CHEETA_Thermal.Utilities.Media_Utilities.Common.InputChoice;
              parameter InputChoice basePropertiesInputChoice=inputChoice
                "Choice of input variables for property computations";
              FixedPhase phaseInput
                "Phase input for property computation functions, 2 for two-phase, 1 for one-phase, 0 if not known";
              Integer phaseOutput
                "Phase output for medium, 2 for two-phase, 1 for one-phase";
              SpecificEntropy s(
                stateSelect = if (basePropertiesInputChoice == InputChoice.hs or
                                  basePropertiesInputChoice == InputChoice.ps) then
                                 StateSelect.prefer else StateSelect.default)
                "Specific entropy";
              SaturationProperties sat "saturation property record";
            equation
              MM = externalFluidConstants.molarMass;
              R = Modelica.Constants.R/MM;
              if (onePhase or (basePropertiesInputChoice == InputChoice.pT)) then
                phaseInput = 1 "Force one-phase property computation";
              else
                phaseInput = 0 "Unknown phase";
              end if;
              if (basePropertiesInputChoice == InputChoice.ph) then
                // Compute the state record (including the unique ID)
                state = setState_ph(p, h, phaseInput);
                // Compute the remaining variables.
                // It is not possible to use the standard functions like
                // d = density(state), because differentiation for index
                // reduction and change of state variables would not be supported
                // density_ph(), which has an appropriate derivative annotation,
                // is used instead. The implementation of density_ph() uses
                // setState with the same inputs, so there's no actual overhead
                d = density_ph(p, h, phaseInput);
                s = specificEntropy_ph(p, h, phaseInput);
                T = temperature_ph(p, h, phaseInput);
              elseif (basePropertiesInputChoice == InputChoice.dT) then
                state = setState_dT(d, T, phaseInput);
                h = specificEnthalpy(state);
                p = pressure(state);
                s = specificEntropy(state);
              elseif (basePropertiesInputChoice == InputChoice.pT) then
                state = setState_pT(p, T, phaseInput);
                d = density(state);
                h = specificEnthalpy(state);
                s = specificEntropy(state);
              elseif (basePropertiesInputChoice == InputChoice.ps) then
                state = setState_ps(p, s, phaseInput);
                d = density(state);
                h = specificEnthalpy(state);
                T = temperature(state);
              elseif (basePropertiesInputChoice == InputChoice.hs) then
                state = setState_hs(h, s, phaseInput);
                d = density(state);
                p = pressure(state);
                T = temperature(state);
              end if;
              // Compute the internal energy
              u = h - p/d;
              // Compute the saturation properties record only if below critical point
              //sat = setSat_p(min(p,fluidConstants[1].criticalPressure));
              sat = setSat_p_state(state);
              // Event generation for phase boundary crossing
              if smoothModel then
                // No event generation
                phaseOutput = state.phase;
              else
                // Event generation at phase boundary crossing
                if basePropertiesInputChoice == InputChoice.ph then
                  phaseOutput = if ((h > bubbleEnthalpy(sat) and h < dewEnthalpy(sat)) and
                                     p < fluidConstants[1].criticalPressure) then 2 else 1;
                elseif basePropertiesInputChoice == InputChoice.dT then
                  phaseOutput = if  ((d < bubbleDensity(sat) and d > dewDensity(sat)) and
                                      T < fluidConstants[1].criticalTemperature) then 2 else 1;
                elseif basePropertiesInputChoice == InputChoice.ps then
                  phaseOutput = if ((s > bubbleEntropy(sat) and s < dewEntropy(sat)) and
                                     p < fluidConstants[1].criticalPressure) then 2 else 1;
                elseif basePropertiesInputChoice == InputChoice.hs then
                  phaseOutput = if ((s > bubbleEntropy(sat)  and s < dewEntropy(sat)) and
                                    (h > bubbleEnthalpy(sat) and h < dewEnthalpy(sat))) then 2 else 1;
                elseif basePropertiesInputChoice == InputChoice.pT then
                  phaseOutput = 1;
                else
                  assert(false, "You are using an unsupported pair of inputs.");
                end if;
              end if;
            end BaseProperties;

            redeclare function molarMass "Return the molar mass of the medium"
                input ThermodynamicState state;
                output MolarMass MM "Mixture molar mass";
            algorithm
              MM := fluidConstants[1].molarMass;
            end molarMass;

            replaceable function getMolarMass
              output MolarMass MM "molar mass";
              external "C" MM = TwoPhaseMedium_getMolarMass_C_impl(mediumName, libraryName, substanceName)
                annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                               LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end getMolarMass;

            replaceable function getCriticalTemperature
              output Temperature Tc "Critical temperature";
              external "C" Tc = TwoPhaseMedium_getCriticalTemperature_C_impl(mediumName, libraryName, substanceName)
                annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                               LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end getCriticalTemperature;

            replaceable function getCriticalPressure
              output AbsolutePressure pc "Critical temperature";
              external "C" pc = TwoPhaseMedium_getCriticalPressure_C_impl(mediumName, libraryName, substanceName)
                annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                               LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end getCriticalPressure;

            replaceable function getCriticalMolarVolume
              output MolarVolume vc "Critical molar volume";
              external "C" vc = TwoPhaseMedium_getCriticalMolarVolume_C_impl(mediumName, libraryName, substanceName)
                annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                               LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end getCriticalMolarVolume;

            redeclare replaceable function setState_ph
              "Return thermodynamic state record from p and h"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "pressure";
              input SpecificEnthalpy h "specific enthalpy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output ThermodynamicState state;
            external "C" TwoPhaseMedium_setState_ph_C_impl(p, h, phase, state, mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end setState_ph;

            redeclare replaceable function setState_pT
              "Return thermodynamic state record from p and T"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "pressure";
              input Temperature T "temperature";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output ThermodynamicState state;
            external "C" TwoPhaseMedium_setState_pT_C_impl(p, T, state, mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end setState_pT;

            redeclare replaceable function setState_dT
              "Return thermodynamic state record from d and T"
              extends Modelica.Icons.Function;
              input Density d "density";
              input Temperature T "temperature";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output ThermodynamicState state;
            external "C" TwoPhaseMedium_setState_dT_C_impl(d, T, phase, state, mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end setState_dT;

            redeclare replaceable function setState_ps
              "Return thermodynamic state record from p and s"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "pressure";
              input SpecificEntropy s "specific entropy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output ThermodynamicState state;
            external "C" TwoPhaseMedium_setState_ps_C_impl(p, s, phase, state, mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end setState_ps;

            replaceable function setState_hs
              "Return thermodynamic state record from h and s"
              extends Modelica.Icons.Function;
              input SpecificEnthalpy h "specific enthalpy";
              input SpecificEntropy s "specific entropy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output ThermodynamicState state;
            external "C" TwoPhaseMedium_setState_hs_C_impl(h, s, phase, state, mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end setState_hs;

            replaceable function partialDeriv_state
              "Return partial derivative from a thermodynamic state record"
              extends Modelica.Icons.Function;
              input String of "The property to differentiate";
              input String wrt "Differentiate with respect to this";
              input String cst "Keep this constant";
              input ThermodynamicState  state;
              output Real partialDerivative;
              external "C" partialDerivative = TwoPhaseMedium_partialDeriv_state_C_impl(of, wrt, cst, state, mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end partialDeriv_state;

            redeclare function extends setState_phX
            algorithm
              // The composition is an empty vector
              state :=setState_ph(p, h, phase);
            end setState_phX;

            redeclare function extends setState_pTX
            algorithm
              // The composition is an empty vector
              state :=setState_pT(p, T, phase);
            end setState_pTX;

            redeclare function extends setState_dTX
            algorithm
              // The composition is an empty vector
              state :=setState_dT(d, T, phase);
            end setState_dTX;

            redeclare function extends setState_psX
            algorithm
              // The composition is an empty vector
              state :=setState_ps(p, s, phase);
            end setState_psX;

            replaceable function setState_hsX
                                              extends Modelica.Icons.Function;
              input SpecificEnthalpy h "specific enthalpy";
              input SpecificEntropy s "specific entropy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output ThermodynamicState state;
            algorithm
              // The composition is an empty vector
              state :=setState_hs(h, s, phase);
            end setState_hsX;

            redeclare replaceable function density_ph "Return density from p and h"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEnthalpy h "Specific enthalpy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output Density d "Density";
            algorithm
              d := density_ph_state(p=p, h=h, state=setState_ph(p=p, h=h, phase=phase));
            annotation (Inline = true);
            end density_ph;

            function density_ph_state "returns density for given p and h"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEnthalpy h "Enthalpy";
              input ThermodynamicState state;
              output Density d "density";
            algorithm
              d := density(state);
            annotation (
              Inline=false,
              LateInline=true,
              derivative(noDerivative=state)=density_ph_der);
            end density_ph_state;

            replaceable function density_ph_der "Total derivative of density_ph"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEnthalpy h "Specific enthalpy";
              input ThermodynamicState state;
              input Real p_der "time derivative of pressure";
              input Real h_der "time derivative of specific enthalpy";
              output Real d_der "time derivative of density";
            algorithm
              d_der := p_der*density_derp_h(state=state)
                     + h_der*density_derh_p(state=state);
            annotation (Inline=true);
            end density_ph_der;

            redeclare replaceable function temperature_ph
              "Return temperature from p and h"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEnthalpy h "Specific enthalpy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output Temperature T "Temperature";
            algorithm
              T := temperature_ph_state(p=p, h=h, state=setState_ph(p=p, h=h, phase=phase));
            annotation (
              Inline=true,
              inverse(h=specificEnthalpy_pT(p=p, T=T, phase=phase)));
            end temperature_ph;

            function temperature_ph_state "returns temperature for given p and h"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEnthalpy h "Enthalpy";
              input ThermodynamicState state;
              output Temperature T "Temperature";
            algorithm
              T := temperature(state);
            annotation (
              Inline=false,
              LateInline=true,
              inverse(h=specificEnthalpy_pT_state(p=p, T=T, state=state)));
            end temperature_ph_state;

            replaceable function specificEntropy_ph
              "Return specific entropy from p and h"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEnthalpy h "Specific enthalpy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output SpecificEntropy s "specific entropy";
            algorithm
              s := specificEntropy_ph_state(p=p, h=h, state=setState_ph(p=p, h=h, phase=phase));
              annotation (
              Inline=true,
              inverse(h=specificEnthalpy_ps(p=p, s=s, phase=phase)));
            end specificEntropy_ph;

            function specificEntropy_ph_state
              "returns specific entropy for a given p and h"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEnthalpy h "Specific Enthalpy";
              input ThermodynamicState state;
              output SpecificEntropy s "Specific Entropy";
            algorithm
              s := specificEntropy(state);
            annotation (
              Inline=false,
              LateInline=true,
              derivative(noDerivative=state)=specificEntropy_ph_der);
            end specificEntropy_ph_state;

            function specificEntropy_ph_der "time derivative of specificEntropy_ph"
              extends Modelica.Icons.Function;
              input AbsolutePressure p;
              input SpecificEnthalpy h;
              input ThermodynamicState state;
              input Real p_der "time derivative of pressure";
              input Real h_der "time derivative of specific enthalpy";
              output Real s_der "time derivative of specific entropy";
            algorithm
              s_der := p_der*(-1.0/(state.d*state.T))
                     + h_der*( 1.0/state.T);
            annotation (Inline = true);
            end specificEntropy_ph_der;

            redeclare replaceable function density_pT "Return density from p and T"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input Temperature T "Temperature";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output Density d "Density";
            algorithm
              d := density_pT_state(p=p, T=T, state=setState_pT(p=p, T=T, phase=phase));
            annotation (
              Inline=true,
              inverse(p=pressure_dT(d=d, T=T, phase=phase)));
            end density_pT;

            function density_pT_state
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input Temperature T "Temperature";
              input ThermodynamicState state;
              output Density d "Density";
            algorithm
              d := density(state);
            annotation (
              Inline=false,
              LateInline=true,
              inverse(p=pressure_dT_state(d=d, T=T, state=state)));
            end density_pT_state;

            replaceable function density_pT_der "Total derivative of density_pT"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input Temperature T "Temperature";
              input FixedPhase phase "2 for two-phase, 1 for one-phase, 0 if not known";
              input Real p_der;
              input Real T_der;
              output Real d_der;
            algorithm
              d_der:=density_derp_T(setState_pT(p, T))*p_der +
                     density_derT_p(setState_pT(p, T))*T_der;
              /*  // If special definition in "C"
    external "C" d_der=  TwoPhaseMedium_density_pT_der_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
    */
              annotation(Inline = true);
            end density_pT_der;

            redeclare replaceable function specificEnthalpy_pT
              "Return specific enthalpy from p and T"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input Temperature T "Temperature";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output SpecificEnthalpy h "specific enthalpy";
            algorithm
              h := specificEnthalpy_pT_state(p=p, T=T, state=setState_pT(p=p, T=T, phase=phase));
            annotation (
              Inline=true,
              inverse(T=temperature_ph(p=p, h=h, phase=phase)));
            end specificEnthalpy_pT;

            function specificEnthalpy_pT_state
              "returns specific enthalpy for given p and T"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input Temperature T "Temperature";
              input ThermodynamicState state;
              output SpecificEnthalpy h "specific enthalpy";
            algorithm
              h := specificEnthalpy(state);
            annotation (
              Inline=false,
              LateInline=true,
              inverse(T=temperature_ph_state(p=p, h=h, state=state)));
            end specificEnthalpy_pT_state;

            function specificEntropy_pT "returns specific entropy for a given p and T"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input Temperature T "Temperature";
              input FixedPhase phase=0 "2 for two-phase, 1 for one-phase, 0 if not known";
              output SpecificEntropy s "Specific Entropy";
            algorithm
              s := specificEntropy_pT_state(p=p, T=T, state=setState_pT(p=p, T=T, phase=phase));
            annotation (
              Inline=true,
              inverse(T=temperature_ps(p=p, s=s, phase=phase)));
            end specificEntropy_pT;

            function specificEntropy_pT_state
              "returns specific entropy for a given p and T"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input Temperature T "Temperature";
              input ThermodynamicState state;
              output SpecificEntropy s "Specific Entropy";
            algorithm
              s := specificEntropy(state);
            annotation (
              Inline=false,
              LateInline=true);
            end specificEntropy_pT_state;

            redeclare replaceable function pressure_dT "Return pressure from d and T"
              extends Modelica.Icons.Function;
              input Density d "Density";
              input Temperature T "Temperature";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output AbsolutePressure p "Pressure";
            algorithm
              p := pressure_dT_state(d=d, T=T, state=setState_dT(d=d, T=T, phase=phase));
              annotation (
              Inline=true,
              inverse(d=density_pT(p=p, T=T, phase=phase)));
            end pressure_dT;

            function pressure_dT_state
              extends Modelica.Icons.Function;
              input Density d "Density";
              input Temperature T "Temperature";
              input ThermodynamicState state;
              output AbsolutePressure p "pressure";
            algorithm
              p := pressure(state);
            annotation (
              Inline=false,
              LateInline=true,
              inverse(d=density_pT_state(p=p, T=T, state=state)));
            end pressure_dT_state;

            redeclare replaceable function specificEnthalpy_dT
              "Return specific enthalpy from d and T"
              extends Modelica.Icons.Function;
              input Density d "Density";
              input Temperature T "Temperature";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output SpecificEnthalpy h "specific enthalpy";
            algorithm
              h := specificEnthalpy_dT_state(d=d, T=T, state=setState_dT(d=d, T=T, phase=phase));
            annotation (
              Inline=true);
            end specificEnthalpy_dT;

            function specificEnthalpy_dT_state
              extends Modelica.Icons.Function;
              input Density d "Density";
              input Temperature T "Temperature";
              input ThermodynamicState state;
              output SpecificEnthalpy h "SpecificEnthalpy";
            algorithm
              h := specificEnthalpy(state);
            annotation (
              Inline=false,
              LateInline=true);
            end specificEnthalpy_dT_state;

            function specificEntropy_dT "returns specific entropy for a given d and T"
              extends Modelica.Icons.Function;
              input Density d "Density";
              input Temperature T "Temperature";
              input FixedPhase phase=0 "2 for two-phase, 1 for one-phase, 0 if not known";
            output SpecificEntropy s "Specific Entropy";
            algorithm
              s := specificEntropy_dT_state(d=d, T=T, state=setState_dT(d=d, T=T, phase=phase));
            annotation (Inline=true);
            end specificEntropy_dT;

            function specificEntropy_dT_state
              "returns specific entropy for a given d and T"
              extends Modelica.Icons.Function;
              input Density d "Density";
              input Temperature T "Temperature";
              input ThermodynamicState state;
              output SpecificEntropy s "Specific Entropy";
            algorithm
              s := specificEntropy(state);
            annotation (
              Inline=false,
              LateInline=true);
            end specificEntropy_dT_state;

            redeclare replaceable function density_ps "Return density from p and s"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEntropy s "Specific entropy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output Density d "Density";
            algorithm
              d := density_ps_state(p=p, s=s, state=setState_ps(p=p, s=s, phase=phase));
            annotation (
              Inline=true);
            end density_ps;

            function density_ps_state "Return density from p and s"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEntropy s "Specific entropy";
              input ThermodynamicState state;
              output Density d "Density";
            algorithm
              d := density(state);
            annotation (
              Inline=false,
              LateInline=true,
              derivative(noDerivative=state) = density_ps_der);
            end density_ps_state;

            replaceable partial function density_ps_der "Total derivative of density_ps"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEntropy s "Specific entropy";
              input ThermodynamicState state;
              input Real p_der;
              input Real h_der;
              output Real d_der;
              // To be implemented
              annotation(Inline = true);
            end density_ps_der;

            redeclare replaceable function temperature_ps
              "Return temperature from p and s"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEntropy s "Specific entropy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output Temperature T "Temperature";
            algorithm
              T := temperature_ps_state(p=p, s=s, state=setState_ps(p=p, s=s, phase=phase));
            annotation (
              Inline=true,
              inverse(s=specificEntropy_pT(p=p, T=T, phase=phase)));
            end temperature_ps;

            function temperature_ps_state "returns temperature for given p and s"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEntropy s "Specific entropy";
              input ThermodynamicState state;
              output Temperature T "Temperature";
            algorithm
              T := temperature(state);
            annotation (
              Inline=false,
              LateInline=true,
              inverse(s=specificEntropy_pT_state(p=p, T=T, state=state)));
            end temperature_ps_state;

            redeclare replaceable function specificEnthalpy_ps
              "Return specific enthalpy from p and s"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEntropy s "Specific entropy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output SpecificEnthalpy h "specific enthalpy";
            algorithm
              h := specificEnthalpy_ps_state(p=p, s=s, state=setState_ps(p=p, s=s, phase=phase));
              annotation (
              Inline = true,
              inverse(s=specificEntropy_ph(p=p, h=h, phase=phase)));
            end specificEnthalpy_ps;

            function specificEnthalpy_ps_state "Return enthalpy from p and s"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "Pressure";
              input SpecificEntropy s "Specific entropy";
              input ThermodynamicState state;
              output SpecificEnthalpy h "Enthalpy";
            algorithm
              h := specificEnthalpy(state);
            annotation (
              Inline=false,
              LateInline=true,
              inverse(s=specificEntropy_ph_state(p=p, h=h, state=state)));
            end specificEnthalpy_ps_state;

            function density_hs "Return density for given h and s"
              extends Modelica.Icons.Function;
              input SpecificEnthalpy h "Enthalpy";
              input SpecificEntropy s "Specific entropy";
              input FixedPhase phase=0 "2 for two-phase, 1 for one-phase, 0 if not known";
              output Density d "density";
            algorithm
              d := density_hs_state(h=h, s=s, state=setState_hs(h=h, s=s, phase=phase));
            annotation (
              Inline=true);
            end density_hs;

            function density_hs_state "Return density for given h and s"
              extends Modelica.Icons.Function;
              input SpecificEnthalpy h "Enthalpy";
              input SpecificEntropy s "Specific entropy";
              input ThermodynamicState state;
              output Density d "density";
            algorithm
              d := density(state);
            annotation (
              Inline=false,
              LateInline=true);
            end density_hs_state;

            replaceable function pressure_hs "Return pressure from h and s"
              extends Modelica.Icons.Function;
              input SpecificEnthalpy h "Specific enthalpy";
              input SpecificEntropy s "Specific entropy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output AbsolutePressure p "Pressure";
            algorithm
              p := pressure_hs_state(h=h, s=s, state=setState_hs(h=h, s=s, phase=phase));
              annotation (
                Inline = true,
                inverse(
                  h=specificEnthalpy_ps(p=p, s=s, phase=phase),
                  s=specificEntropy_ph(p=p, h=h, phase=phase)));
            end pressure_hs;

            function pressure_hs_state "Return pressure for given h and s"
              extends Modelica.Icons.Function;
              input SpecificEnthalpy h "Enthalpy";
              input SpecificEntropy s "Specific entropy";
              input ThermodynamicState state;
              output AbsolutePressure p "Pressure";
            algorithm
              p := pressure(state);
            annotation (
              Inline=false,
              LateInline=true);
            end pressure_hs_state;

            replaceable function temperature_hs "Return temperature from h and s"
              extends Modelica.Icons.Function;
              input SpecificEnthalpy h "Specific enthalpy";
              input SpecificEntropy s "Specific entropy";
              input FixedPhase phase = 0
                "2 for two-phase, 1 for one-phase, 0 if not known";
              output Temperature T "Temperature";
            algorithm
              T := temperature_hs_state(h=h, s=s, state=setState_hs(h=h, s=s, phase=phase));
              annotation (
                Inline = true);
            end temperature_hs;

            function temperature_hs_state "Return temperature for given h and s"
              extends Modelica.Icons.Function;
              input SpecificEnthalpy h "Enthalpy";
              input SpecificEntropy s "Specific entropy";
              input ThermodynamicState state;
              output Temperature T "Temperature";
            algorithm
              T := temperature(state);
            annotation (
              Inline=false,
              LateInline=true);
            end temperature_hs_state;

            redeclare function extends prandtlNumber "Returns Prandtl number"
              /*  // If special definition in "C"
  external "C" T=  TwoPhaseMedium_prandtlNumber_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end prandtlNumber;

            redeclare replaceable function extends temperature
              "Return temperature from state"
              // Standard definition
            algorithm
              T := state.T;
              /*  // If special definition in "C"
  external "C" T=  TwoPhaseMedium_temperature_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end temperature;

            redeclare replaceable function extends velocityOfSound
              "Return velocity of sound from state"
              // Standard definition
            algorithm
              a := state.a;
              /*  // If special definition in "C"
  external "C" a=  TwoPhaseMedium_velocityOfSound_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end velocityOfSound;

            redeclare replaceable function extends isobaricExpansionCoefficient
              "Return isobaric expansion coefficient from state"
              // Standard definition
            algorithm
              beta := state.beta;
              /*  // If special definition in "C"
  external "C" beta=  TwoPhaseMedium_isobaricExpansionCoefficient_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end isobaricExpansionCoefficient;

            redeclare replaceable function extends isentropicExponent
              "Return isentropic exponent"
              extends Modelica.Icons.Function;
            algorithm
              gamma := density(state) / pressure(state) * velocityOfSound(state) * velocityOfSound(state);
            end isentropicExponent;

            redeclare replaceable function extends specificHeatCapacityCp
              "Return specific heat capacity cp from state"
              // Standard definition
            algorithm
              cp := state.cp;
              /*  // If special definition in "C"
  external "C" cp=  TwoPhaseMedium_specificHeatCapacityCp_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end specificHeatCapacityCp;

            redeclare replaceable function extends specificHeatCapacityCv
              "Return specific heat capacity cv from state"
              // Standard definition
            algorithm
              cv := state.cv;
              /*  // If special definition in "C"
  external "C" cv=  TwoPhaseMedium_specificHeatCapacityCv_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end specificHeatCapacityCv;

            redeclare replaceable function extends density "Return density from state"
              // Standard definition
            algorithm
              d := state.d;
              /*  // If special definition in "C"
  external "C" d=  TwoPhaseMedium_density_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end density;

            redeclare replaceable function extends density_derh_p
              "Return derivative of density wrt enthalpy at constant pressure from state"
              // Standard definition
            algorithm
              ddhp := state.ddhp;
              /*  // If special definition in "C"
  external "C" ddhp=  TwoPhaseMedium_density_derh_p_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end density_derh_p;

            redeclare replaceable function extends density_derp_h
              "Return derivative of density wrt pressure at constant enthalpy from state"
              // Standard definition
            algorithm
              ddph := state.ddph;
              /*  // If special definition in "C"
  external "C" ddph=  TwoPhaseMedium_density_derp_h_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end density_derp_h;

            redeclare replaceable function extends density_derp_T
            algorithm
              ddpT := state.kappa*state.d;
            end density_derp_T;

            redeclare replaceable function extends density_derT_p
            algorithm
              ddTp :=-state.beta*state.d;
            end density_derT_p;

            redeclare replaceable function extends dynamicViscosity
              "Return dynamic viscosity from state"
              // Standard definition
            algorithm
              eta := state.eta;
              /*  // If special definition in "C"
  external "C" eta=  TwoPhaseMedium_dynamicViscosity_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end dynamicViscosity;

            redeclare replaceable function extends specificEnthalpy
              "Return specific enthalpy from state"
              // Standard definition
            algorithm
              h := state.h;
              /*  // If special definition in "C"
  external "C" h=  TwoPhaseMedium_specificEnthalpy_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end specificEnthalpy;

            redeclare replaceable function extends specificInternalEnergy
              "Returns specific internal energy"
              extends Modelica.Icons.Function;
            algorithm
              u := specificEnthalpy(state) - pressure(state)/density(state);
            end specificInternalEnergy;

            redeclare replaceable function extends isothermalCompressibility
              "Return isothermal compressibility from state"
              // Standard definition
            algorithm
              kappa := state.kappa;
              /*  // If special definition in "C"
  external "C" kappa=  TwoPhaseMedium_isothermalCompressibility_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end isothermalCompressibility;

            redeclare replaceable function extends thermalConductivity
              "Return thermal conductivity from state"
              // Standard definition
            algorithm
              lambda := state.lambda;
              /*  // If special definition in "C"
  external "C" lambda=  TwoPhaseMedium_thermalConductivity_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end thermalConductivity;

            redeclare replaceable function extends pressure "Return pressure from state"
              // Standard definition
            algorithm
              p := state.p;
              /*  // If special definition in "C"
  external "C" p=  TwoPhaseMedium_pressure_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end pressure;

            redeclare replaceable function extends specificEntropy
              "Return specific entropy from state"
              // Standard definition
            algorithm
              s := state.s;
              /*  // If special definition in "C"
    external "C" s=  TwoPhaseMedium_specificEntropy_C_impl(state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end specificEntropy;

            redeclare replaceable function extends isentropicEnthalpy
            external "C" h_is = TwoPhaseMedium_isentropicEnthalpy_C_impl(p_downstream, refState,
             mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end isentropicEnthalpy;

            redeclare replaceable function setSat_p "Return saturation properties from p"
              extends Modelica.Icons.Function;
              input AbsolutePressure p "pressure";
              output SaturationProperties sat "saturation property record";
            external "C" TwoPhaseMedium_setSat_p_C_impl(p, sat, mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end setSat_p;

            replaceable function setSat_p_state
              "Return saturation properties from the state"
              extends Modelica.Icons.Function;
              input ThermodynamicState state;
              output SaturationProperties sat "saturation property record";
              // Standard definition
            algorithm
              sat:=setSat_p(state.p);
              //Redeclare this function for more efficient implementations avoiding the repeated computation of saturation properties
            /*  // If special definition in "C"
  external "C" TwoPhaseMedium_setSat_p_state_C_impl(state, sat)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end setSat_p_state;

            redeclare replaceable function setSat_T "Return saturation properties from p"
              extends Modelica.Icons.Function;
              input Temperature T "temperature";
              output SaturationProperties sat "saturation property record";
            external "C" TwoPhaseMedium_setSat_T_C_impl(T, sat, mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end setSat_T;

            replaceable function setSat_T_state
              "Return saturation properties from the state"
              extends Modelica.Icons.Function;
              input ThermodynamicState state;
              output SaturationProperties sat "saturation property record";
              // Standard definition
            algorithm
              sat:=setSat_T(state.T);
              //Redeclare this function for more efficient implementations avoiding the repeated computation of saturation properties
            /*  // If special definition in "C"
  external "C" TwoPhaseMedium_setSat_T_state_C_impl(state, sat)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end setSat_T_state;

            redeclare replaceable function extends setBubbleState
              "set the thermodynamic state on the bubble line"
              extends Modelica.Icons.Function;
              input SaturationProperties sat "saturation point";
              input FixedPhase phase(min = 1, max = 2) =  1 "phase: default is one phase";
              output ThermodynamicState state "complete thermodynamic state info";
              // Standard definition
            algorithm
              state :=setState_ph(sat.psat, sat.hl, phase);
              /*  // If special definition in "C"
  external "C" TwoPhaseMedium_setBubbleState_C_impl(sat, phase, state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end setBubbleState;

            redeclare replaceable function extends setDewState
              "set the thermodynamic state on the dew line"
              extends Modelica.Icons.Function;
              input SaturationProperties sat "saturation point";
              input FixedPhase phase(min = 1, max = 2) = 1 "phase: default is one phase";
              output ThermodynamicState state "complete thermodynamic state info";
              // Standard definition
            algorithm
              state :=setState_ph(sat.psat, sat.hv, phase);
              /*  // If special definition in "C"
  external "C" TwoPhaseMedium_setDewState_C_impl(sat, phase, state, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end setDewState;

            redeclare replaceable function extends saturationTemperature
              // Standard definition
            algorithm
              T :=saturationTemperature_sat(setSat_p(p));
              /*  // If special definition in "C"
  external "C" T=  TwoPhaseMedium_saturationTemperature_C_impl(p, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end saturationTemperature;

            redeclare function extends saturationTemperature_sat

              annotation(Inline = true);
            end saturationTemperature_sat;

            redeclare replaceable function extends saturationTemperature_derp "Returns derivative of saturation temperature w.r.t.. pressureBeing this function inefficient, it is strongly recommended to use saturationTemperature_derp_sat
     and never use saturationTemperature_derp directly"
            external "C" dTp = TwoPhaseMedium_saturationTemperature_derp_C_impl(p, mediumName, libraryName, substanceName)
              annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory=
                    "modelica://CHEETA_Thermal/Resources/Include",                                                                                             LibraryDirectory=
                    "modelica://CHEETA_Thermal/Resources/Library");
            end saturationTemperature_derp;

            redeclare replaceable function saturationTemperature_derp_sat
              "Returns derivative of saturation temperature w.r.t.. pressure"
              extends Modelica.Icons.Function;
              input SaturationProperties sat "saturation property record";
              output Real dTp "derivative of saturation temperature w.r.t. pressure";
              // Standard definition
            algorithm
              dTp := sat.dTp;
              /*  // If special definition in "C"
  external "C" dTp=  TwoPhaseMedium_saturationTemperature_derp_sat_C_impl(sat.psat, sat.Tsat, sat.uniqueID, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end saturationTemperature_derp_sat;

            redeclare replaceable function extends dBubbleDensity_dPressure
              "Returns bubble point density derivative"
              // Standard definition
            algorithm
              ddldp := sat.ddldp;
              /*  // If special definition in "C"
  external "C" ddldp=  TwoPhaseMedium_dBubbleDensity_dPressure_C_impl(sat, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end dBubbleDensity_dPressure;

            redeclare replaceable function extends dDewDensity_dPressure
              "Returns dew point density derivative"
              // Standard definition
            algorithm
              ddvdp := sat.ddvdp;
              /*  // If special definition in "C"
  external "C" ddvdp=  TwoPhaseMedium_dDewDensity_dPressure_C_impl(sat, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end dDewDensity_dPressure;

            redeclare replaceable function extends dBubbleEnthalpy_dPressure
              "Returns bubble point specific enthalpy derivative"
              // Standard definition
            algorithm
              dhldp := sat.dhldp;
              /*  // If special definition in "C"
  external "C" dhldp=  TwoPhaseMedium_dBubbleEnthalpy_dPressure_C_impl(sat, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end dBubbleEnthalpy_dPressure;

            redeclare replaceable function extends dDewEnthalpy_dPressure
              "Returns dew point specific enthalpy derivative"
              // Standard definition
            algorithm
              dhvdp := sat.dhvdp;
              /*  // If special definition in "C"
  external "C" dhvdp=  TwoPhaseMedium_dDewEnthalpy_dPressure_C_impl(sat, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end dDewEnthalpy_dPressure;

            redeclare replaceable function extends bubbleDensity
              "Returns bubble point density"
              // Standard definition
            algorithm
              dl := sat.dl;
              /*  // If special definition in "C"
  external "C" dl=  TwoPhaseMedium_bubbleDensity_C_impl(sat, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end bubbleDensity;

            redeclare replaceable function extends dewDensity "Returns dew point density"
              // Standard definition
            algorithm
              dv := sat.dv;
              /*  // If special definition in "C"
  external "C" dv=  TwoPhaseMedium_dewDensity_C_impl(sat, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end dewDensity;

            redeclare replaceable function extends bubbleEnthalpy
              "Returns bubble point specific enthalpy"
              // Standard definition
            algorithm
              hl := sat.hl;
              /*  // If special definition in "C"
  external "C" hl=  TwoPhaseMedium_bubbleEnthalpy_C_impl(sat, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end bubbleEnthalpy;

            redeclare replaceable function extends dewEnthalpy
              "Returns dew point specific enthalpy"
              // Standard definition
            algorithm
              hv := sat.hv;
              /*  // If special definition in "C"
  external "C" hv=  TwoPhaseMedium_dewEnthalpy_C_impl(sat, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end dewEnthalpy;

            redeclare replaceable function extends saturationPressure
              // Standard definition
            algorithm
              p :=saturationPressure_sat(setSat_T(T));
              /*  // If special definition in "C"
  external "C" p=  TwoPhaseMedium_saturationPressure_C_impl(T, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = false,
                         LateInline = true,
                         derivative = saturationPressure_der);
            end saturationPressure;

            function saturationPressure_der "Return saturation pressure time derivative"
              extends Modelica.Icons.Function;
              input Temperature T "temperature";
              input Real T_der "Temperature derivative";
              output Real p_der "saturation pressure derivative";
              // Standard definition
            algorithm
              p_der :=T_der/saturationTemperature_derp_sat(setSat_T(T));
              annotation(Inline = true);
            end saturationPressure_der;

            redeclare function extends saturationPressure_sat

              annotation(Inline = true);
            end saturationPressure_sat;

            redeclare replaceable function extends surfaceTension
              "Returns surface tension sigma in the two phase region"
              //Standard definition
            algorithm
              sigma := sat.sigma;
              /*  //If special definition in "C"
  external "C" sigma=  TwoPhaseMedium_surfaceTension_C_impl(sat, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end surfaceTension;

            redeclare replaceable function extends bubbleEntropy
              "Returns bubble point specific entropy"
              //Standard definition
            algorithm
              sl := specificEntropy(setBubbleState(sat));
              /*  //If special definition in "C"
  external "C" sl=  TwoPhaseMedium_bubbleEntropy_C_impl(sat, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end bubbleEntropy;

            redeclare replaceable function extends dewEntropy
              "Returns dew point specific entropy"
              //Standard definition
            algorithm
              sv := specificEntropy(setDewState(sat));
              /*  //If special definition in "C"
  external "C" sv=  TwoPhaseMedium_dewEntropy_C_impl(sat, mediumName, libraryName, substanceName)
    annotation(Include="#include \"externalmedialib.h\"", Library="ExternalMediaLib", IncludeDirectory="modelica://ExternalMedia/Resources/Include", LibraryDirectory="modelica://ExternalMedia/Resources/Library");
*/
              annotation(Inline = true);
            end dewEntropy;
          end ExternalTwoPhaseMedium;
        end BaseClasses;
      end Media;

      package Examples "Examples of external medium models"
        extends Modelica.Icons.Package;
        package WaterIF95 "RefProp water model"
          extends Media_Utilities.Media.FluidPropMedium(
            mediumName="Water",
            libraryName="FluidProp.RefProp",
            substanceNames={"H2O"},
            ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.ph);
        end WaterIF95;

        package WaterTPSI "TPSI Water model"
          extends Media_Utilities.Media.FluidPropMedium(
            mediumName="Water",
            libraryName="FluidProp.TPSI",
            substanceNames={"H2O"},
            ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.ph);
        end WaterTPSI;

        package WaterIF97 "IF97 Water model"
          extends Media_Utilities.Media.FluidPropMedium(
            mediumName="Water",
            libraryName="FluidProp.IF97",
            substanceNames={"H2O"},
            ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.ph);
        end WaterIF97;

        package CO2StanMix "StanMix model of CO2"
          extends Media_Utilities.Media.FluidPropMedium(
            mediumName="Carbon Dioxide",
            libraryName="FluidProp.StanMix",
            substanceNames={"CO2"},
            ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.ph);
        end CO2StanMix;

        package CO2RefProp "RefProp model of CO2"
          extends Media_Utilities.Media.FluidPropMedium(
            mediumName="Carbon Dioxide",
            libraryName="FluidProp.RefProp",
            substanceNames={"CO2"},
            ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.ph,
            SpecificEnthalpy(start=2e5));

        end CO2RefProp;

        package CO2CoolProp "CoolProp model of CO2"
          extends Media_Utilities.Media.CoolPropMedium(
            mediumName="CarbonDioxide",
            substanceNames={"CO2"},
            ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.ph,
            SpecificEnthalpy(start=2e5));

        end CO2CoolProp;

        package WaterCoolProp "CoolProp model of water"
          extends Media_Utilities.Media.CoolPropMedium(
            mediumName="Water",
            substanceNames={"water"},
            ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.ph,
            AbsolutePressure(start=10e5),
            SpecificEnthalpy(start=2e5));

        end WaterCoolProp;

        package DowQCoolProp "DowthermQ properties from CoolProp"
          extends Media_Utilities.Media.IncompressibleCoolPropMedium(
            mediumName="DowQ",
            substanceNames={"DowQ|calc_transport=1"},
            ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.pT);
        end DowQCoolProp;

        package LiBrAQCoolProp "Lithium bromide solution properties from CoolProp"
          extends Media_Utilities.Media.IncompressibleCoolPropMedium(
            mediumName="LiBr",
            substanceNames={"LiBr|calc_transport=1",
                "dummyToMakeBasePropertiesWork"},
            ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.pTX);
        end LiBrAQCoolProp;

      end Examples;

      package Test "Test models for the different solvers"
        extends Modelica.Icons.ExamplesPackage;
        package TestMedium "Test cases for TestMedium"
          extends Modelica.Icons.ExamplesPackage;
          model TestConstants "Test case using TestMedium with package constants"
            extends Modelica.Icons.Example;
            replaceable package Medium = Media.TestMedium;
            Medium.Temperature Tc=Medium.fluidConstants[1].criticalTemperature;
          end TestConstants;

          model TestState "Test case using TestMedium with a single state record"
            extends Modelica.Icons.Example;
            replaceable package Medium = Media.TestMedium;
            Medium.ThermodynamicState state;
          equation
            state = Medium.setState_ph(1e5, 1e5 + 1e5*time);
          end TestState;

          model TestSat
            "Test case using TestMedium with a single saturation properties record"
            extends Modelica.Icons.Example;
            replaceable package Medium = Media.TestMedium;
            Medium.SaturationProperties sat;
          equation
            sat = Medium.setSat_p(1e5 + 1e5*time);
          end TestSat;

          model TestStatesSat "Test case using TestMedium with state + sat records"
            extends Modelica.Icons.Example;
            replaceable package Medium = Media.TestMedium;
            Medium.BaseProperties baseProperties1;
            Medium.BaseProperties baseProperties2;
            Medium.ThermodynamicState state1;
            Medium.ThermodynamicState state2;
            Medium.SaturationProperties sat1;
            Medium.SaturationProperties sat2;
            Medium.Temperature Ts;
            Medium.AbsolutePressure ps;
            GenericModels.CompleteThermodynamicState completeState1(redeclare
                package Medium =
                         Medium, state=state1);
            GenericModels.CompleteThermodynamicState completeState2(redeclare
                package Medium =
                         Medium, state=state2);
            GenericModels.CompleteSaturationProperties completeSat1(redeclare
                package Medium =
                         Medium, sat=sat1);
            GenericModels.CompleteSaturationProperties completeSat2(redeclare
                package Medium =
                         Medium, sat=sat2);
            GenericModels.CompleteBubbleDewStates completeBubbleDewStates1(redeclare
                package Medium = Medium, sat=sat1);
            GenericModels.CompleteBubbleDewStates completeBubbleDewStates2(redeclare
                package Medium = Medium, sat=sat1);
          equation
            baseProperties1.p = 1e5 + 1e5*time;
            baseProperties1.h = 1e5;
            baseProperties2.p = 1e5;
            baseProperties2.h = 1e5 + 2e5*time;
            state1 = Medium.setState_ph(1e5 + 1e5*time, 1e5);
            state2 = Medium.setState_pT(1e5, 300 + 50*time);
            sat1 = Medium.setSat_p(1e5 + 1e5*time);
            sat2 = Medium.setSat_T(300 + 50*time);
            Ts = Medium.saturationTemperature(1e5 + 1e5*time);
            ps = Medium.saturationPressure(300 + 50*time);
          end TestStatesSat;

          model TestBasePropertiesExplicit
            "Test case using TestMedium and BaseProperties with explicit equations"
            extends Modelica.Icons.Example;
            replaceable package Medium = Media.TestMedium;
            Media_Utilities.Test.TestMedium.GenericModels.CompleteBaseProperties
              medium1(redeclare package Medium = Medium)
              "Constant pressure, varying enthalpy";
            Media_Utilities.Test.TestMedium.GenericModels.CompleteBaseProperties
              medium2(redeclare package Medium = Medium)
              "Varying pressure, constant enthalpy";
          equation
            medium1.baseProperties.p = 1e5 + 1e5*time;
            medium1.baseProperties.h = 1e5;
            medium2.baseProperties.p = 1e5;
            medium2.baseProperties.h = 1e5 + 2e5*time;
          end TestBasePropertiesExplicit;

          model TestBasePropertiesImplicit
            "Test case using TestMedium and BaseProperties with implicit equations"
            replaceable package Medium = Media.TestMedium;
            extends Modelica.Icons.Example;
            Media_Utilities.Test.TestMedium.GenericModels.CompleteBaseProperties
              medium1(redeclare package Medium = Medium, baseProperties(h(start=
                     1e5))) "Constant pressure, varying enthalpy";
            Media_Utilities.Test.TestMedium.GenericModels.CompleteBaseProperties
              medium2(redeclare package Medium = Medium, baseProperties(h(start=
                     1e5))) "Varying pressure, constant enthalpy";
          equation
            medium1.baseProperties.p = 1e5*time;
            medium1.baseProperties.T = 300 + 25*time;
            medium2.baseProperties.p = 1e5 + 1e5*time;
            medium2.baseProperties.T = 300;
          end TestBasePropertiesImplicit;

          model TestBasePropertiesDynamic
            "Test case using TestMedium and dynamic equations"
            extends Modelica.Icons.Example;
            replaceable package Medium = Media.TestMedium;
            parameter SI.Volume V=1 "Storage Volume";
            parameter Real p_atm=101325 "Atmospheric pressure";
            parameter SI.Temperature Tstart=300;
            parameter Real Kv0=1.00801e-2 "Valve flow coefficient";
            Medium.BaseProperties medium(preferredMediumStates=true);
            SI.Mass M;
            SI.Energy U;
            SI.MassFlowRate win(start=100);
            SI.MassFlowRate wout;
            SI.SpecificEnthalpy hin;
            SI.SpecificEnthalpy hout;
            SI.Power Q;
            Real Kv;
          equation
            // Mass & energy balance equation
            M = medium.d*V;
            U = medium.u*M;
            der(M) = win - wout;
            der(U) = win*hin - wout*hout + Q;
            // Inlet pump equations
            medium.p - p_atm = 2e5 - (1e5/100^2)*win^2;
            hin = 1e5;
            // Outlet valve equation
            wout = Kv*sqrt(medium.d*(medium.p - p_atm));
            hout = medium.h;
            // Input variables
            Kv = if time < 50 then Kv0 else Kv0*1.1;
            Q = if time < 1 then 0 else 1e7;
          initial equation
            // Initial conditions
            // Fixed initial states
            // medium.p = 2e5;
            // medium.h = 1e5;
            // Steady state equations
            der(medium.p) = 0;
            der(medium.h) = 0;
            annotation (experiment(StopTime=80, Tolerance=1e-007),
                experimentSetupOutput(equdistant=false));
          end TestBasePropertiesDynamic;

          package GenericModels
            "Contains generic models to use for thorough medium model testing"
            extends Modelica.Icons.BasesPackage;
            model CompleteFluidConstants
              "Compute all available medium fluid constants"
              replaceable package Medium =
                  Modelica.Media.Interfaces.PartialTwoPhaseMedium;
              // Fluid constants
              Medium.Temperature Tc=Medium.fluidConstants[1].criticalTemperature;
              Medium.AbsolutePressure pc=Medium.fluidConstants[1].criticalPressure;
              Medium.MolarVolume vc=Medium.fluidConstants[1].criticalMolarVolume;
              Medium.MolarMass MM=Medium.fluidConstants[1].molarMass;
            end CompleteFluidConstants;

            model CompleteThermodynamicState
              "Compute all available two-phase medium properties from a ThermodynamicState model"
              replaceable package Medium =
                  Modelica.Media.Interfaces.PartialTwoPhaseMedium;
              // ThermodynamicState record
              input Medium.ThermodynamicState state;
              // Medium properties
              Medium.AbsolutePressure p=Medium.pressure(state);
              Medium.SpecificEnthalpy h=Medium.specificEnthalpy(state);
              Medium.Temperature T=Medium.temperature(state);
              Medium.Density d=Medium.density(state);
              Medium.SpecificEntropy s=Medium.specificEntropy(state);
              Medium.SpecificHeatCapacity cp=Medium.specificHeatCapacityCp(state);
              Medium.SpecificHeatCapacity cv=Medium.specificHeatCapacityCv(state);
              Medium.IsobaricExpansionCoefficient beta=
                  Medium.isobaricExpansionCoefficient(state);
              SI.IsothermalCompressibility kappa=Medium.isothermalCompressibility(
                  state);
              Medium.DerDensityByPressure d_d_dp_h=Medium.density_derp_h(state);
              Medium.DerDensityByEnthalpy d_d_dh_p=Medium.density_derh_p(state);
              Medium.MolarMass MM=Medium.molarMass(state);
            end CompleteThermodynamicState;

            model CompleteSaturationProperties
              "Compute all available saturation properties from a SaturationProperties record"
              replaceable package Medium =
                  Modelica.Media.Interfaces.PartialTwoPhaseMedium;
              // SaturationProperties record
              input Medium.SaturationProperties sat;
              // Saturation properties
              Medium.Temperature Ts=Medium.saturationTemperature_sat(sat);
              Medium.Density dl=Medium.bubbleDensity(sat);
              Medium.Density dv=Medium.dewDensity(sat);
              Medium.SpecificEnthalpy hl=Medium.bubbleEnthalpy(sat);
              Medium.SpecificEnthalpy hv=Medium.dewEnthalpy(sat);
              Real d_Ts_dp=Medium.saturationTemperature_derp_sat(sat);
              Real d_dl_dp=Medium.dBubbleDensity_dPressure(sat);
              Real d_dv_dp=Medium.dDewDensity_dPressure(sat);
              Real d_hl_dp=Medium.dBubbleEnthalpy_dPressure(sat);
              Real d_hv_dp=Medium.dDewEnthalpy_dPressure(sat);
            end CompleteSaturationProperties;

            model CompleteBubbleDewStates
              "Compute all available properties for dewpoint and bubble point states corresponding to a sat record"
              replaceable package Medium =
                  Modelica.Media.Interfaces.PartialTwoPhaseMedium;
              // SaturationProperties record
              input Medium.SaturationProperties sat;
              CompleteThermodynamicState dewStateOnePhase(state=Medium.setDewState(
                    sat, 1), redeclare package Medium = Medium);
              CompleteThermodynamicState dewStateTwoPhase(state=Medium.setDewState(
                    sat, 2), redeclare package Medium = Medium);
              CompleteThermodynamicState bubbleStateOnePhase(state=
                    Medium.setBubbleState(sat, 1), redeclare package Medium = Medium);
              CompleteThermodynamicState bubbleStateTwoPhase(state=
                    Medium.setBubbleState(sat, 2), redeclare package Medium = Medium);
            end CompleteBubbleDewStates;

            model CompleteBaseProperties
              "Compute all available two-phase medium properties from a BaseProperties model"
              replaceable package Medium =
                  Modelica.Media.Interfaces.PartialTwoPhaseMedium;
              // BaseProperties object
              Medium.BaseProperties baseProperties;
              // All the complete properties
              CompleteThermodynamicState completeState(redeclare package Medium =
                    Medium, state=baseProperties.state);
              CompleteSaturationProperties completeSat(redeclare package Medium =
                    Medium, sat=baseProperties.sat);
              CompleteFluidConstants completeConstants(redeclare package Medium =
                    Medium);
              CompleteBubbleDewStates completeBubbleDewStates(redeclare package
                  Medium = Medium, sat=baseProperties.sat);
            end CompleteBaseProperties;
          end GenericModels;

          model TestRunner "A model to collect generaic test cases"
            import ExternalMedia =
                   CHEETA_Thermal.Utilities.Media_Utilities;
            extends Modelica.Icons.Example;
            ExternalMedia.Test.GenericModels.CompleteFluidConstants
              completeFluidConstants(redeclare package Medium =
                  ExternalMedia.Media.TestMedium)
              annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
          equation

          end TestRunner;
        end TestMedium;

        package FluidProp "Test cases for FluidPropMedium"
          extends Modelica.Icons.ExamplesPackage;

          package WaterIF95 "Test suite for the FluidProp-Refprop IF95 medium model"
            extends Modelica.Icons.ExamplesPackage;
             model TestStates "Test case with state records"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
               extends Modelica.Icons.Example;
               extends ExternalMedia.Test.GenericModels.TestStates(
                                                redeclare package Medium =
                    ExternalMedia.Examples.WaterIF95);
             equation
              p1 = 1e5;
              h1 = 1e5 + 2e5*time;
              p2 = 1e5;
              T2 = 300 + 50*time;
             end TestStates;

            model TestStatesSat "Test case with state + sat records"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends ExternalMedia.Test.GenericModels.TestStatesSat(
                                                  redeclare package Medium =
                    ExternalMedia.Examples.WaterIF95);
            equation
              p1 = 1e5;
              h1 = 1e5 + 2e5*time;
              p2 = 1e5;
              T2 = 300 + 50*time;
            end TestStatesSat;

            model TestBasePropertiesExplicit
              "Test case using BaseProperties and explicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesExplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.WaterIF95);
            equation
              p1 = 1e5 + 1e5*time;
              h1 = 1e5;
              p2 = 1e5;
              h2 = 1e5 + 2e5*time;
            end TestBasePropertiesExplicit;

            model TestBasePropertiesImplicit
              "Test case using BaseProperties and implicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesImplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.WaterIF95, hstart=1e5);
            equation
              p1 = 1e5 + 1e5*time;
              T1 = 300 + 25*time;
              p2 = 1e5 + 1e5*time;
              T2 = 300;
            end TestBasePropertiesImplicit;

            model TestBasePropertiesDynamic
              "Test case using BaseProperties and dynamic equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesDynamic(
                redeclare package Medium = ExternalMedia.Examples.WaterIF95,
                Tstart=300,
                Kv0=1.00801e-2);
            equation
              // Inlet pump equations
              medium.p - p_atm = 2e5 - (1e5/100^2)*win^2;
              hin = 1e5;
              // Input variables
              Kv = if time < 50 then Kv0 else Kv0*1.1;
              Q = if time < 1 then 0 else 1e7;
              annotation (experiment(StopTime=80, Tolerance=1e-007),
                  experimentSetupOutput(equdistant=false));
            end TestBasePropertiesDynamic;

            model CompareModelicaFluidProp_liquid
              "Comparison between Modelica IF97 and FluidProp IF95 models - liquid"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.CompareModelicaTestMedium(
                redeclare package ModelicaMedium =
                    Modelica.Media.Water.StandardWater,
                redeclare package FluidPropMedium =
                    ExternalMedia.Examples.WaterIF95,
                pmin=1e5,
                pmax=1e5,
                hmin=1e5,
                hmax=4e5);
            end CompareModelicaFluidProp_liquid;

            model CompareModelicaFluidProp_twophase
              "Comparison between Modelica IF97 and FluidProp IF95 models - liquid"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.CompareModelicaTestMedium(
                redeclare package ModelicaMedium =
                    Modelica.Media.Water.StandardWater,
                redeclare package FluidPropMedium =
                    ExternalMedia.Examples.WaterIF95,
                pmin=60e5,
                pmax=60e5,
                hmin=1000e3,
                hmax=2000e3);
            end CompareModelicaFluidProp_twophase;

            model CompareModelicaFluidProp_vapour
              "Comparison between Modelica IF97 and FluidProp IF95 models - liquid"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.CompareModelicaTestMedium(
                redeclare package ModelicaMedium =
                    Modelica.Media.Water.StandardWater,
                redeclare package FluidPropMedium =
                    ExternalMedia.Examples.WaterIF95,
                pmin=60e5,
                pmax=60e5,
                hmin=2800e3,
                hmax=3200e3);
            end CompareModelicaFluidProp_vapour;
          end WaterIF95;

          package WaterIF97 "Test suite for the FluidProp IF97 medium model"
            extends Modelica.Icons.ExamplesPackage;
            model TestStates "Test case with state records"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends ExternalMedia.Test.GenericModels.TestStates(
                                               redeclare package Medium =
                    ExternalMedia.Examples.WaterIF97);
            equation
              p1 = 1e5;
              h1 = 1e5 + 2e5*time;
              p2 = 1e5;
              T2 = 300 + 50*time;
            end TestStates;

            model TestStatesSat "Test case with state + sat records"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends ExternalMedia.Test.GenericModels.TestStatesSat(
                                                  redeclare package Medium =
                    ExternalMedia.Examples.WaterIF97);
            equation
              p1 = 1e5;
              h1 = 1e5 + 2e5*time;
              p2 = 1e5;
              T2 = 300 + 50*time;
            end TestStatesSat;

            model TestBasePropertiesExplicit
              "Test case using BaseProperties and explicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesExplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.WaterIF97);
            equation
              p1 = 1e5 + 1e5*time;
              h1 = 1e5;
              p2 = 1e5;
              h2 = 1e5 + 2e5*time;
            end TestBasePropertiesExplicit;

            model TestBasePropertiesImplicit
              "Test case using BaseProperties and implicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesImplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.WaterIF97, hstart=1e5);
            equation
              p1 = 1e5 + 1e5*time;
              T1 = 300 + 25*time;
              p2 = 1e5 + 1e5*time;
              T2 = 300;
            end TestBasePropertiesImplicit;

            model TestBasePropertiesDynamic
              "Test case using BaseProperties and dynamic equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesDynamic(
                redeclare package Medium = ExternalMedia.Examples.WaterIF97,
                Tstart=300,
                Kv0=1.00801e-2);
            equation
              // Inlet pump equations
              medium.p - p_atm = 2e5 - (1e5/100^2)*win^2;
              hin = 1e5;
              // Input variables
              Kv = if time < 50 then Kv0 else Kv0*1.1;
              Q = if time < 1 then 0 else 1e7;
              annotation (experiment(StopTime=80, Tolerance=1e-007),
                  experimentSetupOutput(equdistant=false));
            end TestBasePropertiesDynamic;

            model CompareModelicaFluidProp_liquid
              "Comparison between Modelica IF97 and FluidProp IF97 models - liquid"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.CompareModelicaTestMedium(
                redeclare package ModelicaMedium =
                    Modelica.Media.Water.StandardWater,
                redeclare package FluidPropMedium =
                    ExternalMedia.Examples.WaterIF97,
                pmin=1e5,
                pmax=1e5,
                hmin=1e5,
                hmax=4e5);
            end CompareModelicaFluidProp_liquid;

            model CompareModelicaFluidProp_twophase
              "Comparison between Modelica IF97 and FluidProp IF97 models - liquid"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.CompareModelicaTestMedium(
                redeclare package ModelicaMedium =
                    Modelica.Media.Water.StandardWater,
                redeclare package FluidPropMedium =
                    ExternalMedia.Examples.WaterIF97,
                pmin=60e5,
                pmax=60e5,
                hmin=1000e3,
                hmax=2000e3);
            end CompareModelicaFluidProp_twophase;

            model CompareModelicaFluidProp_vapour
              "Comparison between Modelica IF97 and FluidProp IF97 models - liquid"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.CompareModelicaTestMedium(
                redeclare package ModelicaMedium =
                    Modelica.Media.Water.StandardWater,
                redeclare package FluidPropMedium =
                    ExternalMedia.Examples.WaterIF97,
                pmin=60e5,
                pmax=60e5,
                hmin=2800e3,
                hmax=3200e3);
            end CompareModelicaFluidProp_vapour;
          end WaterIF97;

          package WaterTPSI "Test suite for the FluidProp TPSI water medium model"
            extends Modelica.Icons.ExamplesPackage;
            model TestStates "Test case with state records"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends ExternalMedia.Test.GenericModels.TestStates(
                                               redeclare package Medium =
                    ExternalMedia.Examples.WaterTPSI);
            equation
              p1 = 1e5;
              h1 = 1e5 + 2e5*time;
              p2 = 1e5;
              T2 = 300 + 50*time;
            end TestStates;

            model TestStatesSat "Test case with state + sat records"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends ExternalMedia.Test.GenericModels.TestStatesSat(
                                                  redeclare package Medium =
                    ExternalMedia.Examples.WaterTPSI);
            equation
              p1 = 1e5;
              h1 = 1e5 + 2e5*time;
              p2 = 1e5;
              T2 = 300 + 50*time;
            end TestStatesSat;

            model TestBasePropertiesExplicit
              "Test case using BaseProperties and explicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesExplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.WaterTPSI);
            equation
              p1 = 1e5 + 1e5*time;
              h1 = 1e5;
              p2 = 1e5;
              h2 = 1e5 + 2e5*time;
            end TestBasePropertiesExplicit;

            model TestBasePropertiesImplicit
              "Test case using BaseProperties and implicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesImplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.WaterTPSI, hstart=1e5);
            equation
              p1 = 1e5 + 1e5*time;
              T1 = 300 + 25*time;
              p2 = 1e5 + 1e5*time;
              T2 = 300;
            end TestBasePropertiesImplicit;

            model TestBasePropertiesDynamic
              "Test case using BaseProperties and dynamic equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesDynamic(
                redeclare package Medium = ExternalMedia.Examples.WaterTPSI,
                Tstart=300,
                Kv0=1.00801e-2);
            equation
              // Inlet pump equations
              medium.p - p_atm = 2e5 - (1e5/100^2)*win^2;
              hin = 1e5;
              // Input variables
              Kv = if time < 50 then Kv0 else Kv0*1.1;
              Q = if time < 1 then 0 else 1e7;
              annotation (experiment(StopTime=80, Tolerance=1e-007),
                  experimentSetupOutput(equdistant=false));
            end TestBasePropertiesDynamic;

            model CompareModelicaFluidProp_liquid
              "Comparison between Modelica IF97 and FluidProp TPSI models - liquid"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.CompareModelicaTestMedium(
                redeclare package ModelicaMedium =
                    Modelica.Media.Water.StandardWater,
                redeclare package FluidPropMedium =
                    ExternalMedia.Examples.WaterTPSI,
                pmin=1e5,
                pmax=1e5,
                hmin=1e5,
                hmax=4e5);
            end CompareModelicaFluidProp_liquid;

            model CompareModelicaFluidProp_twophase
              "Comparison between Modelica IF97 and FluidProp TPSI models - liquid"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.CompareModelicaTestMedium(
                redeclare package ModelicaMedium =
                    Modelica.Media.Water.StandardWater,
                redeclare package FluidPropMedium =
                    ExternalMedia.Examples.WaterTPSI,
                pmin=60e5,
                pmax=60e5,
                hmin=1000e3,
                hmax=2000e3);
            end CompareModelicaFluidProp_twophase;

            model CompareModelicaFluidProp_vapour
              "Comparison between Modelica IF97 and FluidProp TPSI models - liquid"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.CompareModelicaTestMedium(
                redeclare package ModelicaMedium =
                    Modelica.Media.Water.StandardWater,
                redeclare package FluidPropMedium =
                    ExternalMedia.Examples.WaterTPSI,
                pmin=60e5,
                pmax=60e5,
                hmin=2800e3,
                hmax=3200e3);
            end CompareModelicaFluidProp_vapour;
          end WaterTPSI;

          package CO2StanMix "Test suite for the StanMix CO2 medium model"
            extends Modelica.Icons.ExamplesPackage;
            model TestStatesSupercritical
              "Test case with state records, supercritical conditions"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends ExternalMedia.Test.GenericModels.TestStates(
                                               redeclare package Medium =
                    ExternalMedia.Examples.CO2StanMix);
            equation
              p1 = 8e6;
              h1 = -4.2e5 + 6e5*time;
              p2 = 8e6;
              T2 = 280 + 50*time;
            end TestStatesSupercritical;

            model TestStatesTranscritical
              "Test case with state records, transcritical conditions"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends ExternalMedia.Test.GenericModels.TestStates(
                                               redeclare package Medium =
                    ExternalMedia.Examples.CO2StanMix);
            equation
              p1 = 1e6 + time*10e6;
              h1 = -4.2e5 + 0*time;
              p2 = 1e6 + time*10e6;
              T2 = 330;
            end TestStatesTranscritical;

            model TestStatesSatSubcritical
              "Test case with state + sat records, subcritical conditions"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends ExternalMedia.Test.GenericModels.TestStatesSat(
                                                  redeclare package Medium =
                    ExternalMedia.Examples.CO2StanMix);
            equation
              p1 = 1e6;
              h1 = -4.2e5 + 6e5*time;
              p2 = 1e6;
              T2 = 250 + 50*time;
            end TestStatesSatSubcritical;

            model TestBasePropertiesExplicit
              "Test case using BaseProperties and explicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesExplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.CO2StanMix);
            equation
              p1 = 8e6;
              h1 = -4.2e5 + 6e5*time;
              p2 = 1e6;
              h2 = -4.2e5 + 6e5*time;
            end TestBasePropertiesExplicit;

            model TestBasePropertiesImplicit
              "Test case using BaseProperties and implicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesImplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.CO2StanMix, hstart=0);
            equation
              p1 = 8e6;
              T1 = 280 + 20*time;
              p2 = 1e6;
              T2 = 280 + 20*time;
            end TestBasePropertiesImplicit;

            model TestBasePropertiesDynamic
              "Test case using BaseProperties and dynamic equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesDynamic(
                redeclare package Medium = ExternalMedia.Examples.CO2StanMix,
                Tstart=300,
                hstart=0,
                pstart=1e6,
                Kv0=1.00801e-4,
                V=0.1);
            equation
              // Inlet equations
              win = 1;
              hin = 0;
              // Input variables
              Kv = if time < 50 then Kv0 else Kv0*1.1;
              Q = if time < 1 then 0 else 1e4;
              annotation (experiment(StopTime=80, Tolerance=1e-007),
                  experimentSetupOutput(equdistant=false));
            end TestBasePropertiesDynamic;

            model TestBasePropertiesTranscritical
              "Test case using BaseProperties and explicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesExplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.CO2StanMix);
            equation
              p1 = 1e6 + time*10e6;
              h1 = -4.2e5 + 0*time;
              p2 = 1e6 + time*10e6;
              h2 = 2.0e5;
            end TestBasePropertiesTranscritical;
          end CO2StanMix;

          package CO2RefProp "Test suite for the REFPROP CO2 medium model"
            extends Modelica.Icons.ExamplesPackage;
            model TestStatesSupercritical
              "Test case with state records, supercritical conditions"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends ExternalMedia.Test.GenericModels.TestStates(
                                               redeclare package Medium =
                    ExternalMedia.Examples.CO2RefProp);
            equation
              p1 = 8e6;
              h1 = 1.0e5 + 6e5*time;
              p2 = 8e6;
              T2 = 280 + 50*time;
            end TestStatesSupercritical;

            model TestStatesTranscritical
              "Test case with state records, transcritical conditions"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends ExternalMedia.Test.GenericModels.TestStates(
                                               redeclare package Medium =
                    ExternalMedia.Examples.CO2RefProp);
            equation
              p1 = 1e6 + time*10e6;
              h1 = 1.0e5;
              p2 = 1e6 + time*10e6;
              T2 = 330;
            end TestStatesTranscritical;

            model TestStatesSatSubcritical
              "Test case state + sat records, subcritical conditions"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends ExternalMedia.Test.GenericModels.TestStatesSat(
                                                  redeclare package Medium =
                    ExternalMedia.Examples.CO2RefProp);
            equation
              p1 = 1e6;
              h1 = 1.0e5 + 6e5*time;
              p2 = 1e6;
              T2 = 250 + 50*time;
            end TestStatesSatSubcritical;

            model TestBasePropertiesExplicit
              "Test case using BaseProperties and explicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesExplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.CO2RefProp);
            equation
              p1 = 8e6;
              h1 = 1.0e5 + 6e5*time;
              p2 = 1e6;
              h2 = 1.0e5 + 6e5*time;
            end TestBasePropertiesExplicit;

            model TestBasePropertiesImplicit
              "Test case using BaseProperties and implicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesImplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.CO2RefProp, hstart=1e5);
            equation
              p1 = 8e6;
              T1 = 280 + 50*time;
              p2 = 1e6;
              T2 = 280 + 50*time;
            end TestBasePropertiesImplicit;

            model TestBasePropertiesDynamic
              "Test case using BaseProperties and dynamic equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesDynamic(
                redeclare package Medium = ExternalMedia.Examples.CO2RefProp,
                Tstart=300,
                hstart=4e5,
                pstart=1e6,
                Kv0=1.00801e-4,
                V=0.1);
            equation
              // Inlet equations
              win = 1;
              hin = 5e5;
              // Input variables
              Kv = if time < 50 then Kv0 else Kv0*1.1;
              Q = if time < 1 then 0 else 1e4;
              annotation (experiment(StopTime=80, Tolerance=1e-007),
                  experimentSetupOutput(equdistant=false));
            end TestBasePropertiesDynamic;

            model TestBasePropertiesTranscritical
              "Test case using BaseProperties and explicit equations"
              import ExternalMedia =
                     CHEETA_Thermal.Utilities.Media_Utilities;
              extends Modelica.Icons.Example;
              extends
                ExternalMedia.Test.GenericModels.TestBasePropertiesExplicit(
                                                               redeclare
                  package Medium =
                           ExternalMedia.Examples.CO2RefProp);
            equation
              p1 = 1e6 + time*10e6;
              h1 = 1.0e5;
              p2 = 1e6 + time*10e6;
              h2 = 7.0e5;
            end TestBasePropertiesTranscritical;
          end CO2RefProp;
        end FluidProp;

        package CoolProp "Test cases for CoolProp"
          extends Modelica.Icons.ExamplesPackage;
          package CO2 "Test suite for the CoolProp CO2 medium model"
             extends Modelica.Icons.ExamplesPackage;
            model TestStatesSupercritical
              "Test case with state records, supercritical conditions"
              extends Modelica.Icons.Example;
              extends GenericModels.TestStates(          redeclare package
                  Medium =
                    Media_Utilities.Examples.CO2CoolProp);
            equation
              p1 = 8e6;
              h1 = 1.0e5 + 6e5*time;
              p2 = 8e6;
              T2 = 280 + 50*time;
            end TestStatesSupercritical;

            model TestStatesTranscritical
              "Test case with state records, transcritical conditions"
              extends Modelica.Icons.Example;
              extends GenericModels.TestStates(          redeclare package
                  Medium =
                    Media_Utilities.Examples.CO2CoolProp);
            equation
              p1 = 1e6 + time*10e6;
              h1 = 1.0e5;
              p2 = 1e6 + time*10e6;
              T2 = 330;
            end TestStatesTranscritical;

            model TestStatesSatSubcritical
              "Test case state + sat records, subcritical conditions"
              extends Modelica.Icons.Example;
              extends GenericModels.TestStatesSat(          redeclare package
                  Medium =
                    Media_Utilities.Examples.CO2CoolProp);
            equation
              p1 = 1e6;
              h1 = 1.0e5 + 6e5*time;
              p2 = 1e6;
              T2 = 250 + 50*time;
            end TestStatesSatSubcritical;

            model TestBasePropertiesExplicit
              "Test case using BaseProperties and explicit equations"
              extends Modelica.Icons.Example;
              extends GenericModels.TestBasePropertiesExplicit(          redeclare
                  package Medium = Media_Utilities.Examples.CO2CoolProp);
            equation
              p1 = 8e6;
              h1 = 1.0e5 + 6e5*time;
              p2 = 1e6;
              h2 = 1.0e5 + 6e5*time;
            end TestBasePropertiesExplicit;

            model TestBasePropertiesImplicit
              "Test case using BaseProperties and implicit equations"
              extends Modelica.Icons.Example;
              extends GenericModels.TestBasePropertiesImplicit(          redeclare
                  package Medium = Media_Utilities.Examples.CO2CoolProp,
                                                                       hstart=1e5);
            equation
              p1 = 8e6;
              T1 = 280 + 50*time;
              p2 = 1e6;
              T2 = 280 + 50*time;
            end TestBasePropertiesImplicit;

            model TestBasePropertiesDynamic
              "Test case using BaseProperties and dynamic equations"
              extends Modelica.Icons.Example;
              extends GenericModels.TestBasePropertiesDynamic(
                redeclare package Medium = Media_Utilities.Examples.CO2CoolProp,
                Tstart=300,
                hstart=4e5,
                pstart=1e6,
                Kv0=1.00801e-4,
                V=0.1);
            equation
              // Inlet equations
              win = 1;
              hin = 5e5;
              // Input variables
              Kv = if time < 50 then Kv0 else Kv0*1.1;
              Q = if time < 1 then 0 else 1e4;
              annotation (experiment(StopTime=80, Tolerance=1e-007),
                  experimentSetupOutput(equdistant=false));
            end TestBasePropertiesDynamic;

            model TestBasePropertiesTranscritical
              "Test case using BaseProperties and explicit equations"
              extends Modelica.Icons.Example;
              extends GenericModels.TestBasePropertiesExplicit(          redeclare
                  package Medium = Media_Utilities.Examples.CO2CoolProp);
            equation
              p1 = 1e6 + time*10e6;
              h1 = 1.0e5;
              p2 = 1e6 + time*10e6;
              h2 = 7.0e5;
            end TestBasePropertiesTranscritical;

          end CO2;

          model Pentane_hs
          package wf
            extends Media_Utilities.Media.CoolPropMedium(
                mediumName="Pentane",
                substanceNames={"n-Pentane"},
                inputChoice=Media_Utilities.Common.InputChoice.hs);
          end wf;
            wf.BaseProperties fluid "Properties of the two-phase fluid";
            Modelica.SIunits.SpecificEnthalpy h;
            Modelica.SIunits.Pressure p;
            Modelica.SIunits.SpecificEntropy s;
            Modelica.SIunits.DerDensityByEnthalpy drdh
              "Derivative of average density by enthalpy";
            Modelica.SIunits.DerDensityByPressure drdp
              "Derivative of average density by pressure";
          equation
            //p = 1E5;
            h = 0 + time*1E6;
            s = 1500;  //600 + time*2000;
            fluid.p = p;
            fluid.s = s;
            fluid.h = h;
            drdp = wf.density_derp_h(fluid.state);
            drdh = wf.density_derh_p(fluid.state);
          end Pentane_hs;

          model Pentane_hs_state
          package wf
            extends Media_Utilities.Media.CoolPropMedium(
                mediumName="Pentane",
                substanceNames={"n-Pentane"},
                inputChoice=Media_Utilities.Common.InputChoice.hs);
          end wf;
            wf.ThermodynamicState fluid "Properties of the two-phase fluid";
            Modelica.SIunits.SpecificEnthalpy h;
            Modelica.SIunits.Pressure p;
            Modelica.SIunits.SpecificEntropy s;
            Modelica.SIunits.DerDensityByEnthalpy drdh
              "Derivative of average density by enthalpy";
            Modelica.SIunits.DerDensityByPressure drdp
              "Derivative of average density by pressure";
          equation
            //p = 1E5;
            h = 0 + time*1E6;
            s = 600 + time*2000;
            fluid = wf.setState_hs(h,s);
            fluid.p = p;
            drdp = wf.density_derp_h(fluid);
            drdh = wf.density_derh_p(fluid);
          end Pentane_hs_state;

          model MSL_Models
            import ExternalMedia =
                   CHEETA_Thermal.Utilities.Media_Utilities;
            extends Modelica.Icons.Example;

            ExternalMedia.Test.MSL_Models.BranchingDynamicPipes branchingDynamicPipes(
                redeclare package NewMedium =
                  ExternalMedia.Examples.WaterCoolProp)
              annotation (Placement(transformation(extent={{-50,20},{-30,40}})));
          end MSL_Models;

          model CheckOptions
            extends Modelica.Icons.Example;
            String test;
          algorithm
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|enable_TTSE");
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|enable_TTSE=0");
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|enable_TTSE=1");
            //
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|enable_BICUBIC");
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|enable_BICUBIC=0");
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|enable_BICUBIC=1");
            //
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|enable_EXTTP");
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|enable_EXTTP=0");
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|enable_EXTTP=1");
            //
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|twophase_derivsmoothing_xend");
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|twophase_derivsmoothing_xend=0.0");
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|twophase_derivsmoothing_xend=0.1");
            //
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|rho_smoothing_xend");
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|rho_smoothing_xend=0.0");
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|rho_smoothing_xend=0.1");
            //
            test := Media_Utilities.Common.CheckCoolPropOptions("LiBr|debug");
            test := Media_Utilities.Common.CheckCoolPropOptions("LiBr|debug=0");
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|debug=100");
            //
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|enable_TTSE=1|debug=0|enable_EXTTP", debug=true);
            test := Media_Utilities.Common.CheckCoolPropOptions(
              "LiBr|enable_TTSE=1|debug=0|enableEXTTP=1");
          end CheckOptions;

          package Incompressible
                                 extends Modelica.Icons.ExamplesPackage;
            model incompressibleCoolPropMedium
                                               extends Modelica.Icons.Example;

            package DowQ_CP "DowthermQ properties from CoolProp"
              extends Media_Utilities.Media.IncompressibleCoolPropMedium(
                  mediumName="DowQ",
                  substanceNames={"DowQ|calc_transport=1|debug=1000"},
                  ThermoStates=Modelica.Media.Interfaces.PartialMedium.Choices.IndependentVariables.pT);
            end DowQ_CP;

            //replaceable package Fluid = ExternalMedia.Examples.WaterCoolProp (
            //  ThermoStates = Modelica.Media.Interfaces.Choices.IndependentVariables.pTX) constrainedby
            //    Modelica.Media.Interfaces.PartialMedium "Medium model";

            replaceable package Fluid =  DowQ_CP constrainedby
                Modelica.Media.Interfaces.PartialMedium "Medium model";
              Fluid.ThermodynamicState state;
              Fluid.Temperature T;
              Fluid.AbsolutePressure p;
              Fluid.BaseProperties props;

            equation
              p     = 10E5;
              T     = 273.15 + 15.0 + time * 50.0;
              state = Fluid.setState_pT(p,T);
              // And now we do some testing with the BaseProperties
              props.T = T;
              props.p = p;
            end incompressibleCoolPropMedium;

            model incompressibleCoolPropMixture
                                               extends Modelica.Icons.Example;

            package LiBr_CP "Lithium bromide solution properties from CoolProp"
              extends Media_Utilities.Media.IncompressibleCoolPropMedium(
                  mediumName="LiBr",
                  substanceNames={"LiBr|calc_transport=1|debug=1000",
                      "dummyToMakeBasePropertiesWork"},
                  ThermoStates=Modelica.Media.Interfaces.PartialMedium.Choices.IndependentVariables.pTX);
            end LiBr_CP;

            replaceable package Fluid = LiBr_CP constrainedby
                Modelica.Media.Interfaces.PartialMedium "Medium model";
              Fluid.ThermodynamicState state_var;
              Fluid.ThermodynamicState state_con;
              Fluid.Temperature T;
              Fluid.AbsolutePressure p;
              Fluid.MassFraction[1] X_var;
              Fluid.MassFraction[1] X_con;
              Fluid.BaseProperties varProps;

            equation
              p         = 10E5;
              T         = 273.15 + 15.0 + time * 50.0;
              X_var[1]  =   0.00 +  0.1 + time *  0.5;
              X_con[1]  =   0.00 +  0.1;
              state_var = Fluid.setState_pTX(p,T,X_var);
              state_con = Fluid.setState_pTX(p,T,X_con);
              // And now we do some testing with the BaseProperties
              varProps.T = T;
              varProps.p = p;
              varProps.Xi = X_var;
            end incompressibleCoolPropMixture;
          end Incompressible;

          model RhoSmoothing
            extends Modelica.Icons.Example;
          package fluid_std
            extends Media_Utilities.Media.CoolPropMedium(
                mediumName="Pentane",
                substanceNames={"n-Pentane|rho_smoothing_xend=0.0"},
                inputChoice=Media_Utilities.Common.InputChoice.ph);
          end fluid_std;

          package fluid_spl
            extends Media_Utilities.Media.CoolPropMedium(
                mediumName="Pentane",
                substanceNames={"n-Pentane|rho_smoothing_xend=0.2"},
                inputChoice=Media_Utilities.Common.InputChoice.ph);
          end fluid_spl;

          fluid_std.ThermodynamicState state_std "Properties of the two-phase fluid";
          fluid_spl.ThermodynamicState state_spl "Properties of the two-phase fluid";

          Modelica.SIunits.AbsolutePressure p;
          Modelica.SIunits.SpecificEnthalpy h;

          fluid_std.SaturationProperties sat_std;

          Modelica.SIunits.SpecificEnthalpy h_start;
          Modelica.SIunits.SpecificEnthalpy h_end;
          Modelica.SIunits.SpecificEnthalpy h_delta;

          Modelica.SIunits.Time t = 1;

          equation
            p = 10E5;
            sat_std = fluid_std.setSat_p(p);
            h_start = fluid_std.bubbleEnthalpy(sat_std);
            h_end = fluid_std.dewEnthalpy(sat_std);
            h_delta = 3e3;
            h = (h_start - h_delta) + (h_end-h_start+2*h_delta)*time/t;
            state_std = fluid_std.setState_ph(p,h);
            state_spl = fluid_spl.setState_ph(p,h);
          end RhoSmoothing;
        end CoolProp;

        package WrongMedium "Test cases with wrong medium models"
          extends Modelica.Icons.ExamplesPackage;
          model TestWrongMedium
            "Test the error reporting messages for unsupported external media"
            extends Modelica.Icons.Example;
            package Medium = Media.BaseClasses.ExternalTwoPhaseMedium;
            Medium.BaseProperties medium;
          equation
            medium.p = 1e5;
            medium.h = 1e5;
          end TestWrongMedium;
        end WrongMedium;

        package TestOMC "Test cases for OpenModelica implementation"
          extends Modelica.Icons.ExamplesPackage;
          package TestHelium
            "Test for NIST Helium model using ExternalMedia and FluidProp"
            extends Modelica.Icons.ExamplesPackage;
            package Helium "Helium model from NIST RefProp database"
              extends Media_Utilities.Media.BaseClasses.ExternalTwoPhaseMedium(
                mediumName="Helium",
                libraryName="FluidProp.RefProp",
                substanceNames={"He"},
                ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.ph,
                AbsolutePressure(
                  min=500,
                  max=44e5,
                  nominal=1e5,
                  start=1e5),
                Density(
                  min=0.1,
                  max=200,
                  nominal=100,
                  start=100),
                SpecificEnthalpy(
                  min=-6000,
                  max=1.7e6,
                  nominal=1000,
                  start=0),
                SpecificEntropy(
                  min=-4000,
                  max=30e3,
                  nominal=1000,
                  start=0),
                Temperature(
                  min=2.17,
                  max=310,
                  nominal=10,
                  start=5,
                  displayUnit="K"));

            end Helium;

            model TestSupercriticalHelium
              extends Modelica.Icons.Example;
              package Medium = Helium;
              Medium.ThermodynamicState state;
              Medium.Temperature T;
              Medium.Temperature Tcrit=Medium.fluidConstants[1].criticalTemperature;
              Medium.AbsolutePressure p;
              Modelica.SIunits.Density d;
              Medium.AbsolutePressure pcrit=Medium.fluidConstants[1].criticalPressure;
            equation
              T = 300 - 297.5*time;
              p = 4e5 + 0*time;
              state = Medium.setState_pT(p, T);
              d = Medium.density(state);
            end TestSupercriticalHelium;

            model TestSaturatedHelium
              extends Modelica.Icons.Example;
              package Medium = Helium;
              Medium.SaturationProperties sat;
              Medium.Temperature T;
              Medium.AbsolutePressure p;
              Modelica.SIunits.Density dl;
              Modelica.SIunits.Density dv;
            equation
              p = 1e5 + 1.27e5*time;
              sat = Medium.setSat_p(p);
              dv = Medium.dewDensity(sat);
              dl = Medium.bubbleDensity(sat);
              T = Medium.saturationTemperature_sat(sat);
            end TestSaturatedHelium;

            model TypicalHeliumProperties
              extends Modelica.Icons.Example;
              package Medium = Helium;
              Medium.ThermodynamicState state;
              Medium.Temperature T;
              Medium.Temperature Tcrit=Medium.fluidConstants[1].criticalTemperature;
              Medium.AbsolutePressure p;
              Modelica.SIunits.Density d;
              Medium.AbsolutePressure pcrit=Medium.fluidConstants[1].criticalPressure;
              Modelica.SIunits.SpecificHeatCapacity cv=Medium.specificHeatCapacityCv(
                  state);
            equation
              T = 5;
              p = 5e5;
              state = Medium.setState_pT(p, T);
              d = Medium.density(state);
            end TypicalHeliumProperties;
          end TestHelium;

          package TestHeliumHardCodedProperties
            "Test for NIST Helium model using ExternalMedia and FluidProp, hard-coded fluid properties package constants"
            extends Modelica.Icons.ExamplesPackage;
            package Helium "Helium model from NIST RefProp database"
              extends Media_Utilities.Media.BaseClasses.ExternalTwoPhaseMedium(
                mediumName="Helium",
                libraryName="FluidProp.RefProp",
                substanceNames={"He"},
                externalFluidConstants=FluidConstants(
                                iupacName="unknown",
                                casRegistryNumber="unknown",
                                chemicalFormula="unknown",
                                structureFormula="unknown",
                                molarMass=4.0026e-3,
                                criticalTemperature=5.1953,
                                criticalPressure=2.2746e5,
                                criticalMolarVolume=1/69.641*4.0026e-3,
                                acentricFactor=0,
                                triplePointTemperature=280.0,
                                triplePointPressure=500.0,
                                meltingPoint=280,
                                normalBoilingPoint=380.0,
                                dipoleMoment=2.0),
                ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.ph,
                AbsolutePressure(
                  min=500,
                  max=44e5,
                  nominal=1e5,
                  start=1e5),
                Density(
                  min=0.1,
                  max=200,
                  nominal=100,
                  start=100),
                SpecificEnthalpy(
                  min=-6000,
                  max=1.7e6,
                  nominal=1000,
                  start=0),
                SpecificEntropy(
                  min=-4000,
                  max=30e3,
                  nominal=1000,
                  start=0),
                Temperature(
                  min=2.17,
                  max=310,
                  nominal=10,
                  start=5,
                  displayUnit="K"));

            end Helium;

            model TestSupercriticalHelium
              extends Modelica.Icons.Example;
              package Medium = Helium;
              Medium.ThermodynamicState state;
              Medium.Temperature T;
              Medium.Temperature Tcrit=Medium.fluidConstants[1].criticalTemperature;
              Medium.AbsolutePressure p;
              Modelica.SIunits.Density d;
              Medium.AbsolutePressure pcrit=Medium.fluidConstants[1].criticalPressure;
            equation
              T = 300 - 297.5*time;
              p = 4e5 + 0*time;
              state = Medium.setState_pT(p, T);
              d = Medium.density(state);
            end TestSupercriticalHelium;

            model TestSaturatedHelium
              extends Modelica.Icons.Example;
              package Medium = Helium;
              Medium.SaturationProperties sat;
              Medium.Temperature T;
              Medium.AbsolutePressure p;
              Modelica.SIunits.Density dl;
              Modelica.SIunits.Density dv;
            equation
              p = 1e5 + 1.27e5*time;
              sat = Medium.setSat_p(p);
              dv = Medium.dewDensity(sat);
              dl = Medium.bubbleDensity(sat);
              T = Medium.saturationTemperature_sat(sat);
            end TestSaturatedHelium;

            model TypicalHeliumProperties
              extends Modelica.Icons.Example;
              package Medium = Helium;
              Medium.ThermodynamicState state;
              Medium.Temperature T;
              Medium.Temperature Tcrit=Medium.fluidConstants[1].criticalTemperature;
              Medium.AbsolutePressure p;
              Modelica.SIunits.Density d;
              Medium.AbsolutePressure pcrit=Medium.fluidConstants[1].criticalPressure;
              Modelica.SIunits.SpecificHeatCapacity cv=Medium.specificHeatCapacityCv(
                  state);
            equation
              T = 5;
              p = 5e5;
              state = Medium.setState_pT(p, T);
              d = Medium.density(state);
            end TypicalHeliumProperties;
          end TestHeliumHardCodedProperties;
        end TestOMC;

        package GenericModels "Generic models for FluidProp media tests"
          extends Modelica.Icons.BasesPackage;
          package DummyTwoPhaseMedium "A dummy to allow for pedantic checking"
            extends Modelica.Media.Water.WaterIF97_ph;
          end DummyTwoPhaseMedium;

          model CompleteFluidConstants "Compute all available medium fluid constants"
            replaceable package Medium =
                Media_Utilities.Test.GenericModels.DummyTwoPhaseMedium
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);
            // Fluid constants
            Medium.Temperature Tc=Medium.fluidConstants[1].criticalTemperature;
            Medium.AbsolutePressure pc=Medium.fluidConstants[1].criticalPressure;
            Medium.MolarVolume vc=Medium.fluidConstants[1].criticalMolarVolume;
            Medium.MolarMass MM=Medium.fluidConstants[1].molarMass;
          end CompleteFluidConstants;

          model CompleteThermodynamicState
            "Compute all available two-phase medium properties from a ThermodynamicState model"
            replaceable package Medium =
                Media_Utilities.Test.GenericModels.DummyTwoPhaseMedium
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);
            // ThermodynamicState record
            input Medium.ThermodynamicState state;
            // Medium properties
            Medium.AbsolutePressure p=Medium.pressure(state);
            Medium.SpecificEnthalpy h=Medium.specificEnthalpy(state);
            Medium.Temperature T=Medium.temperature(state);
            Medium.Density d=Medium.density(state);
            Medium.SpecificEntropy s=Medium.specificEntropy(state);
            Medium.SpecificHeatCapacity cp=Medium.specificHeatCapacityCp(state);
            Medium.SpecificHeatCapacity cv=Medium.specificHeatCapacityCv(state);
            Medium.IsobaricExpansionCoefficient beta=Medium.isobaricExpansionCoefficient(state);
            SI.IsothermalCompressibility kappa=Medium.isothermalCompressibility(state);
            Medium.DerDensityByPressure d_d_dp_h=Medium.density_derp_h(state);
            Medium.DerDensityByEnthalpy d_d_dh_p=Medium.density_derh_p(state);
            Medium.MolarMass MM=Medium.molarMass(state);
          end CompleteThermodynamicState;

          model CompleteSaturationProperties
            "Compute all available saturation properties from a SaturationProperties record"
            replaceable package Medium =
                Media_Utilities.Test.GenericModels.DummyTwoPhaseMedium
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);
            // SaturationProperties record
            input Medium.SaturationProperties sat;
            // Saturation properties
            Medium.Temperature Ts=Medium.saturationTemperature_sat(sat);
            Medium.Density dl=Medium.bubbleDensity(sat);
            Medium.Density dv=Medium.dewDensity(sat);
            Medium.SpecificEnthalpy hl=Medium.bubbleEnthalpy(sat);
            Medium.SpecificEnthalpy hv=Medium.dewEnthalpy(sat);
            Real d_Ts_dp=Medium.saturationTemperature_derp_sat(sat);
            Real d_dl_dp=Medium.dBubbleDensity_dPressure(sat);
            Real d_dv_dp=Medium.dDewDensity_dPressure(sat);
            Real d_hl_dp=Medium.dBubbleEnthalpy_dPressure(sat);
            Real d_hv_dp=Medium.dDewEnthalpy_dPressure(sat);
          end CompleteSaturationProperties;

          model CompleteBubbleDewStates
            "Compute all available properties for dewpoint and bubble point states corresponding to a sat record"
            replaceable package Medium =
                Media_Utilities.Test.GenericModels.DummyTwoPhaseMedium
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);
            // SaturationProperties record
            input Medium.SaturationProperties sat;
            // and the rest based on sat
            CompleteThermodynamicState dewStateOnePhase(state=Medium.setDewState(sat, 1),
                redeclare package Medium = Medium);
            CompleteThermodynamicState dewStateTwoPhase(state=Medium.setDewState(sat, 2),
                redeclare package Medium = Medium);
            CompleteThermodynamicState bubbleStateOnePhase(state=Medium.setBubbleState(
                  sat, 1), redeclare package Medium = Medium);
            CompleteThermodynamicState bubbleStateTwoPhase(state=Medium.setBubbleState(
                  sat, 2), redeclare package Medium = Medium);
          end CompleteBubbleDewStates;

          model CompleteBaseProperties
            "Compute all available two-phase medium properties from a BaseProperties model"
            replaceable package Medium =
                Media_Utilities.Test.GenericModels.DummyTwoPhaseMedium
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);
            // BaseProperties object
            Medium.BaseProperties baseProperties;
            // All the complete properties
            CompleteThermodynamicState completeState(redeclare package Medium = Medium,
                state=baseProperties.state);
            CompleteSaturationProperties completeSat(redeclare package Medium = Medium,
                sat=baseProperties.sat);
            CompleteFluidConstants completeConstants(redeclare package Medium = Medium);
          end CompleteBaseProperties;

          partial model TestStates "Test case with state"
            replaceable package Medium =
                Media_Utilities.Test.GenericModels.DummyTwoPhaseMedium
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);
            Medium.AbsolutePressure p1;
            Medium.SpecificEnthalpy h1;
            Medium.AbsolutePressure p2;
            Medium.Temperature T2;
            Medium.ThermodynamicState state1;
            Medium.ThermodynamicState state2;
            CompleteThermodynamicState completeState1(redeclare package Medium = Medium,
                state=state1);
            CompleteThermodynamicState completeState2(redeclare package Medium = Medium,
                state=state2);
          equation
            state1 = Medium.setState_ph(p1, h1);
            state2 = Medium.setState_pT(p2, T2);
          end TestStates;

          partial model TestStatesSat "Test case with state + sat records"
            replaceable package Medium =
                Media_Utilities.Test.GenericModels.DummyTwoPhaseMedium
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);
            Medium.AbsolutePressure p1;
            Medium.SpecificEnthalpy h1;
            Medium.AbsolutePressure p2;
            Medium.Temperature T2;
            Medium.ThermodynamicState state1;
            Medium.ThermodynamicState state2;
            Medium.SaturationProperties sat1;
            Medium.SaturationProperties sat2;
            Medium.Temperature Ts;
            Medium.AbsolutePressure ps;
            CompleteThermodynamicState completeState1(redeclare package Medium = Medium,
                state=state1);
            CompleteThermodynamicState completeState2(redeclare package Medium = Medium,
                state=state2);
            CompleteSaturationProperties completeSat1(redeclare package Medium = Medium,
                sat=sat1);
            CompleteSaturationProperties completeSat2(redeclare package Medium = Medium,
                sat=sat2);
            CompleteBubbleDewStates completeBubbleDewStates1(redeclare package
                Medium =
                  Medium, sat=sat1);
            CompleteBubbleDewStates completeBubbleDewStates2(redeclare package
                Medium =
                  Medium, sat=sat2);
          equation
            state1 = Medium.setState_ph(p1, h1);
            state2 = Medium.setState_pT(p2, T2);
            sat1 = Medium.setSat_p(p1);
            sat2 = Medium.setSat_T(T2);
            Ts = Medium.saturationTemperature(p1);
            ps = Medium.saturationPressure(T2);
          end TestStatesSat;

          partial model TestBasePropertiesExplicit
            "Test case using BaseProperties and explicit equations"
            replaceable package Medium =
                Media_Utilities.Test.GenericModels.DummyTwoPhaseMedium
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);
            CompleteBaseProperties medium1(redeclare package Medium = Medium)
              "Constant pressure, varying enthalpy";
            CompleteBaseProperties medium2(redeclare package Medium = Medium)
              "Varying pressure, constant enthalpy";
            Medium.AbsolutePressure p1;
            Medium.AbsolutePressure p2;
            Medium.SpecificEnthalpy h1;
            Medium.SpecificEnthalpy h2;
          equation
            medium1.baseProperties.p = p1;
            medium1.baseProperties.h = h1;
            medium2.baseProperties.p = p2;
            medium2.baseProperties.h = h2;
          end TestBasePropertiesExplicit;

          partial model TestBasePropertiesImplicit
            "Test case using BaseProperties and implicit equations"
            replaceable package Medium =
                Media_Utilities.Test.GenericModels.DummyTwoPhaseMedium
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);
            parameter Medium.SpecificEnthalpy hstart
              "Start value for specific enthalpy";
            CompleteBaseProperties medium1(redeclare package Medium = Medium,
                baseProperties(h(start=hstart)))
              "Constant pressure, varying enthalpy";
            CompleteBaseProperties medium2(redeclare package Medium = Medium,
                baseProperties(h(start=hstart)))
              "Varying pressure, constant enthalpy";
            Medium.AbsolutePressure p1;
            Medium.AbsolutePressure p2;
            Medium.Temperature T1;
            Medium.Temperature T2;
          equation
            medium1.baseProperties.p = p1;
            medium1.baseProperties.T = T1;
            medium2.baseProperties.p = p2;
            medium2.baseProperties.T = T2;
          end TestBasePropertiesImplicit;

          partial model TestBasePropertiesDynamic
            "Test case using BaseProperties and dynamic equations"
            replaceable package Medium =
                Media_Utilities.Test.GenericModels.DummyTwoPhaseMedium
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);
            parameter SI.Volume V=1 "Storage Volume";
            parameter Real p_atm=101325 "Atmospheric pressure";
            parameter SI.Temperature Tstart=300;
            parameter SI.SpecificEnthalpy hstart=1e5;
            parameter SI.Pressure pstart=p_atm;
            parameter Real Kv0 "Valve flow coefficient";
            Medium.BaseProperties medium(
              preferredMediumStates=true,
              h(start=hstart),
              p(start=pstart));
            SI.Mass M;
            SI.Energy U;
            SI.MassFlowRate win(start=100);
            SI.MassFlowRate wout;
            SI.SpecificEnthalpy hin;
            SI.SpecificEnthalpy hout;
            SI.Power Q;
            Real Kv;
          equation
            // Mass & energy balance equation
            M = medium.d*V;
            U = medium.u*M;
            der(M) = win - wout;
            der(U) = win*hin - wout*hout + Q;
            // Outlet valve equation
            wout = Kv*sqrt(medium.d*(medium.p - p_atm));
            hout = medium.h;
          initial equation
            // Steady state equations
            der(medium.p) = 0;
            der(medium.h) = 0;
            annotation (experiment(StopTime=80, Tolerance=1e-007));
          end TestBasePropertiesDynamic;

          partial model CompareModelicaTestMedium
            "Comparison between Modelica and TestMedium models"
            replaceable package ModelicaMedium =
                Modelica.Media.Water.WaterIF97_ph
              constrainedby Modelica.Media.Interfaces.PartialMedium
              annotation(choicesAllMatching=true);
            replaceable package TestMedium =
                Media_Utilities.Test.GenericModels.DummyTwoPhaseMedium
              constrainedby Modelica.Media.Interfaces.PartialMedium
              annotation(choicesAllMatching=true);
            CompleteBaseProperties modelicaMedium(redeclare package Medium =
                  ModelicaMedium) "Modelica medium model";
            CompleteBaseProperties testMedium(redeclare package Medium = TestMedium)
              "TestMedium medium model";
            parameter Modelica.SIunits.Pressure pmin;
            parameter Modelica.SIunits.Pressure pmax;
            parameter Modelica.SIunits.SpecificEnthalpy hmin;
            parameter Modelica.SIunits.SpecificEnthalpy hmax;
          equation
            modelicaMedium.baseProperties.p = pmin + (pmax - pmin)*time;
            modelicaMedium.baseProperties.h = hmin + (hmax - hmin)*time;
            testMedium.baseProperties.p = pmin + (pmax - pmin)*time;
            testMedium.baseProperties.h = hmin + (hmax - hmin)*time;
          end CompareModelicaTestMedium;

          partial model TestRunner
            "A collection of models to test the states and base properties"
            extends Modelica.Icons.Example;

            replaceable package Medium = Modelica.Media.Water.StandardWater
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);

            Medium.AbsolutePressure p_in;
            Medium.SpecificEnthalpy h_in;
            Medium.Temperature T_in;
            Medium.SaturationProperties sat_in;
            parameter Medium.SpecificEnthalpy hstart = 2e5
              "Start value for specific enthalpy";

            model TestStates_Impl
              extends Media_Utilities.Test.GenericModels.TestStates;
              input Medium.AbsolutePressure _p1;
              input Medium.SpecificEnthalpy _h1;
              input Medium.AbsolutePressure _p2;
              input Medium.Temperature _T2;
            equation
              p1 = _p1;
              h1 = _h1;
              p2 = _p2;
              T2 = _T2;
            end TestStates_Impl;

            TestStates_Impl testStates(
              redeclare package Medium = Medium,
              _p1=p_in,
              _h1=h_in,
              _p2=p_in*1.15,
              _T2=T_in) annotation (Placement(transformation(extent={{-80,60},{-60,80}})));

            model TestStatesSat_Impl
              extends Media_Utilities.Test.GenericModels.TestStatesSat;
              input Medium.AbsolutePressure _p1;
              input Medium.SpecificEnthalpy _h1;
              input Medium.AbsolutePressure _p2;
              input Medium.Temperature _T2;
            equation
              p1 = _p1;
              h1 = _h1;
              p2 = _p2;
              T2 = _T2;
            end TestStatesSat_Impl;

            TestStatesSat_Impl testStatesSat(
              redeclare package Medium = Medium,
              _p1=p_in,
              _h1=h_in,
              _p2=p_in*1.15,
              _T2=T_in) annotation (Placement(transformation(extent={{-40,60},{-20,80}})));

            model TestBasePropertiesExplicit_Impl
              extends
                Media_Utilities.Test.GenericModels.TestBasePropertiesExplicit;
              input Medium.AbsolutePressure _p1;
              input Medium.SpecificEnthalpy _h1;
              input Medium.AbsolutePressure _p2;
              input Medium.SpecificEnthalpy _h2;
            equation
              p1 = _p1;
              h1 = _h1;
              p2 = _p2;
              h2 = _h2;
            end TestBasePropertiesExplicit_Impl;

            TestBasePropertiesExplicit_Impl testBasePropertiesExplicit(
              redeclare package Medium = Medium,
              _p1=p_in,
              _h1=h_in,
              _p2=p_in*1.15,
              _h2=h_in) annotation (Placement(transformation(extent={{-40,20},{-20,40}})));

            model TestBasePropertiesImplicit_Impl
              extends
                Media_Utilities.Test.GenericModels.TestBasePropertiesImplicit;
              input Medium.AbsolutePressure _p1;
              input Medium.Temperature _T1;
              input Medium.AbsolutePressure _p2;
              input Medium.Temperature _T2;
            equation
              p1 = _p1;
              T1 = _T1;
              p2 = _p2;
              T2 = _T2;
            end TestBasePropertiesImplicit_Impl;

            TestBasePropertiesImplicit_Impl testBasePropertiesImplicit(
              redeclare package Medium = Medium,
              _p1=p_in,
              _T1=T_in,
              _p2=p_in*1.15,
              _T2=T_in,
              hstart=hstart)
                        annotation (Placement(transformation(extent={{-80,20},{-60,40}})));

            model TestBasePropertiesDynamic_Impl
              extends
                Media_Utilities.Test.GenericModels.TestBasePropertiesDynamic;
              input Medium.SpecificEnthalpy _h1;
            equation
              // Inlet equations
              win = 1;
              hin = _h1;
              // Input variables
              Kv = if time < 50 then Kv0 else Kv0*1.1;
              Q = if time < 1 then 0 else 1e4;
            end TestBasePropertiesDynamic_Impl;

            TestBasePropertiesDynamic_Impl testBasePropertiesDynamic(Tstart=300,
              hstart=4e5,
              pstart=1e6,
              Kv0=1.00801e-4,
              V=0.1,
              redeclare package Medium = Medium,_h1=h_in)
              annotation (Placement(transformation(extent={{0,20},{20,40}})));
            annotation (experiment(StopTime=80, Tolerance=1e-007));
          end TestRunner;

          model TestRunnerTwoPhase
            "A collection of models to test the saturation states"
            extends Modelica.Icons.Example;
            extends Media_Utilities.Test.GenericModels.TestRunner(redeclare
                package Medium = TwoPhaseMedium);

            replaceable package TwoPhaseMedium =
                Modelica.Media.Water.StandardWater
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);

            parameter Medium.AbsolutePressure p_start = 1e5;
          algorithm
            assert(Medium.fluidConstants[1].criticalPressure>p_start, "You have to start below the critical pressure.");
          equation
            p_in = p_start+0.5*(Medium.fluidConstants[1].criticalPressure-p_start)*time;
            sat_in = Medium.setSat_p(p=p_in);
            h_in = Medium.bubbleEnthalpy(sat_in);
            T_in = Medium.saturationTemperature_sat(sat_in);
          end TestRunnerTwoPhase;

          model TestRunnerTranscritical
            "A collection of models to test the transcritical states"
            extends Modelica.Icons.Example;
            extends Media_Utilities.Test.GenericModels.TestRunner(redeclare
                package Medium = TwoPhaseMedium);

            replaceable package TwoPhaseMedium =
                Modelica.Media.Water.StandardWater
              constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
              annotation(choicesAllMatching=true);

            parameter Medium.AbsolutePressure p_start = 1e5;
          algorithm
            assert(Medium.fluidConstants[1].criticalPressure>p_start, "You have to start below the critical pressure.");
          equation
            p_in = p_start+1.5*(Medium.fluidConstants[1].criticalPressure-p_start)*time;
            sat_in = Medium.setSat_p(p=p_in);
            h_in = Medium.bubbleEnthalpy(sat_in);
            T_in = Medium.saturationTemperature_sat(sat_in);
          end TestRunnerTranscritical;
        end GenericModels;

        package MSL_Models
          "Test cases taken from the Modelica Standard Library, medium redefinition needed."
          extends Modelica.Icons.BasesPackage;

          model BranchingDynamicPipes "From Fluid library, needs medium definition"
            extends Modelica.Fluid.Examples.BranchingDynamicPipes(
              redeclare package Medium=NewMedium);

            replaceable package NewMedium=Modelica.Media.Water.StandardWater constrainedby
              Modelica.Media.Interfaces.PartialMedium
              annotation(choicesAllMatching=true);

            //replaceable package NewMedium=ExternalMedia.Examples.WaterCoolProp;
            //replaceable package NewMedium=Modelica.Media.Water.StandardWater;
            //replaceable package NewMedium=ExternalMedia.Examples.WaterIF97;
          end BranchingDynamicPipes;

          model IncompressibleFluidNetwork
            "From Fluid library, needs medium definition"
            extends Modelica.Fluid.Examples.IncompressibleFluidNetwork(
              redeclare package Medium=NewMedium);
            replaceable package NewMedium=Modelica.Media.Water.StandardWater constrainedby
              Modelica.Media.Interfaces.PartialMedium
              annotation(choicesAllMatching=true);
          end IncompressibleFluidNetwork;
        end MSL_Models;

        model WaterComparison "Compares different implementations of water"
          extends Modelica.Icons.Example;

          GenericModels.TestRunnerTwoPhase      testRunnerTwoPhaseWater1(
            hstart=4e5,
            redeclare package TwoPhaseMedium =
                Media_Utilities.Examples.WaterCoolProp,
            p_start=100000)
            annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
        end WaterComparison;
      end Test;
      annotation (
      Documentation(info="<html>
<p>The <b>ExternalMedia</b> library provides a framework for interfacing external codes computing fluid properties to Modelica.Media-compatible component models. The library has been designed with two main goals: maximizing the efficiency of the code, while minimizing the amount of extra code required to interface existing external codes to the library.</p>
<p>The library covers pure fluids models, possibly two-phase, compliant with the <a href=\"modelica://Modelica.Media.Interfaces.PartialTwoPhaseMedium\">Modelica.Media.Interfaces.PartialTwoPhaseMedium</a> interface. </p>
<p>Two external softwares for fluid property computation are currently suppored by the ExternalMedia library:</p>
<ul>
<li><a href=\"http://www.fluidprop.com\">FluidProp</a>, formerly developed at TU Delft and currently devloped and maintained by Asimptote</li>
<li><a href=\"http://coolprop.org\">CoolProp</a>, developed at the University of Liege and at the Technical University of Denmark (DTU)</li>
</ul>
<p>The library has been tested with the Dymola and OpenModelica tools under the Windows operating system. If you are interested in the support of other tools, operating systems, and external fluid property computation codes, please contact the developers.</p>
<p>Main contributors: Francesco Casella, Christoph Richter, Roberto Bonifetto, Ian Bell.</p>
<p><b>The code is licensed under the Modelica License 2. </b>For license conditions (including the disclaimer of warranty) visit <a href=\"https://www.modelica.org/licenses/ModelicaLicense2\">https://www.modelica.org/licenses/ModelicaLicense2</a>. </p>
<p>Copyright &copy; 2006-2014, Politecnico di Milano, TU Braunschweig, Politecnico di Torino, Universit&eacute; de Liege.</p>
</html>"));
    end Media_Utilities;

    record Default "Default values from estimation"
      extends DymolaModels.Icons.Data.Record;
      extends
        ElectrifiedPowertrains.ElectricMachines.AIM.Thermal.Records.Base.ForcedCoolingThreeMassEstimation2(
        coolingMedium=CHEETA_Thermal.Utilities.Media_Utilities.Media.CoolPropMedium,
        airGapMedium=Modelica.Thermal.FluidHeatFlow.Media.Air_70degC(),
        machineMass=10,
        housingLength(displayUnit="mm") = 0.25,
        housingRadius(displayUnit="mm") = 0.10,
        initialTemperature=298.15);
    end Default;

    package LH2CoolProp "CoolProp model of LH2"
      extends Media_Utilities.Media.CoolPropMedium(
        mediumName="LH2",
        substanceNames={"Liquid Hydrogen"},
        ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.ph,
        AbsolutePressure(start=10e5),
        SpecificEnthalpy(start=2e5));
    end LH2CoolProp;
  end Utilities;

  package Inverters
    model ThermalImpedanceWithCooling "thermal impedance with cooling"
      import ElectrifiedPowertrains;
      import Cooling;
      extends ThermalManagementDemos.EPTLExample.InverterCoolingInterface;
      extends ElectrifiedPowertrains.Common.Icons.Electrical.ThermalModel;

      parameter Modelica.SIunits.Mass m=0.1 "Mass of medium";

      parameter Modelica.SIunits.Conversions.NonSIunits.Temperature_degC T = 75 "Initial temperature of medium";
      parameter Modelica.SIunits.PressureDifference dp_nominal=11900;
      parameter Modelica.SIunits.MassFlowRate m_flowNominal=0.00017971;
      parameter Modelica.SIunits.Density d_nominal=1078.26;
      parameter DassaultSystemes.Fluid.Types.InitType initType = DassaultSystemes.Fluid.Types.InitType.FixedInitial;
      parameter Boolean initialize_T = true;
      parameter Modelica.SIunits.MassFlowRate m_flowInit=0.1;
      parameter Modelica.SIunits.AbsolutePressure p_aInit=Medium.p_default;
      parameter Modelica.SIunits.AbsolutePressure p_bInit=Medium.p_default;

      final parameter Modelica.SIunits.Temperature T0= Modelica.SIunits.Conversions.from_degC(T);

      replaceable parameter
        ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Records.Data.ThermalImpedanceWithCooling.Default
                                                                                                                   data
        constrainedby
        ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Records.Base.ThermalImpedanceWithCooling
        annotation (choicesAllMatching, Placement(transformation(extent={{-70,70},{-50,90}})));
      ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.ThermalImpedance coreElement_IGBT(
        n=data.n,
        readFromFile=data.readFromFile,
        filename=data.filename,
        R_thJF=data.R_thJF_IGBT,
        V_flow_R_thJF=data.V_flow_R_thJF_IGBT,
        R_thJF_table(dataset="R_thJF_IGBT"))   annotation (choicesAllMatching, Placement(transformation(extent={{-40,-20},{-20,0}})));
      ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.ThermalImpedance coreElement_diode(
        n=data.n,
        readFromFile=data.readFromFile,
        filename=data.filename,
        R_thJF=data.R_thJF_diode,
        V_flow_R_thJF=data.V_flow_R_thJF_diode,
        R_thJF_table(dataset="R_thJF_diode"))   annotation (Placement(transformation(extent={{40,-20},{20,0}})));

      LiquidCooling coreElement_liquidCooling(
        redeclare package Medium = Medium,
        T=T,
        readFromFile=data.readFromFile,
        filename=data.filename,
        initType=initType,
        initialize_T=initialize_T,
        m_flowInit=m_flowInit,
        dp_nominal(displayUnit="Pa") = dp_nominal,
        m_flowNominal=m_flowNominal,
        d_nominal=d_nominal,
        p_aInit=p_aInit,
        p_bInit=p_bInit)
        annotation (Placement(transformation(extent={{-10,60},{10,40}})));

      Cooling.Sensors.LiquidSensors.VolumeFlowSensor volumeFlowSensor(redeclare
          package Medium =                                                                       Medium)
        annotation (Placement(transformation(extent={{-70,60},{-50,40}})));
    equation
      connect(coreElement_diode.heatPort_heatSink, coreElement_liquidCooling.heatPort)
        annotation (Line(
          points={{20,-10},{0,-10},{0,40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(coreElement_IGBT.heatPort_heatSink, coreElement_liquidCooling.heatPort)
        annotation (Line(
          points={{-20,-10},{0,-10},{0,40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(temperatureSensor_heatSink.port, coreElement_liquidCooling.heatPort)
        annotation (Line(points={{70,60},{60,60},{60,30},{0,30},{0,40}}, color={191,0,0}));
      connect(coreElement_IGBT.heatPort_junction, thermalPortInverter.heatPort_switch)
        annotation (Line(points={{-40,-10},{-60,-10},{-80,-10},{-80,-98},{-1,-98}},
            color={191,0,0}));
      connect(coreElement_diode.heatPort_junction, thermalPortInverter.heatPort_diode)
        annotation (Line(points={{40,-10},{60,-10},{60,-98},{-1,-98}}, color={191,0,
              0}));
      connect(flowPort_in, volumeFlowSensor.port_a)
        annotation (Line(points={{-80,100},{-80,100},{-80,64},{-80,50},{-70,50}}, color={255,127,36}));
      connect(coreElement_liquidCooling.flowPort_in, volumeFlowSensor.port_b)
        annotation (Line(points={{-10,50},{-30,50},{-50,50}}, color={255,127,36}));
      connect(coreElement_liquidCooling.flowPort_out, flowPort_out)
        annotation (Line(points={{10,50},{40,50},{40,100},{80,100}}, color={255,127,36}));
      connect(volumeFlowSensor.y, coreElement_IGBT.volumeFlow) annotation (Line(points={{-49,44},{-30,44},{-30,2}}, color={0,0,127}));
      connect(volumeFlowSensor.y, coreElement_diode.volumeFlow)
        annotation (Line(points={{-49,44},{-30,44},{-30,10},{30,10},{30,2},{30,2}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={Text(
              extent={{-56,-24},{56,-50}},
              lineColor={28,108,200},
              horizontalAlignment=TextAlignment.Left,
              textString="Auf der Fluid-Seite muss keine HT-Korrelation
berechnet werden, da hier ein Wärmestrom 
durch coreElement_IGBT und coreElement_diode 
berechnet wird."),     Text(
              extent={{-30,108},{32,48}},
              lineColor={28,108,200},
              textString="Das abweichende Druckverlustmodell
basiert auf einem Nennbetriebspunkt
und legt eine quadratische dp(m_flow)
Kurve durch. Der Nennpunkt kommt
aus dem Datenblatt, die Werte dafür
sind jedoch nicht im data-Record
enthalten. Daher hier direkt eingetragen",
              horizontalAlignment=TextAlignment.Left)}),
                                    Documentation(info="<html>
<p>This model includes the thermal conductance from junction to case of the inverter's switch and diode.</p>
<ul>
<li>In inverter datasheets the thermal conductance is usually provided for a single element (IGBT/diode)</li>
<li>Thus, the thermal conductance is multiplied by the number of elements</li>
</ul>
<p>The thermal circuit is liquid cooled. This model is based on <a href=\"ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.LiquidCooling\">LiquidCooling</a></p>
</html>"));
    end ThermalImpedanceWithCooling;

    model ThermalConductanceWithBaseplate "thermal conductance with baseplate"
      extends
        ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Interfaces.Base;
      extends ElectrifiedPowertrains.Common.Icons.Electrical.ThermalModel;
      //parameter Integer n_switches=6 "Number of switches, i.e. pairs of IGBTs and diodes";
      //parameter Integer n_modules=6 "Number of modules mounted to the heatsink";

      replaceable parameter
        ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Records.Data.ThermalConductanceWithBaseplate.Default
        data constrainedby
        ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Records.Base.ThermalConductanceWithBaseplate
        annotation (choicesAllMatching, Placement(transformation(extent={{-70,
                50},{-50,70}})));
      ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.ThermalConductanceWithBaseplate
        coreElement(
        G_thJC_IGBT=data.n_switches*data.G_thJC_IGBT,
        G_thJC_diode=data.n_switches*data.G_thJC_diode,
        G_thCS_module=data.n_modules*data.G_thCS_module)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heatPort_heatSink
        annotation (Placement(transformation(extent={{-10,90},{10,110}})));
    equation
      connect(temperatureSensor_heatSink.port, heatPort_heatSink) annotation (Line(points={{70,60},{0,60},{0,100}}, color={191,0,0}));
      connect(heatPort_heatSink, coreElement.heatPort_heatSink) annotation (Line(points={{0,100},{0,10}}, color={191,0,0}));
      connect(coreElement.heatPort_junctionIGBT, thermalPortInverter.heatPort_switch)
        annotation (Line(points={{-10,0},{-46,0},{-80,0},{-80,-98},{-1,-98}}, color=
             {191,0,0}));
      connect(coreElement.heatPort_junctionDiode, thermalPortInverter.heatPort_diode)
        annotation (Line(points={{10,0},{36,0},{60,0},{60,-98},{-1,-98}}, color={
              191,0,0}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}})), Documentation(info="<html>
<p>This model is based on <a href=\"ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.ThermalConductanceWithBaseplate\">ThermalConductanceWithBaseplate</a>.</p>
<ul>
<li>The model includes the thermal conductance from junction to case (baseplate) of the module's switch and diode.</li>
<li>The thermal conductance from case to sink (ambient) of the baseplate is additionally included.</li>
</ul>
<p><h4>Important Information</h4></p>
<ul>
<li>This model is based on a module <a href=\"ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.ThermalConductanceWithBaseplate\">ThermalConductanceWithBaseplate</a>
that includes only one IGBT (or any type of switch) and one diode connected to a baseplate</li>
<li>Thus, in case of a three-phase inverter, the thermal conductance is multiplied by 6 (number of elements)</li>
</ul>
</html>"));
    end ThermalConductanceWithBaseplate;

    partial model InverterCoolingInterface
      "ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Interfaces.Cooling;"
      import Cooling;
      extends
        ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Interfaces.Base;

      /*
  parameter Modelica.Thermal.FluidHeatFlow.Media.Medium medium=Modelica.Thermal.FluidHeatFlow.Media.Medium()
    "Medium in the component"
    annotation(choicesAllMatching=true);
    */
      replaceable package Medium = Cooling.Media.Liquids.EthyleneGlycol52 constrainedby
        Cooling.Media.Templates.BaseLiquid
      annotation(choicesAllMatching=true);

      DassaultSystemes.Fluid.Interfaces.LiquidPort_a
                                             flowPort_in(
        redeclare package Medium=Medium)
        annotation (Placement(transformation(extent={{-90,90},{-70,110}})));
      DassaultSystemes.Fluid.Interfaces.LiquidPort_b
                                             flowPort_out(
        redeclare package Medium=Medium)
        annotation (Placement(transformation(extent={{70,90},{90,110}})));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        Documentation(info="<html>
<p>The thermal cooling interface extends the base interface and adds two flow ports (inlet and outlet)</p>
</html>"));
    end InverterCoolingInterface;

    model FiveElementsWithCooling
      "thermal model with cooling based on foster equivalent circuits"
    import ElectrifiedPowertrains;

        extends ThermalManagementDemos.EPTLExample.InverterCoolingInterface;
      extends ElectrifiedPowertrains.Common.Icons.Electrical.ThermalModel;

      parameter Modelica.SIunits.Mass m=0.1 "Mass of medium";

      parameter Modelica.SIunits.Conversions.NonSIunits.Temperature_degC T = 75 "Initial temperature of medium";

      final parameter Modelica.SIunits.Temperature T0= Modelica.SIunits.Conversions.from_degC(T);

      replaceable parameter ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Records.Data.FiveElementsWithCooling.Default
        data constrainedby
        ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Records.Base.FiveElementsWithCooling
             annotation (choicesAllMatching, Placement(transformation(extent={{-70,70},{-50,90}})));
      ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.FosterCircuitFiveElements
        coreElement_IGBT(
        tau1=data.IGBT.tau1,
        tau2=data.IGBT.tau2,
        tau3=data.IGBT.tau3,
        tau4=data.IGBT.tau4,
        tau5=data.IGBT.tau5,
        R1=data.IGBT.R1/data.n,
        R2=data.IGBT.R2/data.n,
        R3=data.IGBT.R3/data.n,
        R4=data.IGBT.R4/data.n,
        R5=data.IGBT.R5/data.n) annotation (choicesAllMatching, Placement(
            transformation(extent={{-40,-20},{-20,0}})));
      ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.FosterCircuitFiveElements
        coreElement_diode(
        tau1=data.diode.tau1,
        tau2=data.diode.tau2,
        tau3=data.diode.tau3,
        tau4=data.diode.tau4,
        tau5=data.diode.tau5,
        R1=data.diode.R1/data.n,
        R2=data.diode.R2/data.n,
        R3=data.diode.R3/data.n,
        R4=data.diode.R4/data.n,
        R5=data.diode.R5/data.n)
        annotation (Placement(transformation(extent={{40,-20},{20,0}})));

      LiquidCooling coreElement_liquidCooling(
        dp_nominal=20000,
        m_flowNominal=1,
        d_nominal=200,
        T=T,
        readFromFile=data.readFromFile,
        filename=data.filename)
        annotation (Placement(transformation(extent={{-10,60},{10,40}})));
    equation
      connect(coreElement_IGBT.heatPort_heatSink, coreElement_diode.heatPort_heatSink)
        annotation (Line(
          points={{-20,-10},{20,-10}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(coreElement_liquidCooling.heatPort, coreElement_diode.heatPort_heatSink)
        annotation (Line(
          points={{0,40},{0,-10},{20,-10}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(temperatureSensor_heatSink.port, coreElement_liquidCooling.heatPort)
        annotation (Line(points={{70,60},{60,60},{60,20},{0,20},{0,40}}, color={191,0,0}));
      connect(coreElement_liquidCooling.flowPort_out, flowPort_out)
        annotation (Line(points={{10,50},{40,50},{40,100},{80,100}}, color={255,0,0}));
      connect(coreElement_liquidCooling.flowPort_in, flowPort_in)
        annotation (Line(points={{-10,50},{-10,50},{-80,50},{-80,100}}, color={255,0,0}));
      connect(coreElement_IGBT.heatPort_junction, thermalPortInverter.heatPort_switch)
        annotation (Line(points={{-40,-10},{-80,-10},{-80,-98},{-1,-98}},
            color={191,0,0}));
      connect(coreElement_diode.heatPort_junction, thermalPortInverter.heatPort_diode)
        annotation (Line(points={{40,-10},{50,-10},{60,-10},{60,-98},{-1,-98}},
            color={191,0,0}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}})), Documentation(info="<html>
<p>The model includes a foster circuit <a href=\"ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.FosterCircuitFourElements\">FosterCircuitFourElements</a> 
for the diode and one for the IGBT of an inverter.</p>
<ul>
<li>In semiconductor datasheets the values of the <i>resistors and time constants</i> or the <i>resistors and capacitors</i> of the foster circuit are provided for a single element (IGBT/diode)</li>
<li>Thus, the thermal resistances are divided by the number of elements</li>
</ul>
<p>The thermal circuit is liquid cooled. This model is based on <a href=\"ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.LiquidCooling\">LiquidCooling</a></p>
</html>"));
    end FiveElementsWithCooling;

    model LiquidCooling
      import SI = Modelica.SIunits;
      import Cooling;

      // Parameters
      extends DymolaModels.Utilities.Parameters.ReadFromFile;

      // Icon
      extends ElectrifiedPowertrains.Common.Icons.Electrical.CoolingModel;
    /*
  parameter Modelica.Thermal.FluidHeatFlow.Media.Medium medium=Modelica.Thermal.FluidHeatFlow.Media.Medium()
    "Medium in the component" annotation (choicesAllMatching=true);
    */
      replaceable package Medium =
            CHEETA_Thermal.Utilities.Media_Utilities.Media.CoolPropMedium
      annotation(choicesAllMatching=true);

      Modelica.SIunits.Mass m "Mass of medium";
      /*
  parameter SI.PressureDifference pressureDrop[size(V_flow_pressureDrop,1)]={0.0,10.25,25.0,45.2,70.0,100.0,134.65,175.0,219.2,269.0,323.5}
    "Pressure drop = f(volume flow rate), depending on 'V_flow_pressureDrop'" annotation (Dialog(group="Pressure drop"));
  parameter SI.VolumeFlowRate V_flow_pressureDrop[:] = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0}
  "Volume flow scale for pressure drop" annotation (Dialog(group="Pressure drop"));
  */
      parameter SI.PressureDifference dp_nominal;
      parameter SI.MassFlowRate m_flowNominal;
      parameter SI.Density d_nominal;
      parameter DassaultSystemes.Fluid.Types.InitType initType = DassaultSystemes.Fluid.Types.InitType.FixedInitial;
      parameter Boolean initialize_T = true;
      parameter SI.MassFlowRate m_flowInit = 0.1;
      parameter SI.AbsolutePressure p_aInit = Medium.p_default;
      parameter SI.AbsolutePressure p_bInit = Medium.p_default;

      parameter Modelica.SIunits.Conversions.NonSIunits.Temperature_degC T=75 "Initial temperature of medium";

      final parameter Modelica.SIunits.Temperature T0=Modelica.SIunits.Conversions.from_degC(T);

      DassaultSystemes.Fluid.Interfaces.LiquidPort_a
                                             flowPort_in(
        redeclare package Medium=Medium)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      DassaultSystemes.Fluid.Interfaces.LiquidPort_b
                                             flowPort_out(
        redeclare package Medium=Medium)
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort annotation (Placement(transformation(extent={{-10,90},{10,110}})));
      Modelica.Blocks.Interfaces.RealOutput relativePressure
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-110})));

      //ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.Cooling.HeatedPipeTableBased pipe(
      /*
  Cooling_Development.EPTLExample.HeatedPipeTableBased pipe(
    T0=T0,
    medium=medium,
    m=m,
    T0fixed=false,
    pressureDrop=pressureDrop,
    V_flow_pressureDrop=V_flow_pressureDrop,
    readFromFile=readFromFile,
    filename=filename,
    tapT=0.5) annotation (Placement(transformation(extent={{-10,10},{10,-10}}, rotation=0)));
    */
      DassaultSystemes.Fluid.Pipes.DiscretizedLiquidPipe
                                                    pipe(
        redeclare package Medium = Medium,
        allowFlowReversal=false,
        final useHeatTransfer=true,
        final N=1,
        dp_nominal=2000,
        m_flowNominal=1,
        L(displayUnit="mm") = 0.194,
        final initType=initType,
        initialize_T=initialize_T,
        final m_flowInit=m_flowInit,
        final useDynamicMomentum=false,
        A=6.70669e-5/pipe.L,
        perimeter(displayUnit="mm") = pipe.A_ht/pipe.L,
        A_ht=662*8.1e-3*Modelica.Constants.pi*2,
        final p_aInit=p_aInit,
        final p_bInit=p_bInit,
        final T_aInit=T0,
        final T_bInit=T0,
        redeclare model HT = Cooling.Pipes.HeatTransfer.ConstAlpha ( alpha_set=20000),
        redeclare model DP = Cooling.Pipes.Friction.QuadraticNominalOpPoint) "Coolant control volume of cooling tray"
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      Cooling.Sensors.LiquidSensors.PressureSensor upstreamPressureSensor(
        redeclare package Medium = Medium)
        annotation (Placement(transformation(extent={{-30,-32},{-10,-52}})));
      Cooling.Sensors.LiquidSensors.PressureSensor downStreamPressureSensor(
        redeclare package Medium = Medium)
        annotation (Placement(transformation(extent={{10,-20},{30,-40}})));
      Modelica.Blocks.Math.Add relPressure(
        k1=-1,
        k2=+1)
        annotation (Placement(transformation(extent={{52,-46},{72,-26}})));
    equation
      m = sum(pipe.ms);
      /*
  connect(pipe.flowPort_b, flowPort_out)
    annotation (Line(
      points={{10,0},{100,0}},
      color={255,0,0},
      smooth=Smooth.None));
  connect(relPressureSensor.flowPort_a, pipe.flowPort_a)
    annotation (Line(
      points={{-10,-32},{-20,-32},{-20,0},{-10,0}},
      color={255,0,0},
      smooth=Smooth.None));
  connect(relPressureSensor.flowPort_b, flowPort_out)
    annotation (Line(
      points={{10,-32},{20,-32},{20,0},{100,0}},
      color={255,0,0},
      smooth=Smooth.None));
  connect(pipe.heatPort, heatPort) annotation (Line(
      points={{0,10},{0,100}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(relPressureSensor.y, relativePressure)
    annotation (Line(
      points={{0,-43},{0,-110}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pipe.flowPort_a, flowPort_in)
    annotation (Line(
      points={{-10,0},{-100,0}},
      color={255,0,0},
      smooth=Smooth.None));
  */
      connect(flowPort_in, pipe.port_a) annotation (Line(points={{-100,0},{-50,0},{-10,0}}, color={255,127,36}));
      connect(pipe.port_b, flowPort_out) annotation (Line(points={{10,0},{60,0},{100,0}}, color={255,127,36}));
      connect(heatPort, pipe.heatPort[1]) annotation (Line(points={{0,100},{0,3.6},{0,3.6}},color={191,0,0}));
      connect(pipe.port_a, upstreamPressureSensor.port) annotation (Line(points={{-10,0},{-20,0},{-20,-32}}, color={255,127,36}));
      connect(pipe.port_b, downStreamPressureSensor.port) annotation (Line(points={{10,0},{20,0},{20,-20}}, color={255,127,36}));
      connect(downStreamPressureSensor.y, relPressure.u1) annotation (Line(points={{31,-30},{31,-30},{50,-30}}, color={0,0,127}));
      connect(upstreamPressureSensor.y, relPressure.u2)
        annotation (Line(points={{-9,-42},{-9,-42},{40,-42},{40,-42},{50,-42}}, color={0,0,127}));
      connect(relPressure.y, relativePressure)
        annotation (Line(points={{73,-36},{78,-36},{80,-36},{80,-80},{0,-80},{0,-110}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})), Documentation(info="<html>
<p>This model is based on the <a href=\"ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.CoreElements.Cooling.HeatedPipeTableBased\">HeatedPipeTableBased</a> model</p>
<p>The pressure drop accross the pipe is described by means of a characteristic curve <i>dp = f(dV/dt)</i> typically provided in the datasheet of forced cooled inverters</p>
</html>"));
    end LiquidCooling;
  end Inverters;

  package Motors
    model WaterCoolingThreeMasses
      "Water cooled PSM with three thermal masses: stator iron, stator copper and rotor (including shaft)"
      import ElectrifiedPowertrains;
      import Cooling;
      //extends ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.PartialModels.FluidCoolingThreeMasses;
      extends ThermalManagementDemos.EPTLExample.FluidCoolingThreeMasses;

      // Icon
      extends ElectrifiedPowertrains.Common.Icons.ThermalMachine.WaterCooled;

      Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(m=2)
        annotation (Placement(transformation(extent={{-26,-52},{-14,-64}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor statorCoreTemperatureSensor
        annotation (Placement(transformation(extent={{40,-46},{52,-34}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor statorWindingTemperatureSensor
        annotation (Placement(transformation(extent={{40,-66},{52,-54}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor rotorTemperatureSensor
        annotation (Placement(transformation(extent={{40,-86},{52,-74}})));
      DymolaModels.Blocks.Interfaces.TemperatureOutput rotorTemperature
        annotation (Placement(transformation(extent={{100,-90},{120,-70}})));
      DymolaModels.Blocks.Interfaces.TemperatureOutput statorWindingTemperature
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
      DymolaModels.Blocks.Interfaces.TemperatureOutput statorCoreTemperature
        annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
      DassaultSystemes.Fluid.Interfaces.LiquidPort_b
                                             flowPort_b(
        redeclare package Medium=CoolantMedium)
        annotation (Placement(transformation(extent={{70,90},{90,110}})));
      DassaultSystemes.Fluid.Interfaces.LiquidPort_a
                                             flowPort_a(
        redeclare package Medium=CoolantMedium)
        annotation (Placement(transformation(extent={{-90,90},{-70,110}})));

    protected
      Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortSMPM internalThermalPort(final m=3, final useDamperCage=false)
        annotation (Placement(transformation(extent={{-10,-90},{10,-70}})));
    public
      Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortSMPM thermalPort(final m=3, final useDamperCage=false)
        annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
    equation
      connect(stator.statorAirGap, airGap.airGapStator) annotation (Line(points={{0,30},{0,10}}, color={191,0,0}));
      connect(airGap.airGapRotor, rotor.rotorAirGap) annotation (Line(points={{0,-10},{0,-10},{0,-30}}, color={191,0,0}));
      connect(internalThermalPort.heatPortStrayLoad, stator.strayLoadLosses)
        annotation (Line(points={{0,-80},{0,-80},{0,-78},{-36,-78},{-36,36},{-10,36}},
                                                                         color={199,0,0}));
      connect(stator.coreLosses, internalThermalPort.heatPortStatorCore)
        annotation (Line(points={{-10,40},{-38,40},{-38,-82},{0,-82},{0,-80}},
                                                                             color={191,0,0}));
      connect(internalThermalPort.heatPortStatorWinding, stator.copperLosses)
        annotation (Line(points={{0,-80},{0,-84},{-40,-84},{-40,44},{-10,44}},             color={199,0,0}));
      connect(rotor.windingLosses, internalThermalPort.heatPortRotorWinding)
        annotation (Line(points={{-10,-36},{-34,-36},{-34,-76},{0,-76},{0,-82},{-1,-82}},
                                                                         color={191,0,0}));
      connect(airGap.flange, flange) annotation (Line(points={{10,0},{10,0},{100,0}}, color={0,0,0}));
      connect(fluidCooling.stator, stator.statorHousing) annotation (Line(points={{0,70},{0,70},{0,50}}, color={191,0,0}));
      connect(fixedTemperature.port, stator.statorInnerAirFront)
        annotation (Line(points={{-20,0},{-14,0},{-14,20},{-6,20},{-6,30}}, color={191,0,0}));
      connect(stator.statorInnerAirRear, fixedTemperature.port)
        annotation (Line(points={{6,30},{6,20},{-14,20},{-14,0},{-20,0}}, color={191,0,0}));
      connect(fixedTemperature.port, rotor.innerAirFront)
        annotation (Line(points={{-20,0},{-18,0},{-14,0},{-14,-20},{-6,-20},{-6,-30}}, color={191,0,0}));
      connect(rotor.innerAirRear, fixedTemperature.port)
        annotation (Line(points={{6,-30},{6,-30},{6,-20},{-14,-20},{-14,0},{-20,0}}, color={191,0,0}));
      connect(thermalCollector.port_b, rotor.coreLosses) annotation (Line(points={{-20,-52},{-20,-44},{-10,-44}}, color={191,0,0}));
      connect(thermalCollector.port_a[1], internalThermalPort.heatPortRotorCore)
        annotation (Line(points={{-20,-64.3},{-20,-74},{0,-74},{0,-80}},
                                                                       color={191,0,0}));
      connect(thermalCollector.port_a[2], internalThermalPort.heatPortPermanentMagnet)
        annotation (Line(points={{-20,-63.7},{-20,-74},{-1,-74},{-1,-80}},
                                                                 color={191,0,0}));
      connect(rotorTemperatureSensor.port, rotor.rotorAirGap)
        annotation (Line(points={{40,-80},{20,-80},{20,-16},{0,-16},{0,-30}}, color={191,0,0}));
      connect(statorCoreTemperatureSensor.port, stator.statorAirGap)
        annotation (Line(points={{40,-40},{24,-40},{24,26},{0,26},{0,30}}, color={191,0,0}));
      connect(statorCoreTemperatureSensor.T, statorCoreTemperature)
        annotation (Line(points={{52,-40},{110,-40},{110,-40}}, color={0,0,127}));
      connect(statorWindingTemperature, statorWindingTemperatureSensor.T)
        annotation (Line(points={{110,-60},{81,-60},{52,-60}}, color={0,0,127}));
      connect(rotorTemperatureSensor.T, rotorTemperature) annotation (Line(points={{52,-80},{110,-80}},           color={0,0,127}));
      connect(noBearing.bearingLosses, internalThermalPort.heatPortFriction)
        annotation (Line(points={{-70,0},{-80,0},{-80,-86},{0,-86},{0,-80}},   color={191,0,0}));
      connect(internalThermalPort, thermalPort) annotation (Line(points={{0,-80},{0,-80},{0,-102},{0,-102},{0,-100},{0,-100}},
                                                                                                color={199,0,0}));
      connect(stator.copperLosses[1], statorWindingTemperatureSensor.port)
        annotation (Line(points={{-10,43.3333},{-14,43.3333},{-14,24},{22,24},{
              22,-60},{40,-60}},                                                                  color={191,0,0}));
      connect(flowPort_a, fluidCooling.flowPort_in)
        annotation (Line(points={{-80,100},{-80,100},{-80,80},{-10,80}}, color={255,127,36}));
      connect(fluidCooling.flowPort_out, flowPort_b) annotation (Line(points={{10,80},{32,80},{80,80},{80,100}}, color={255,127,36}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end WaterCoolingThreeMasses;

    model ThreeMassTEFC2 "AIM: Totally Enclosed Forced Cooling - Three masses"
      import ElectrifiedPowertrains;
      // Interfaces
      extends CHEETA_Thermal.Motors.GenericForcedCooling2;
      extends
        ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Interfaces.OverallMachine(
        final m=3,
        redeclare final Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortAIMC thermalPort,
        redeclare final Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortAIMC internalThermalPort);

      // Icon
      extends
        ElectrifiedPowertrains.Common.Icons.ThermalMachine.ForcedConvection;

      // Parameter
      replaceable parameter ElectrifiedPowertrains.ElectricMachines.AIM.Thermal.Records.Data.ForcedCoolingThreeMassEstimation2.MSL_18p5kW
                                                                                                                      data
        constrainedby
        ElectrifiedPowertrains.ElectricMachines.AIM.Thermal.Records.Base.ForcedCoolingThreeMasses
        annotation (choicesAllMatching, Placement(transformation(extent={{-80,-100},{-60,-80}})));

      ForcedCoolingThreeMasses2 coreElement(
        coolingMedium=data.coolingMedium,
        airGapMedium=data.airGapMedium,
        initialTemperature=data.initialTemperature,
        airGapThickness=data.airGapData.airGapThickness,
        rotorRadius=data.airGapData.rotorRadius,
        specificHeatCapacity=data.rotorData.specificHeatCapacity,
        mass=data.rotorData.mass,
        statorIronMass=data.statorData.statorIronMass,
        windingMass=data.statorData.windingMass,
        crossSectionSlotting=data.statorData.crossSectionSlotting,
        coreLength=data.statorData.coreLength,
        contactAreaIronWinding=data.statorData.contactAreaIronWinding,
        insulationThickness=data.statorData.insulationThickness,
        specificHeatCapacityYoke=data.statorData.specificHeatCapacityYoke,
        specificHeatCapacityWinding=data.statorData.specificHeatCapacityWinding,
        fillFactor=data.statorData.fillFactor,
        mediumMass=data.forcedConvectionData.mediumMass,
        temperatureTap=data.forcedConvectionData.temperatureTap,
        machineSurface=data.forcedConvectionData.machineSurface,
        machineLength=data.forcedConvectionData.machineLength,
        innerDiameter=data.forcedConvectionData.innerDiameter,
        outerDiameter=data.forcedConvectionData.outerDiameter,
        finSpacing=data.forcedConvectionData.finSpacing,
        finDepth=data.forcedConvectionData.finDepth)
        annotation (Placement(transformation(extent={{-20,-20},{20,20}})));

      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      DymolaModels.Blocks.Interfaces.TemperatureOutput
        statorWindingTemperature1
        annotation (Placement(transformation(extent={{90,-62},{110,-42}})));
    equation
      connect(coreElement.flowPort_b, flowPort_b)
        annotation (Line(points={{16,20},{16,20},{16,80},{80,80},{80,100}},               color={255,0,0}));
      connect(coreElement.flowPort_a, flowPort_a)
        annotation (Line(points={{-16,20},{-16,20},{-16,80},{-80,80},{-80,100}},    color={255,0,0}));
      connect(coreElement.thermalPort, internalThermalPort) annotation (Line(points={{0,-20},{0,-70},{0,-80}}, color={199,0,0}));
      connect(coreElement.flange, flange) annotation (Line(points={{20,0},{60,0},{100,0}}, color={0,0,0}));
      connect(coreElement.statorWindingTemperature, statorWindingTemperature1)
        annotation (Line(points={{22,-12},{62,-12},{62,-52},{100,-52}}, color={
              0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        Documentation(info="<html>
<p> See <a href=\"modelica://ElectrifiedPowertrains.ElectricMachines.AIM.Thermal.CoreElements.ForcedCoolingThreeMasses\">ForcedCoolingThreeMasses</a> </p>
</html>"));
    end ThreeMassTEFC2;

    model FluidCooled
      "Computes the thermal resistance between housing and coolant fluid"
      import SI = Modelica.SIunits;
      import Modelica.Constants.pi;

      //extends Modelica.Thermal.FluidHeatFlow.Interfaces.Partials.TwoPort(final tapT = temperatureTap, final m = mediumMass);

      // Icon
     // extends ElectrifiedPowertrains.Common.Icons.SubIcons.Background;
     extends DymolaModels.Icons.Components.Boxed;
      extends ElectrifiedPowertrains.Common.Icons.ThermalMachine.LiquidCooling;

      // Parameters
      extends
        ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.ForcedConvection.Records.Base.FluidCooled;

      replaceable package Medium =
              CHEETA_Thermal.Utilities.LH2CoolProp
      annotation(choicesAllMatching=true);
      parameter SI.PressureDifference dp_nominal;
      parameter SI.MassFlowRate m_flowNominal;
      parameter SI.Density d_nominal;
      parameter DassaultSystemes.Fluid.Types.InitType initType = DassaultSystemes.Fluid.Types.InitType.FixedInitial;
      parameter Boolean initialize_T = true;
      parameter SI.MassFlowRate m_flowInit = 0.1;
      parameter SI.Temperature T0(start=293.15, displayUnit="degC")
        "Initial temperature of medium";
      parameter SI.AbsolutePressure p_aInit = Medium.p_default;
      parameter SI.AbsolutePressure p_bInit = Medium.p_default;

      DassaultSystemes.Fluid.Interfaces.LiquidPort_a
                                             flowPort_in(
        redeclare package Medium=Medium)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      DassaultSystemes.Fluid.Interfaces.LiquidPort_b
                                             flowPort_out(
        redeclare package Medium=Medium)
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));

      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a stator annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
      DassaultSystemes.Fluid.Pipes.DiscretizedLiquidPipe
                                                    channel(
        redeclare package Medium = Medium,
        allowFlowReversal=false,
        useHeatTransfer=true,
        dp_nominal=dp_nominal,
        m_flowNominal=m_flowNominal,
        d_nominal=d_nominal,
        final A=ductCrossSection,
        final perimeter=ductPerimeter,
        final A_ht=coolingSurfaceArea,
        initType=initType,
        initialize_T=initialize_T,
        m_flowInit=m_flowInit,
        p_aInit=p_aInit,
        p_bInit=p_bInit,
        T_aInit=T0,
        T_bInit=T0,
        final useDynamicMomentum=false,
        final L=L,
        redeclare model HT =
            ThermalManagementDemos.EPTLExample.LiquidJacketCircularChannel,
        redeclare model DP = Cooling.Pipes.Friction.LaminarTurbulentPipeFlow (from_dp=false, r=0.0),
        N=1) annotation (Placement(transformation(extent={{-10,10},{10,-10}})));

      final parameter SI.Area ductCrossSection=if ductGeometry == ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Types.CoolingDuctGeometry.Circular
           then pi*ductRadius^2 else if ductGeometry == ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Types.CoolingDuctGeometry.Rectangular
           then ductHeight*ductWidth else ductOuterRadius^2*pi - ductInnerRadius^2*pi "Cross-section of cooling duct";
      final parameter SI.Length ductPerimeter=if ductGeometry == ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Types.CoolingDuctGeometry.Circular
           then 2*pi*ductRadius else if ductGeometry == ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Types.CoolingDuctGeometry.Rectangular
           then 2*(ductHeight + ductWidth) else 2*pi*(ductOuterRadius + ductInnerRadius) "Surface area for cooling";
      final parameter SI.Area coolingSurfaceArea=ductPerimeter*coolingDuctLength "Surface area for cooling";
      final parameter SI.Diameter D=4*ductCrossSection/ductPerimeter "Equation for general computation of hydraulic diameter";
      final parameter SI.Length L = coolingDuctLength "Characteristic length of the surface";

    equation

      for i in 1:channel.N loop
      connect(stator, channel.heatPort[i]) annotation (Line(points={{0,-100},{0,-3.6}},                 color={191,0,0}));
      end for;
      connect(flowPort_in, channel.port_a) annotation (Line(points={{-100,0},{-10,0}},         color={255,127,36}));
      connect(channel.port_b, flowPort_out) annotation (Line(points={{10,0},{100,0}},         color={255,127,36}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}})),           Documentation(info="<html>
<p>This model calculates the influence of a liquid cooling system on the overall thermal behavior of an electrical machine. Depending on the shape of the cooling channels, different formulas found in Staton and Cavagnino (2008, section V) are implemented for laminar and turbulent flow conditions.</p>
<p>Equations are taken from <a href=\"ElectrifiedPowertrains.UsersGuide.Literature\">[SC08]</a>.</p>
</html>"));
    end FluidCooled;

    model ForcedCoolingThreeMasses2
      "Forced cooling AIM with three thermal masses: stator iron, stator copper and rotor (including shaft)"
      import ElectrifiedPowertrains;
      extends CHEETA_Thermal.Motors.ForcedCoolingThreeMassesBase(fluidCooling(
          ductGeometry=ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Types.CoolingDuctGeometry.ConcentricCylinder,
          ductRadius=0.2,
          ductHeight=1,
          ductWidth=1,
          ductInnerRadius=0.5,
          ductOuterRadius=1,
          coolingDuctLength=2,
          dp_nominal=20000,
          m_flowNominal=1,
          d_nominal=1000,
          initType=DassaultSystemes.Fluid.Types.InitType.SteadyStateInitial));

      // Icon
      extends
        ElectrifiedPowertrains.Common.Icons.ThermalMachine.ForcedConvection;

      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor statorCoreTemperatureSensor
        annotation (Placement(transformation(extent={{40,-46},{52,-34}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor statorWindingTemperatureSensor
        annotation (Placement(transformation(extent={{40,-66},{52,-54}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor rotorTemperatureSensor
        annotation (Placement(transformation(extent={{40,-86},{52,-74}})));
      DymolaModels.Blocks.Interfaces.TemperatureOutput rotorTemperature
        annotation (Placement(transformation(extent={{100,-90},{120,-70}})));
      DymolaModels.Blocks.Interfaces.TemperatureOutput statorWindingTemperature
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
      DymolaModels.Blocks.Interfaces.TemperatureOutput statorCoreTemperature
        annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
      DassaultSystemes.Fluid.Interfaces.LiquidPort_b
                                             flowPort_b(
        redeclare package Medium =
            CHEETA_Thermal.Utilities.LH2CoolProp)
        annotation (Placement(transformation(extent={{70,90},{90,110}})));
      DassaultSystemes.Fluid.Interfaces.LiquidPort_a
                                             flowPort_a(
        redeclare package Medium =
            CHEETA_Thermal.Utilities.LH2CoolProp)
        annotation (Placement(transformation(extent={{-90,90},{-70,110}})));
    protected
      Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortAIMC internalThermalPort(final m=3)
        annotation (Placement(transformation(extent={{-10,-90},{10,-70}})));
    public
      Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortAIMC thermalPort(final m=3)
        annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
    equation
      connect(stator.statorAirGap, airGap.airGapStator) annotation (Line(points={{0,30},{0,10}}, color={191,0,0}));
      connect(airGap.airGapRotor, rotor.rotorAirGap) annotation (Line(points={{0,-10},{0,-10},{0,-30}}, color={191,0,0}));
      connect(internalThermalPort.heatPortStrayLoad, stator.strayLoadLosses)
        annotation (Line(points={{0,-80},{0,-80},{0,-78},{-36,-78},{-36,36},{-10,36}},
                                                                         color={199,0,0}));
      connect(stator.coreLosses, internalThermalPort.heatPortStatorCore)
        annotation (Line(points={{-10,40},{-38,40},{-38,-82},{0,-82},{0,-80}},
                                                                             color={191,0,0}));
      connect(internalThermalPort.heatPortStatorWinding, stator.copperLosses)
        annotation (Line(points={{0,-80},{0,-84},{-40,-84},{-40,44},{-10,44}},             color={199,0,0}));
      connect(airGap.flange, flange) annotation (Line(points={{10,0},{10,0},{100,0}}, color={0,0,0}));
      connect(fluidCooling.stator, stator.statorHousing) annotation (Line(points={{0,70},{0,70},{0,50}}, color={191,0,0}));
      connect(fixedTemperature.port, stator.statorInnerAirFront)
        annotation (Line(points={{-20,0},{-14,0},{-14,20},{-6,20},{-6,30}}, color={191,0,0}));
      connect(stator.statorInnerAirRear, fixedTemperature.port)
        annotation (Line(points={{6,30},{6,20},{-14,20},{-14,0},{-20,0}}, color={191,0,0}));
      connect(fixedTemperature.port, rotor.innerAirFront)
        annotation (Line(points={{-20,0},{-18,0},{-14,0},{-14,-20},{-6,-20},{-6,-30}}, color={191,0,0}));
      connect(rotor.innerAirRear, fixedTemperature.port)
        annotation (Line(points={{6,-30},{6,-30},{6,-20},{-14,-20},{-14,0},{-20,0}}, color={191,0,0}));
      connect(rotorTemperatureSensor.port, rotor.rotorAirGap)
        annotation (Line(points={{40,-80},{20,-80},{20,-16},{0,-16},{0,-30}}, color={191,0,0}));
      connect(statorCoreTemperatureSensor.port, stator.statorAirGap)
        annotation (Line(points={{40,-40},{24,-40},{24,26},{0,26},{0,30}}, color={191,0,0}));
      connect(statorCoreTemperatureSensor.T, statorCoreTemperature)
        annotation (Line(points={{52,-40},{110,-40},{110,-40}}, color={0,0,127}));
      connect(statorWindingTemperature, statorWindingTemperatureSensor.T)
        annotation (Line(points={{110,-60},{81,-60},{52,-60}}, color={0,0,127}));
      connect(rotorTemperatureSensor.T, rotorTemperature) annotation (Line(points={{52,-80},{110,-80}},           color={0,0,127}));
      connect(noBearing.bearingLosses, internalThermalPort.heatPortFriction)
        annotation (Line(points={{-70,0},{-80,0},{-80,-86},{0,-86},{0,-80}},   color={191,0,0}));
      connect(internalThermalPort, thermalPort) annotation (Line(points={{0,-80},{0,-80},{0,-102},{0,-100}},
                                                                                                color={199,0,0}));
      connect(stator.copperLosses[1], statorWindingTemperatureSensor.port)
        annotation (Line(points={{-10,43.3333},{-14,43.3333},{-14,24},{22,24},{
              22,-60},{40,-60}},                                                                  color={191,0,0}));
      connect(rotor.coreLosses, internalThermalPort.heatPortRotorCore)
        annotation (Line(points={{-10,-44},{-32,-44},{-32,-74},{0,-74},{0,-80}}, color={191,0,0}));
      connect(rotor.windingLosses, internalThermalPort.heatPortRotorWinding)
        annotation (Line(points={{-10,-36},{-34,-36},{-34,-76},{-1,-76},{-1,-82}}, color={191,0,0}));
      connect(flowPort_a, fluidCooling.flowPort_in) annotation (Line(points={{-80,100},
              {-45,100},{-45,80},{-10,80}}, color={255,127,36}));
      connect(fluidCooling.flowPort_out, flowPort_b) annotation (Line(points={{10,80},
              {45,80},{45,100},{80,100}}, color={255,127,36}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})), Documentation(info="<html>
<p>Uses models from the <a href=\"modelica://ElectrifiedPowertrains.ElectricMachines.Generic.Thermal\">Generic.Thermal</a> package to build a three-mass thermal model for forces air cooling.</p>
<p>A fan can be attached externally as shown in <a href=\"modelica://ElectrifiedPowertrains.Examples.ElectricDrives.ForcedCoolingAIM\">ForcedCoolingAIM</a></p> to generate air flow.
<br>
</html>"));
    end ForcedCoolingThreeMasses2;

    partial model ForcedCoolingThreeMassesBase
      "Machine with forced cooling and three thermal masses: stator iron, stator copper and rotor (including shaft)"
      import ElectrifiedPowertrains;
      import SI = Modelica.SIunits;

      // Icon
      extends DymolaModels.Icons.Basic.Interface;

      // Parameters
      parameter Modelica.Thermal.FluidHeatFlow.Media.Medium coolingMedium "Cooling medium used to externally cool the motor" annotation(choicesAllMatching=true, Dialog(group="Fluid Cooling"));
      parameter Modelica.Thermal.FluidHeatFlow.Media.Medium airGapMedium "Cooling medium used in the motor" annotation(choicesAllMatching=true, Dialog(group="Air Gap"));
      parameter SI.Temperature initialTemperature
        "Initial temperatures of all thermal masses within the model";

      extends
        ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.ForcedConvection.Records.Base.ForcedConvection;
      extends
        ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Stator.Records.Base.TwoMassesGeometry;
      extends
        ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.AirGap.Records.Base.PolynomialApproximation;
      extends
        ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Rotor.Records.Base.OneMassGeometry;

      Motors.FluidCooled fluidCooling(
        mediumMass=mediumMass,
        temperatureTap=temperatureTap,
        T0=initialTemperature)
        annotation (Placement(transformation(extent={{-10,70},{10,90}})));

      ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Rotor.CoreElements.OneMass rotor(
        heatCapacity=heatCapacity,
        initialTemperature=initialTemperature) annotation (Placement(transformation(extent={{-10,-50},{10,-30}})));
      ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.AirGap.CoreElements.PolynomialApproximation airGap(
        medium=airGapMedium,
        airGapThickness=airGapThickness,
        rotorRadius=rotorRadius,
        coreLength=coreLength) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Stator.CoreElements.TwoMasses stator(
        initialTemperature=initialTemperature,
        conductanceCopperIronCoupling=conductanceCopperIronCoupling,
        heatCapacityIron=heatCapacityIron,
        heatCapacityWinding=heatCapacityWinding) annotation (Placement(transformation(extent={{-10,30},{10,50}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange annotation (Placement(transformation(extent={{90,-10},{110,10}})));

      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=293.15)
        annotation (Placement(transformation(extent={{-28,-4},{-20,4}})));
      ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Bearing.None noBearing
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
    equation
      connect(stator.statorAirGap, airGap.airGapStator) annotation (Line(points={{0,30},{0,10}}, color={191,0,0}));
      connect(airGap.airGapRotor, rotor.rotorAirGap) annotation (Line(points={{0,-10},{0,-10},{0,-30}}, color={191,0,0}));
      connect(airGap.flange, flange) annotation (Line(points={{10,0},{10,0},{100,0}}, color={0,0,0}));
      connect(fluidCooling.stator, stator.statorHousing) annotation (Line(points={{0,70},{0,70},{0,50}}, color={191,0,0}));
      connect(fixedTemperature.port, stator.statorInnerAirFront)
        annotation (Line(points={{-20,0},{-14,0},{-14,20},{-6,20},{-6,30}}, color={191,0,0}));
      connect(stator.statorInnerAirRear, fixedTemperature.port)
        annotation (Line(points={{6,30},{6,20},{-14,20},{-14,0},{-20,0}}, color={191,0,0}));
      connect(fixedTemperature.port, rotor.innerAirFront)
        annotation (Line(points={{-20,0},{-18,0},{-14,0},{-14,-20},{-6,-20},{-6,-30}}, color={191,0,0}));
      connect(rotor.innerAirRear, fixedTemperature.port)
        annotation (Line(points={{6,-30},{6,-30},{6,-20},{-14,-20},{-14,0},{-20,0}}, color={191,0,0}));
      connect(noBearing.bearingShaft, rotor.rotorShaft)
        annotation (Line(points={{-66,-10},{-66,-70},{0,-70},{0,-50}}, color={191,0,0}));
      connect(noBearing.bearingEndCap, stator.statorHousing)
        annotation (Line(points={{-66,10},{-66,10},{-66,42},{-66,60},{0,60},{0,50}},        color={191,0,0}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end ForcedCoolingThreeMassesBase;

    partial model GenericForcedCooling2
      import ElectrifiedPowertrains;
      import Cooling;
      extends DymolaModels.Icons.Basic.Interface;

      DassaultSystemes.Fluid.Interfaces.LiquidPort_a
                                             flowPort_a(
        redeclare package Medium=Medium)
        annotation (Placement(transformation(extent={{-90,90},{-70,110}})));
      DassaultSystemes.Fluid.Interfaces.LiquidPort_b flowPort_b(
        redeclare package Medium=Medium)
        annotation (Placement(transformation(extent={{70,90},{90,110}})));

    package Medium =
            CHEETA_Thermal.Utilities.LH2CoolProp
      annotation(choicesAllMatching=true);

      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end GenericForcedCooling2;

    model LinearSquirrelCage "AIM: Linear with squirrel cage rotor"
      import ElectrifiedPowertrains;
      import TerminalConnections = DymolaModels.Electrical.MultiPhase.Choices.Wiring;

      // Interface
      extends
        ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Interfaces.ThreePhase;

      // Parameters
      extends DymolaModels.ElectroMechanical.Parameters.MachineInterfaces;
      replaceable parameter
        ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Base.Linear
        data constrainedby
        ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Base.Linear(
          final useThermalPort=useThermalPort, final useSupport=useSupport)
        annotation (choicesAllMatching=true, Placement(transformation(extent={{
                60,60},{80,80}})));

      // Icon
      extends ElectrifiedPowertrains.Common.Icons.ElectricMachines.BlueMachine;
      extends
        ElectrifiedPowertrains.Common.Icons.ElectricMachines.SubIcons.RotorSquirrel;

      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.CoreElements.LinearSquirrelCage
        coreElement(
        p=data.p,
        Rs=data.Rs,
        Lssigma=data.Ls_sigma,
        Jr=data.Jr,
        Lm=data.Lm,
        Lrsigma=data.Lr_sigma,
        Rr=data.Rr,
        fsNominal=data.fs_nom,
        TsOperational=data.Ts_operational,
        TsRef=data.Ts_ref,
        Lszero=data.Ls_zero,
        Js=data.Js,
        frictionParameters=data.frictionParameters,
        statorCoreParameters=data.statorCoreParameters,
        strayLoadParameters=data.strayLoadParameters,
        TrRef=data.Tr_ref,
        TrOperational=data.Tr_operational,
        final useSupport=useSupport,
        final useThermalPort=useThermalPort,
        alpha20s=data.alpha20s,
        alpha20r=data.alpha20r)
        annotation (Placement(transformation(extent={{-60,-20},{-20,20}})));
      ElectrifiedPowertrains.ElectricMachines.Common.Models.TerminalBox terminalBox(
          terminalConnection=data.terminalConnection) annotation (Placement(
            transformation(extent={{-60,20},{-20,60}}, rotation=0)));

      ElectrifiedPowertrains.ElectricDrives.Interfaces.Adapters.ToBus.CosPhi cosPhi
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={-40,-74})));
      Modelica.Blocks.Math.Product innerPower
        annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={20,-36})));
      Modelica.Blocks.Math.Gain signChange(k=-1) "Going from generator to consumer reference arrow system"
        annotation (Placement(transformation(extent={{6,-14},{14,-6}})));
      Modelica.Electrical.MultiPhase.Basic.Delta delta
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=0,
            origin={-94,90})));
      Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=0,
            origin={-80,80})));
      DymolaModels.Electrical.MultiPhase.Sensors.QuasiPowerSensor quasiPowerSensor
        annotation (Placement(transformation(extent={{-54,53},{-40,67}})));
      Modelica.Electrical.MultiPhase.Basic.Star star(final m=3)
        annotation (Placement(transformation(
            origin={-47,47},
            extent={{-6,6},{6,-6}},
            rotation=270)));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=0,
            origin={-47,35})));
      ElectrifiedPowertrains.ElectricDrives.Interfaces.Adapters.ToBus.LineCurrents toBus_lineCurrents
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={-100,-74})));
      ElectrifiedPowertrains.ElectricDrives.Interfaces.Adapters.ToBus.LineVoltages toBus_lineVoltages
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={-80,-74})));
      ElectrifiedPowertrains.ElectricDrives.Interfaces.Adapters.ToBus.RotorSpeed toBus_shaftSpeed
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={40,-74})));
      ElectrifiedPowertrains.ElectricDrives.Interfaces.Adapters.ToBus.RotorAngle toBus_shaftAngle
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={60,-74})));
      ElectrifiedPowertrains.ElectricDrives.Interfaces.Adapters.ToBus.OuterTorque toBus_outerTorque
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={80,-74})));
      ElectrifiedPowertrains.ElectricDrives.Interfaces.Adapters.ToBus.InnerTorque toBus_innerTorque
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={0,-74})));
      ElectrifiedPowertrains.ElectricDrives.Interfaces.Adapters.ToBus.ElectricPower toBus_electricPower
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={-60,-74})));
      ElectrifiedPowertrains.ElectricDrives.Interfaces.Adapters.ToBus.MechanicPower toBus_mechanicPower
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={20,-74})));
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Interfaces.TemperaturesToBus toBus_temperatures
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={-20,-74})));
      Modelica.Mechanics.Rotational.Sensors.RelAngleSensor
                                                        angleSensor
        annotation (Placement(transformation(
            extent={{4,-4},{-4,4}},
            rotation=270,
            origin={68,-10})));
      Modelica.Mechanics.Rotational.Sensors.RelSpeedSensor
                                                        speedSensor
        annotation (Placement(transformation(
            extent={{4,-4},{-4,4}},
            rotation=270,
            origin={48,-10})));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor annotation (Placement(transformation(extent={{79,-4},{87,4}})));
      Modelica.Mechanics.Rotational.Components.Fixed fixed if (not useSupport)
        annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=90,
            origin={70,-100})));
    public
      Modelica.Electrical.MultiPhase.Sensors.CurrentSensor   currentSensor
        annotation (Placement(transformation(extent={{-96,54},{-84,66}})));
    protected
      Modelica.Mechanics.Rotational.Interfaces.Support internalSupport
        annotation (Placement(transformation(extent={{76,-104},{84,-96}})));
    equation
      connect(terminalBox.plug_sn, coreElement.plug_sn)
        annotation (Line(
          points={{-52,20},{-52,20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(terminalBox.plug_sp, coreElement.plug_sp)
        annotation (Line(
          points={{-28,20},{-28,20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(coreElement.thermalPort, thermalPort)
        annotation (Line(
          points={{-40,-20},{-64,-20},{-64,100},{60,100}},
          color={199,0,0},
          smooth=Smooth.None));
      connect(cosPhi.electricDriveBus, electricDriveBus)
        annotation (Line(
          points={{-40,-80},{-40,-90},{0,-90},{0,-100}},
          color={0,86,166},
          thickness=0.5));
      connect(coreElement.innerTorque, toBus_innerTorque.u) annotation (Line(points={{-18,-10},{1.33227e-015,-10},{1.33227e-015,-66.8}},
                                                                                                                   color={0,0,127}));
      connect(innerPower.y, toBus_mechanicPower.u) annotation (Line(points={{20,-40.4},{20,-66.8}},            color={0,0,127}));
      connect(signChange.u, coreElement.innerTorque) annotation (Line(points={{5.2,-10},{5.2,-10},{-18,-10}}, color={0,0,127}));
      connect(signChange.y, innerPower.u2) annotation (Line(points={{14.4,-10},{17.6,-10},{17.6,-31.2}}, color={0,0,127}));
      connect(delta.plug_p, voltageSensor.plug_p) annotation (Line(points={{-100,90},{-100,80},{-86,80}},
                                                                                             color={0,0,255}));
      connect(voltageSensor.plug_n, delta.plug_n) annotation (Line(points={{-74,80},{-74,90},{-88,90}},
                                                                                                     color={0,0,255}));
      connect(voltageSensor.plug_p, plug_p) annotation (Line(points={{-86,80},{-100,80},{-100,0}},                color={0,0,255}));
      connect(quasiPowerSensor.nc, terminalBox.plugSupply) annotation (Line(points={{-40,60},{-40,24}}, color={0,0,255}));
      connect(quasiPowerSensor.pv, terminalBox.plug_sp) annotation (Line(points={{-47,67},{-28,67},{-28,20}}, color={0,0,255}));
      connect(quasiPowerSensor.P, toBus_electricPower.u)
        annotation (Line(points={{-53.3,57.2},{-56,57.2},{-56,54},{-78,54},{-78,-62},{-60,-62},{-60,-66.8}}, color={0,0,127}));
      connect(currentSensor.i, toBus_lineCurrents.u)
        annotation (Line(points={{-90,53.4},{-90,52},{-82,52},{-82,-60},{-100,-60},{-100,-66.8}},
                                                                                  color={0,0,127}));
      connect(quasiPowerSensor.cosphi, cosPhi.u)
        annotation (Line(points={{-49.8,53.7},{-49.8,52},{-76,52},{-76,-60},{-40,-60},{-40,-66.8}}, color={0,0,127}));
      connect(quasiPowerSensor.nv, star.plug_p) annotation (Line(points={{-47,53},{-47,53}}, color={0,0,255}));
      connect(voltageSensor.v, toBus_lineVoltages.u)
        annotation (Line(points={{-80,73.4},{-80,-66.8}},                                     color={0,0,127}));
      connect(ground.p, star.pin_n) annotation (Line(points={{-47,41},{-47,41}}, color={0,0,255}));
      connect(toBus_lineCurrents.electricDriveBus, electricDriveBus)
        annotation (Line(
          points={{-100,-80},{-100,-90},{0,-90},{0,-100}},
          color={0,86,166},
          thickness=0.5));
      connect(toBus_lineVoltages.electricDriveBus, electricDriveBus)
        annotation (Line(
          points={{-80,-80},{-80,-90},{0,-90},{0,-100}},
          color={0,86,166},
          thickness=0.5));
      connect(toBus_outerTorque.electricDriveBus, electricDriveBus)
        annotation (Line(
          points={{80,-80},{80,-90},{0,-90},{0,-100}},
          color={0,86,166},
          thickness=0.5));
      connect(toBus_shaftSpeed.electricDriveBus, electricDriveBus)
        annotation (Line(
          points={{40,-80},{40,-90},{0,-90},{0,-100}},
          color={0,86,166},
          thickness=0.5));
      connect(toBus_shaftAngle.electricDriveBus, electricDriveBus)
        annotation (Line(
          points={{60,-80},{60,-90},{0,-90},{0,-100}},
          color={0,86,166},
          thickness=0.5));
      connect(toBus_innerTorque.electricDriveBus, electricDriveBus)
        annotation (Line(
          points={{0,-80},{0,-80},{0,-100}},
          color={0,86,166},
          thickness=0.5));
      connect(toBus_electricPower.electricDriveBus, electricDriveBus)
        annotation (Line(
          points={{-60,-80},{-60,-90},{0,-90},{0,-100}},
          color={0,86,166},
          thickness=0.5));
      connect(toBus_mechanicPower.electricDriveBus, electricDriveBus)
        annotation (Line(
          points={{20,-80},{20,-90},{0,-90},{0,-100}},
          color={0,86,166},
          thickness=0.5));
      connect(toBus_temperatures.electricDriveBus, electricDriveBus)
        annotation (Line(
          points={{-20,-80},{-20,-90},{0,-90},{0,-100}},
          color={0,86,166},
          thickness=0.5));
      connect(torqueSensor.tau, toBus_outerTorque.u)
        annotation (Line(
          points={{79.8,-4.4},{79.8,-4},{80,-4},{80,-66.8}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(speedSensor.w_rel, innerPower.u1) annotation (Line(points={{43.6,-10},{22.4,-10},{22.4,-31.2}}, color={0,0,127}));
      connect(speedSensor.w_rel, toBus_shaftSpeed.u) annotation (Line(points={{43.6,-10},{40,-10},{40,-66.8}}, color={0,0,127}));
      connect(angleSensor.phi_rel, toBus_shaftAngle.u) annotation (Line(points={{63.6,-10},{60,-10},{60,-66.8}}, color={0,0,127}));
      connect(torqueSensor.flange_a, coreElement.flange) annotation (Line(points={{79,0},{-20,0}},        color={0,0,0}));
      connect(speedSensor.flange_b, coreElement.flange) annotation (Line(points={{48,-6},{48,0},{-20,0}}, color={0,0,0}));
      connect(angleSensor.flange_b, coreElement.flange) annotation (Line(points={{68,-6},{68,0},{-20,0}}, color={0,0,0}));
      connect(torqueSensor.flange_b, flange) annotation (Line(points={{87,0},{100,0}}, color={0,0,0}));
      connect(fixed.flange, internalSupport) annotation (Line(points={{70,-100},{80,-100}}, color={0,0,0}));
      connect(internalSupport, support) annotation (Line(points={{80,-100},{90,-100},{100,-100}}, color={0,0,0}));
      connect(angleSensor.flange_a, internalSupport)
        annotation (Line(points={{68,-14},{68,-50},{90,-50},{90,-100},{80,-100}}, color={0,0,0}));
      connect(speedSensor.flange_a, internalSupport)
        annotation (Line(points={{48,-14},{48,-50},{90,-50},{90,-100},{80,-100}}, color={0,0,0}));
      connect(coreElement.support, internalSupport)
        annotation (Line(points={{-20,-20},{-20,-50},{90,-50},{90,-100},{80,-100}}, color={0,0,0}));
      connect(coreElement.temperatures, toBus_temperatures.u)
        annotation (Line(points={{-18,-14},{-10,-14},{-10,-60},{-20,-60},{-20,-66.8}}, color={191,0,0}));
      connect(quasiPowerSensor.pc, currentSensor.plug_n) annotation (Line(points={{-54,60},{-84,60}}, color={0,0,255}));
      connect(currentSensor.plug_p, plug_p) annotation (Line(points={{-96,60},{-100,60},{-100,0}}, color={0,0,255}));
      annotation (
          defaultComponentName="squirrelCageAIM",
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
                                                                                                       Rectangle(
              extent={{-15,18},{15,-18}},
              lineColor={28,108,200},
              radius=2,
              pattern=LinePattern.Dash,
              origin={-88,87},
              rotation=270,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
                            Text(
              extent={{-16,2},{16,-2}},
              lineColor={28,108,200},
              pattern=LinePattern.Dash,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              origin={-88,98},
              rotation=360,
              textString="line to line voltages")}),
        Documentation(info="<html>
<p>Utilizes the <a href=\"modelica://ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.CoreElements.LinearSquirrelCage\">CoreElement</a> with parameter handling and bus connection.</p>
<p>The variables which are necessary for the controller or for some other computations (e.g. efficiency) are written to the bus.</p>
</html>"));
    end LinearSquirrelCage;
  end Motors;

  package Battery
  end Battery;

  package Fuel_Cell
  end Fuel_Cell;

  package HTS
    model FluidCooled
      "Computes the thermal resistance between housing and coolant fluid"
      import SI = Modelica.SIunits;
      import Modelica.Constants.pi;

      //extends Modelica.Thermal.FluidHeatFlow.Interfaces.Partials.TwoPort(final tapT = temperatureTap, final m = mediumMass);

      // Icon
     // extends ElectrifiedPowertrains.Common.Icons.SubIcons.Background;
     extends DymolaModels.Icons.Components.Boxed;
      extends ElectrifiedPowertrains.Common.Icons.ThermalMachine.LiquidCooling;

      // Parameters
      extends
        ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.ForcedConvection.Records.Base.FluidCooled;

      replaceable package Medium = Cooling.Media.Liquids.EthyleneGlycol52 constrainedby
        Cooling.Media.Templates.BaseLiquid
      annotation(choicesAllMatching=true);
      parameter SI.PressureDifference dp_nominal;
      parameter SI.MassFlowRate m_flowNominal;
      parameter SI.Density d_nominal;
      parameter DassaultSystemes.Fluid.Types.InitType initType = DassaultSystemes.Fluid.Types.InitType.FixedInitial;
      parameter Boolean initialize_T = true;
      parameter SI.MassFlowRate m_flowInit = 0.1;
      parameter SI.Temperature T0(start=293.15, displayUnit="degC")
        "Initial temperature of medium";
      parameter SI.AbsolutePressure p_aInit = Medium.p_default;
      parameter SI.AbsolutePressure p_bInit = Medium.p_default;

      DassaultSystemes.Fluid.Interfaces.LiquidPort_a
                                             flowPort_in(
        redeclare package Medium=Medium)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      DassaultSystemes.Fluid.Interfaces.LiquidPort_b
                                             flowPort_out(
        redeclare package Medium=Medium)
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));

      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a stator annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
      DassaultSystemes.Fluid.Pipes.DiscretizedLiquidPipe
                                                    channel(
        redeclare package Medium = Medium,
        allowFlowReversal=false,
        useHeatTransfer=true,
        dp_nominal=dp_nominal,
        m_flowNominal=m_flowNominal,
        d_nominal=d_nominal,
        final A=ductCrossSection,
        final perimeter=ductPerimeter,
        final A_ht=coolingSurfaceArea,
        initType=initType,
        initialize_T=initialize_T,
        m_flowInit=m_flowInit,
        p_aInit=p_aInit,
        p_bInit=p_bInit,
        T_aInit=T0,
        T_bInit=T0,
        final useDynamicMomentum=false,
        final L=L,
        redeclare model HT =
            ThermalManagementDemos.EPTLExample.LiquidJacketCircularChannel,
        redeclare model DP = Cooling.Pipes.Friction.LaminarTurbulentPipeFlow (from_dp=false, r=0.0),
        N=1) annotation (Placement(transformation(extent={{-10,10},{10,-10}})));

      final parameter SI.Area ductCrossSection=if ductGeometry == ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Types.CoolingDuctGeometry.Circular
           then pi*ductRadius^2 else if ductGeometry == ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Types.CoolingDuctGeometry.Rectangular
           then ductHeight*ductWidth else ductOuterRadius^2*pi - ductInnerRadius^2*pi "Cross-section of cooling duct";
      final parameter SI.Length ductPerimeter=if ductGeometry == ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Types.CoolingDuctGeometry.Circular
           then 2*pi*ductRadius else if ductGeometry == ElectrifiedPowertrains.ElectricMachines.Generic.Thermal.Types.CoolingDuctGeometry.Rectangular
           then 2*(ductHeight + ductWidth) else 2*pi*(ductOuterRadius + ductInnerRadius) "Surface area for cooling";
      final parameter SI.Area coolingSurfaceArea=ductPerimeter*coolingDuctLength "Surface area for cooling";
      final parameter SI.Diameter D=4*ductCrossSection/ductPerimeter "Equation for general computation of hydraulic diameter";
      final parameter SI.Length L = coolingDuctLength "Characteristic length of the surface";

    equation

      for i in 1:channel.N loop
      connect(stator, channel.heatPort[i]) annotation (Line(points={{0,-100},{0,-3.6}},                 color={191,0,0}));
      end for;
      connect(flowPort_in, channel.port_a) annotation (Line(points={{-100,0},{-10,0}},         color={255,127,36}));
      connect(channel.port_b, flowPort_out) annotation (Line(points={{10,0},{100,0}},         color={255,127,36}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}})),           Documentation(info="<html>
<p>This model calculates the influence of a liquid cooling system on the overall thermal behavior of an electrical machine. Depending on the shape of the cooling channels, different formulas found in Staton and Cavagnino (2008, section V) are implemented for laminar and turbulent flow conditions.</p>
<p>Equations are taken from <a href=\"ElectrifiedPowertrains.UsersGuide.Literature\">[SC08]</a>.</p>
</html>"));
    end FluidCooled;
  end HTS;

  package Tests

    package Inverter_Tests
      model ThermalConductanceBasePlate
        Inverters.ThermalConductanceWithBaseplate thermalConductanceWithBaseplate(
            redeclare
            ElectrifiedPowertrains.PowerElectronics.Inverters.Generic.Thermal.Records.Data.ThermalConductance.SKM900GA12E4
            data(
            G_thJC_IGBT=28.571,
            G_thJC_diode=14.285,
            G_thCS_module=26.315,
            n_switches=6,
            n_modules=2))
          annotation (Placement(transformation(extent={{64,-2},{84,18}})));
        Cooling.Machines.LiquidPumpControlled pump(
          redeclare package Medium = Utilities.LH2CoolProp,
          Nparallel=1,
          m_flowNominal=0.02,
          allowFlowReversal=true,
          checkValve=false,
          m_flowInit=0.02,
          d_init=1044,
          n_set=2000,
          useSpeedInput=true,
          p_aInit=104325,
          p_bInit=164325,
          redeclare Cooling.Machines.Records.Data.PumpData.Default mapData(n_nominal=2000))
                          annotation (Placement(transformation(extent={{22,52},
                  {42,72}})));
        DymolaModels.Blocks.UnitConversions.FromLitersPerMinute UnitConversion
          annotation (Placement(transformation(extent={{142,-138},{162,-118}})));
        Modelica.Blocks.Math.UnitConversions.To_degC Kelvin2Celsius
          annotation (Placement(transformation(extent={{18,-138},{38,-118}})));
        Modelica.Blocks.Continuous.PID PID(
          k=5,
          Ti=0.1,
          Td=0.01) annotation (Placement(transformation(extent={{72,-102},{92,-82}})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold=100)
          annotation (Placement(transformation(extent={{58,-138},{78,-118}})));
        Modelica.Blocks.Logical.Switch switch1
          annotation (Placement(transformation(extent={{104,-138},{124,-118}})));
        Modelica.Blocks.Sources.Constant NoOutput(k=0)
          annotation (Placement(transformation(extent={{74,-164},{94,-144}})));
        Modelica.Blocks.Sources.RealExpression HeatSinkTemp(y=
              thermalConductanceWithBaseplate.heatSinkTemperature)
          annotation (Placement(transformation(extent={{-22,-138},{-2,-118}})));
        Cooling.Sources.LiquidPressureBoundary inlet1(
          redeclare package Medium = Utilities.LH2CoolProp,
          N=1,
          p_set=100000,
          T_set=293.15) annotation (Placement(transformation(extent={{-14,52},{
                  6,72}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=UnitConversion.y)
          annotation (Placement(transformation(extent={{-70,68},{8,94}})));
        Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow
          annotation (Placement(transformation(extent={{26,-32},{46,-12}})));
        Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow1
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={132,-22})));
        Modelica.Blocks.Sources.Constant NoOutput1(k=1)
          annotation (Placement(transformation(extent={{-28,-32},{-8,-12}})));
        Modelica.Blocks.Sources.Constant NoOutput2(k=2)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=180,
              origin={174,-22})));
        Cooling.HeatExchangers.ColdPlates.SingleChannelMeandric coldPlate1(
          redeclare package Medium = Utilities.LH2CoolProp,
          dp_nominal=2000,
          m_flowNominal=1,
          N_passPairs=2,
          L_x=0.213,
          L_y=0.347,
          D_channel=0.0107,
          redeclare record WallMaterial =
              DassaultSystemes.Common.MaterialProperties.SolidMaterials.Copper_0degC)
                           annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=180,
              origin={74,40})));
        inner Cooling.Common.SystemSettings systemSettings
          annotation (Placement(transformation(extent={{192,-184},{212,-164}})));
        Cooling.Sources.LiquidPressureBoundary inlet2(
          redeclare package Medium = Utilities.LH2CoolProp,
          N=1,
          p_set=100000,
          T_set=293.15) annotation (Placement(transformation(extent={{-10,-10},
                  {10,10}},
              rotation=180,
              origin={160,62})));
      protected
        ElectrifiedPowertrains.PowerElectronics.Inverters.Interfaces.ThermalPortInverter thermalPortInverter1
          annotation (Placement(transformation(extent={{68,-28},{80,-16}})));
      equation
        connect(Kelvin2Celsius.y,greaterThreshold. u)
          annotation (Line(points={{39,-128},{56,-128}},
                                                       color={0,0,127}));
        connect(greaterThreshold.y,switch1. u2)
          annotation (Line(points={{79,-128},{102,-128}},
                                                    color={255,0,255}));
        connect(switch1.y,UnitConversion. u)
          annotation (Line(points={{125,-128},{140,-128}},
                                                     color={0,0,127}));
        connect(PID.u,Kelvin2Celsius. y) annotation (Line(points={{70,-92},{48,-92},{48,
                -128},{39,-128}},
                               color={0,0,127}));
        connect(PID.y,switch1. u1) annotation (Line(points={{93,-92},{96,-92},{96,-120},
                {102,-120}},
                      color={0,0,127}));
        connect(NoOutput.y,switch1. u3) annotation (Line(points={{95,-154},{98,-154},{
                98,-136},{102,-136}},
                          color={0,0,127}));
        connect(HeatSinkTemp.y,Kelvin2Celsius. u)
          annotation (Line(points={{-1,-128},{16,-128}},
                                                       color={0,0,127}));
        connect(pump.port_a, inlet1.port[1])
          annotation (Line(points={{22,62},{6,62}},    color={255,127,36}));
        connect(realExpression.y, pump.n_in)
          annotation (Line(points={{11.9,81},{32,81},{32,72}},color={0,0,127}));
        connect(thermalConductanceWithBaseplate.thermalPortInverter,
          thermalPortInverter1) annotation (Line(points={{74,-2},{75,-2},{75,-22},{74,
                -22}}, color={199,0,0}));
        connect(prescribedHeatFlow.port, thermalPortInverter1.heatPort_switch)
          annotation (Line(points={{46,-22},{60,-22},{60,-20.8},{73.4,-20.8}}, color={
                191,0,0}));
        connect(prescribedHeatFlow1.port, thermalPortInverter1.heatPort_diode)
          annotation (Line(points={{122,-22},{100,-22},{100,-20.8},{73.4,-20.8}},
              color={191,0,0}));
        connect(prescribedHeatFlow.Q_flow, NoOutput1.y)
          annotation (Line(points={{26,-22},{-7,-22}}, color={0,0,127}));
        connect(prescribedHeatFlow1.Q_flow, NoOutput2.y)
          annotation (Line(points={{142,-22},{163,-22}}, color={0,0,127}));
        connect(pump.port_b, coldPlate1.port_a)
          annotation (Line(points={{42,62},{68,62},{68,50}},
                                                    color={255,127,36}));
        connect(coldPlate1.heatPorts[1, 1], thermalConductanceWithBaseplate.heatPort_heatSink)
          annotation (Line(points={{74,30},{75,30},{75,18},{74,18}}, color={127,0,0}));
        connect(coldPlate1.port_b, inlet2.port[1]) annotation (Line(points={{80,
                50},{80,62},{150,62}}, color={255,127,36}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-200},
                  {220,100}}), graphics={
              Ellipse(lineColor={0,86,134},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-98,-204},{220,96}}),
              Polygon(
                lineColor={0,86,134},
                fillColor={0,86,134},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-32,74},{-32,74},{218,-34},{218,-34},{-30,-178},{-30,-180},{-32,
                    74}},
                smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=
                 false, extent={{-100,-200},{220,100}})));
      end ThermalConductanceBasePlate;

      model FiveElementFoster
        Cooling.Machines.LiquidPumpControlled pump(
          redeclare package Medium =
              Utilities.LH2CoolProp,
          Nparallel=1,
          m_flowNominal=0.02,
          allowFlowReversal=true,
          checkValve=false,
          m_flowInit=0.02,
          d_init=1044,
          n_set=2000,
          useSpeedInput=true,
          p_aInit=104325,
          p_bInit=164325,
          redeclare Cooling.Machines.Records.Data.PumpData.Default mapData(n_nominal=2000))
                          annotation (Placement(transformation(extent={{-18,54},{2,74}})));
        DymolaModels.Blocks.UnitConversions.FromLitersPerMinute UnitConversion
          annotation (Placement(transformation(extent={{142,-138},{162,-118}})));
        Modelica.Blocks.Math.UnitConversions.To_degC Kelvin2Celsius
          annotation (Placement(transformation(extent={{18,-138},{38,-118}})));
        Modelica.Blocks.Continuous.PID PID(
          k=5,
          Ti=0.1,
          Td=0.01) annotation (Placement(transformation(extent={{72,-102},{92,-82}})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold=100)
          annotation (Placement(transformation(extent={{58,-138},{78,-118}})));
        Modelica.Blocks.Logical.Switch switch1
          annotation (Placement(transformation(extent={{104,-138},{124,-118}})));
        Modelica.Blocks.Sources.Constant NoOutput(k=0)
          annotation (Placement(transformation(extent={{74,-164},{94,-144}})));
        Modelica.Blocks.Sources.RealExpression HeatSinkTemp(y=
              fiveElementsWithCooling2_1.heatSinkTemperature)
          annotation (Placement(transformation(extent={{-22,-138},{-2,-118}})));
        Cooling.Sources.LiquidPressureBoundary inlet1(
          redeclare package Medium =
              Utilities.LH2CoolProp,
          N=1,
          p_set=100000,
          T_set=293.15) annotation (Placement(transformation(extent={{-50,54},{-30,74}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=UnitConversion.y)
          annotation (Placement(transformation(extent={{-60,74},{-40,94}})));
        Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow
          annotation (Placement(transformation(extent={{26,-32},{46,-12}})));
        Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow1
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={132,-22})));
        Modelica.Blocks.Sources.Constant NoOutput1(k=0)
          annotation (Placement(transformation(extent={{-28,-32},{-8,-12}})));
        Modelica.Blocks.Sources.Constant NoOutput2(k=0)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=180,
              origin={174,-22})));
        inner Cooling.Common.SystemSettings systemSettings
          annotation (Placement(transformation(extent={{192,-184},{212,-164}})));
        Cooling.Sources.LiquidPressureBoundary inlet2(
          redeclare package Medium =
              Utilities.LH2CoolProp,
          p_set=100000,
          T_set=293.15,
          N=1)          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=180,
              origin={160,62})));

        Inverters.FiveElementsWithCooling fiveElementsWithCooling2_1(redeclare
            package Medium =
              CHEETA_Thermal.Utilities.LH2CoolProp)
          annotation (Placement(transformation(extent={{60,12},{80,32}})));
      protected
        ElectrifiedPowertrains.PowerElectronics.Inverters.Interfaces.ThermalPortInverter thermalPortInverter1
          annotation (Placement(transformation(extent={{68,-28},{80,-16}})));
      equation
        connect(Kelvin2Celsius.y,greaterThreshold. u)
          annotation (Line(points={{39,-128},{56,-128}},
                                                       color={0,0,127}));
        connect(greaterThreshold.y,switch1. u2)
          annotation (Line(points={{79,-128},{102,-128}},
                                                    color={255,0,255}));
        connect(switch1.y,UnitConversion. u)
          annotation (Line(points={{125,-128},{140,-128}},
                                                     color={0,0,127}));
        connect(PID.u,Kelvin2Celsius. y) annotation (Line(points={{70,-92},{48,-92},{48,
                -128},{39,-128}},
                               color={0,0,127}));
        connect(PID.y,switch1. u1) annotation (Line(points={{93,-92},{96,-92},{96,-120},
                {102,-120}},
                      color={0,0,127}));
        connect(NoOutput.y,switch1. u3) annotation (Line(points={{95,-154},{98,-154},{
                98,-136},{102,-136}},
                          color={0,0,127}));
        connect(HeatSinkTemp.y,Kelvin2Celsius. u)
          annotation (Line(points={{-1,-128},{16,-128}},
                                                       color={0,0,127}));
        connect(pump.port_a,inlet1. port[1])
          annotation (Line(points={{-18,64},{-30,64}}, color={255,127,36}));
        connect(realExpression.y,pump. n_in)
          annotation (Line(points={{-39,84},{-8,84},{-8,74}}, color={0,0,127}));
        connect(prescribedHeatFlow.port,thermalPortInverter1. heatPort_switch)
          annotation (Line(points={{46,-22},{60,-22},{60,-20.8},{73.4,-20.8}}, color={
                191,0,0}));
        connect(prescribedHeatFlow1.port,thermalPortInverter1. heatPort_diode)
          annotation (Line(points={{122,-22},{100,-22},{100,-20.8},{73.4,-20.8}},
              color={191,0,0}));
        connect(prescribedHeatFlow.Q_flow,NoOutput1. y)
          annotation (Line(points={{26,-22},{-7,-22}}, color={0,0,127}));
        connect(prescribedHeatFlow1.Q_flow,NoOutput2. y)
          annotation (Line(points={{142,-22},{163,-22}}, color={0,0,127}));
        connect(pump.port_b, fiveElementsWithCooling2_1.flowPort_in) annotation (Line(
              points={{2,64},{32,64},{32,32},{62,32}}, color={255,127,36}));
        connect(inlet2.port[1], fiveElementsWithCooling2_1.flowPort_out) annotation (
            Line(points={{150,62},{114,62},{114,32},{78,32}}, color={255,127,36}));
        connect(thermalPortInverter1, fiveElementsWithCooling2_1.thermalPortInverter)
          annotation (Line(points={{74,-22},{72,-22},{72,12},{70,12}}, color={199,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-220},
                  {220,100}}), graphics={
              Ellipse(lineColor={0,86,134},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-98,-204},{220,96}}),
              Polygon(
                lineColor={0,86,134},
                fillColor={0,86,134},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-32,74},{-32,74},{218,-34},{218,-34},{-30,-178},{-30,-180},{-32,
                    74}},
                smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=
                 false, extent={{-100,-220},{220,100}})));
      end FiveElementFoster;

      model ThermalInpedanceCooling
        Cooling.Machines.LiquidPumpControlled pump(
          redeclare package Medium =
              Utilities.LH2CoolProp,
          Nparallel=1,
          m_flowNominal=0.02,
          allowFlowReversal=true,
          checkValve=false,
          m_flowInit=0.02,
          d_init=1044,
          n_set=2000,
          useSpeedInput=true,
          p_aInit=104325,
          p_bInit=164325,
          redeclare Cooling.Machines.Records.Data.PumpData.Default mapData(
              n_nominal=2000))
                          annotation (Placement(transformation(extent={{-28,42},
                  {-8,62}})));
        DymolaModels.Blocks.UnitConversions.FromLitersPerMinute UnitConversion
          annotation (Placement(transformation(extent={{132,-150},{152,-130}})));
        Modelica.Blocks.Math.UnitConversions.To_degC Kelvin2Celsius
          annotation (Placement(transformation(extent={{8,-150},{28,-130}})));
        Modelica.Blocks.Continuous.PID PID(
          k=5,
          Ti=0.1,
          Td=0.01) annotation (Placement(transformation(extent={{56,-108},{76,
                  -88}})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold=100)
          annotation (Placement(transformation(extent={{48,-150},{68,-130}})));
        Modelica.Blocks.Logical.Switch switch1
          annotation (Placement(transformation(extent={{94,-150},{114,-130}})));
        Modelica.Blocks.Sources.Constant NoOutput(k=0)
          annotation (Placement(transformation(extent={{56,-190},{76,-170}})));
        Modelica.Blocks.Sources.RealExpression HeatSinkTemp(y=
              thermalImpedanceWithCooling.heatSinkTemperature) annotation (
            Placement(transformation(extent={{-94,-152},{-12,-128}})));
        Cooling.Sources.LiquidPressureBoundary inlet1(
          redeclare package Medium =
              Utilities.LH2CoolProp,
          N=1,
          p_set=100000,
          T_set=293.15) annotation (Placement(transformation(extent={{-60,42},{
                  -40,62}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=UnitConversion.y)
          annotation (Placement(transformation(extent={{-112,56},{-50,82}})));
        Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow
          annotation (Placement(transformation(extent={{16,-44},{36,-24}})));
        Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow1
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={122,-34})));
        Modelica.Blocks.Sources.Constant NoOutput1(k=0)
          annotation (Placement(transformation(extent={{-38,-44},{-18,-24}})));
        Modelica.Blocks.Sources.Constant NoOutput2(k=0)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=180,
              origin={164,-34})));
        inner Cooling.Common.SystemSettings systemSettings
          annotation (Placement(transformation(extent={{182,-196},{202,-176}})));
        Cooling.Sources.LiquidPressureBoundary inlet2(
          redeclare package Medium =
              Utilities.LH2CoolProp,
          p_set=100000,
          T_set=293.15,
          N=1)          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=180,
              origin={150,50})));
        Inverters.ThermalImpedanceWithCooling thermalImpedanceWithCooling(
            redeclare package Medium =
              Utilities.LH2CoolProp)
          annotation (Placement(transformation(extent={{50,0},{70,20}})));
      protected
        ElectrifiedPowertrains.PowerElectronics.Inverters.Interfaces.ThermalPortInverter thermalPortInverter1
          annotation (Placement(transformation(extent={{54,-42},{68,-28}})));
      equation
        connect(Kelvin2Celsius.y,greaterThreshold. u)
          annotation (Line(points={{29,-140},{46,-140}},
                                                       color={0,0,127}));
        connect(greaterThreshold.y,switch1. u2)
          annotation (Line(points={{69,-140},{92,-140}},
                                                    color={255,0,255}));
        connect(switch1.y,UnitConversion. u)
          annotation (Line(points={{115,-140},{130,-140}},
                                                     color={0,0,127}));
        connect(PID.u,Kelvin2Celsius. y) annotation (Line(points={{54,-98},{38,
                -98},{38,-140},{29,-140}},
                               color={0,0,127}));
        connect(PID.y,switch1. u1) annotation (Line(points={{77,-98},{86,-98},{
                86,-132},{92,-132}},
                      color={0,0,127}));
        connect(NoOutput.y,switch1. u3) annotation (Line(points={{77,-180},{88,
                -180},{88,-148},{92,-148}},
                          color={0,0,127}));
        connect(HeatSinkTemp.y,Kelvin2Celsius. u)
          annotation (Line(points={{-7.9,-140},{6,-140}},
                                                       color={0,0,127}));
        connect(pump.port_a,inlet1. port[1])
          annotation (Line(points={{-28,52},{-40,52}}, color={255,127,36}));
        connect(realExpression.y,pump. n_in)
          annotation (Line(points={{-46.9,69},{-18,69},{-18,62}},
                                                              color={0,0,127}));
        connect(prescribedHeatFlow.port,thermalPortInverter1. heatPort_switch)
          annotation (Line(points={{36,-34},{50,-34},{50,-33.6},{60.3,-33.6}}, color={
                191,0,0}));
        connect(prescribedHeatFlow1.port,thermalPortInverter1. heatPort_diode)
          annotation (Line(points={{112,-34},{90,-34},{90,-33.6},{60.3,-33.6}},
              color={191,0,0}));
        connect(prescribedHeatFlow.Q_flow,NoOutput1. y)
          annotation (Line(points={{16,-34},{-17,-34}},color={0,0,127}));
        connect(prescribedHeatFlow1.Q_flow,NoOutput2. y)
          annotation (Line(points={{132,-34},{153,-34}}, color={0,0,127}));
        connect(pump.port_b, thermalImpedanceWithCooling.flowPort_in)
          annotation (Line(points={{-8,52},{52,52},{52,20}}, color={255,127,36}));
        connect(inlet2.port[1], thermalImpedanceWithCooling.flowPort_out)
          annotation (Line(points={{140,50},{66,50},{66,20},{68,20}}, color={
                255,127,36}));
        connect(thermalPortInverter1, thermalImpedanceWithCooling.thermalPortInverter)
          annotation (Line(points={{61,-35},{60,-35},{60,0}}, color={199,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                  -100,-220},{220,100}}), graphics={
              Ellipse(lineColor={0,86,134},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-98,-204},{220,96}}),
              Polygon(
                lineColor={0,86,134},
                fillColor={0,86,134},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-32,74},{-32,74},{218,-34},{218,-34},{-30,-178},{-30,-180},{-32,
                    74}},
                smooth=Smooth.Bezier)}), Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-220},{220,100}})));
      end ThermalInpedanceCooling;
    end Inverter_Tests;

    package Motor
      model Motor_Test
        Motors.ThreeMassTEFC2 threeMassTEFC2_1(redeclare Utilities.Default data)
          annotation (Placement(transformation(extent={{50,-78},{70,-58}})));
        Cooling.Machines.LiquidPumpControlled pump(
          redeclare package Medium = Utilities.LH2CoolProp,
          Nparallel=1,
          m_flowNominal=0.02,
          allowFlowReversal=true,
          checkValve=false,
          m_flowInit=0.02,
          d_init=1044,
          n_set=2000,
          useSpeedInput=true,
          p_aInit=104325,
          p_bInit=164325,
          redeclare Cooling.Machines.Records.Data.PumpData.Default mapData(
              n_nominal=2000))
                          annotation (Placement(transformation(extent={{-32,-56},
                  {-12,-36}})));
        Modelica.Blocks.Math.UnitConversions.To_degC Kelvin2Celsius
          annotation (Placement(transformation(extent={{2,38},{22,58}})));
        Modelica.Blocks.Continuous.PID PID(
          k=5,
          Ti=0.1,
          Td=0.01) annotation (Placement(transformation(extent={{56,74},{76,94}})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold=100)
          annotation (Placement(transformation(extent={{42,38},{62,58}})));
        Modelica.Blocks.Logical.Switch switch1
          annotation (Placement(transformation(extent={{88,38},{108,58}})));
        Modelica.Blocks.Sources.Constant NoOutput(k=0)
          annotation (Placement(transformation(extent={{52,2},{72,22}})));
        Cooling.Sources.LiquidPressureBoundary inlet1(
          redeclare package Medium = Utilities.LH2CoolProp,
          N=1,
          p_set=100000,
          T_set=293.15) annotation (Placement(transformation(extent={{-96,-56},
                  {-76,-36}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=Switch1.y)
          annotation (Placement(transformation(extent={{-136,-32},{-54,-6}})));
        inner Cooling.Common.SystemSettings systemSettings(allowFlowReversal=
              false)
          annotation (Placement(transformation(extent={{140,86},{160,106}})));
        Cooling.Sources.LiquidPressureBoundary inlet2(
          redeclare package Medium =
              Utilities.Media_Utilities.Media.CoolPropMedium,
          p_set=100000,
          T_set=293.15,
          N=1)          annotation (Placement(transformation(extent={{-10,-10},
                  {10,10}},
              rotation=180,
              origin={150,-46})));
        Motors.LinearSquirrelCage
          squirrelCageAIM(useThermalPort=true, redeclare
            ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Data.Linear.Industrial_1kW
            data)
          annotation (Placement(transformation(extent={{44,-114},{64,-94}})));
                    ElectrifiedPowertrains.PowerElectronics.Inverters.Switched.LossyIGBT
                                                               inverter(f_mean=
              10e3)
          annotation (Placement(transformation(extent={{-2,-114},{18,-94}})));
                    ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.SpaceVectorModulation
                                                                   modulationMethod(fs=10e3)
          annotation (Placement(transformation(extent={{-88,-114},{-68,-94}})));
        Modelica.Blocks.Sources.Constant NoOutput1(k=20)
          annotation (Placement(transformation(extent={{-166,-114},{-146,-94}})));
        Modelica.Mechanics.Rotational.Sources.ConstantSpeed speedSource(w_fixed=
              200)
          annotation (Placement(transformation(extent={{170,-114},{150,-94}})));
        Modelica.Mechanics.Rotational.Sensors.PowerSensor powerSensor
          annotation (Placement(transformation(extent={{104,-110},{116,-98}})));
        ElectrifiedPowertrains.ElectricMachines.Common.Blocks.EnergyAnalyser
                                                      machineEfficiencyComputation(
            useBusConnector=true)
          annotation (Placement(transformation(extent={{114,-150},{134,-130}})));
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Torque
          torqueController(redeclare
            ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Data.Torque.Automotive_4kW
            data)
          annotation (Placement(transformation(extent={{-124,-114},{-104,-94}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=0,
              origin={-48,-156})));
        Modelica.Electrical.Analog.Sources.ConstantVoltage idealBattery(V=500)
          annotation (Placement(transformation(
              extent={{-8,-8},{8,8}},
              rotation=270,
              origin={-48,-136})));
        Modelica.Blocks.Sources.RealExpression realExpression1(y=
              threeMassTEFC2_1.statorWindingTemperature1)
          annotation (Placement(transformation(extent={{-102,36},{-20,62}})));
      equation
        connect(Kelvin2Celsius.y,greaterThreshold. u)
          annotation (Line(points={{23,48},{40,48}},   color={0,0,127}));
        connect(greaterThreshold.y,switch1. u2)
          annotation (Line(points={{63,48},{86,48}},color={255,0,255}));
        connect(PID.u,Kelvin2Celsius. y) annotation (Line(points={{54,84},{32,
                84},{32,48},{23,48}},
                               color={0,0,127}));
        connect(PID.y,switch1. u1) annotation (Line(points={{77,84},{80,84},{80,
                56},{86,56}},
                      color={0,0,127}));
        connect(NoOutput.y,switch1. u3) annotation (Line(points={{73,12},{82,12},
                {82,40},{86,40}},
                          color={0,0,127}));
        connect(pump.port_a,inlet1. port[1])
          annotation (Line(points={{-32,-46},{-76,-46}},
                                                       color={255,127,36}));
        connect(realExpression.y,pump. n_in)
          annotation (Line(points={{-49.9,-19},{-22,-19},{-22,-36}},
                                                              color={0,0,127}));
        connect(pump.port_b, threeMassTEFC2_1.flowPort_a) annotation (Line(
              points={{-12,-46},{52,-46},{52,-58}},      color={255,127,36}));
        connect(inlet2.port[1], threeMassTEFC2_1.flowPort_b) annotation (Line(
              points={{140,-46},{68,-46},{68,-58}},      color={255,127,36}));
        connect(threeMassTEFC2_1.thermalPort, squirrelCageAIM.thermalPort)
          annotation (Line(points={{60,-78},{60,-94}},
                                                    color={191,0,0}));
        connect(squirrelCageAIM.plug_p, inverter.plug)
          annotation (Line(points={{44,-104},{18,-104}}, color={0,0,255}));
        connect(inverter.gateSignals[1], modulationMethod.gateSignals[1])
          annotation (Line(points={{-4,-105.667},{-4,-104},{-49,-104},{-49,
                -104.833},{-67,-104.833}}, color={255,0,255}));
        connect(speedSource.flange,powerSensor. flange_b) annotation (Line(points={{150,
                -104},{116,-104}},                                                                          color={0,0,0}));
        connect(squirrelCageAIM.flange, powerSensor.flange_a)
          annotation (Line(points={{64,-104},{104,-104}}, color={0,0,0}));
        connect(modulationMethod.electricDriveBus, machineEfficiencyComputation.electricDriveBus)
          annotation (Line(
            points={{-78,-114},{-78,-176},{8,-176},{8,-150},{124,-150}},
            color={0,86,166},
            thickness=0.5));
        connect(inverter.electricDriveBus, machineEfficiencyComputation.electricDriveBus)
          annotation (Line(
            points={{8,-114},{8,-150},{124,-150}},
            color={0,86,166},
            thickness=0.5));
        connect(squirrelCageAIM.electricDriveBus, machineEfficiencyComputation.electricDriveBus)
          annotation (Line(
            points={{54,-114},{54,-150},{124,-150}},
            color={0,86,166},
            thickness=0.5));
        connect(threeMassTEFC2_1.flange, powerSensor.flange_a) annotation (Line(
              points={{70,-68},{90,-68},{90,-104},{104,-104}}, color={0,0,0}));
        connect(torqueController.electricDriveBus, machineEfficiencyComputation.electricDriveBus)
          annotation (Line(
            points={{-114,-114},{-114,-176},{8,-176},{8,-150},{124,-150}},
            color={0,86,166},
            thickness=0.5));
        connect(NoOutput1.y, torqueController.desiredTorque)
          annotation (Line(points={{-145,-104},{-126,-104}},
                                                           color={0,0,127}));
        connect(torqueController.actuatingVoltages[1], modulationMethod.phaseVoltages[
          1]) annotation (Line(points={{-103,-104.667},{-96,-104.667},{-96,
                -105.333},{-90,-105.333}}, color={0,0,127}));
        connect(ground.p,idealBattery. n)
          annotation (Line(
            points={{-48,-152},{-48,-144}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inverter.pin_p, idealBattery.p) annotation (Line(points={{-2,-98},
                {-48,-98},{-48,-128}},                              color={0,0,
                255}));
        connect(idealBattery.n, inverter.pin_n) annotation (Line(points={{-48,
                -144},{-12,-144},{-12,-110},{-2,-110}}, color={0,0,255}));
        connect(Kelvin2Celsius.u, realExpression1.y) annotation (Line(points={{
                0,48},{-8,48},{-8,49},{-15.9,49}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                  -140,-180},{180,120}}), graphics={
              Ellipse(lineColor={0,86,134},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-140,-180},{178,120}}),
              Polygon(
                lineColor={0,86,134},
                fillColor={0,86,134},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-74,98},{-74,98},{176,-10},{176,-10},{-72,-154},{-72,
                    -156},{-74,98}},
                smooth=Smooth.Bezier)}),   Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-140,-180},{180,120}})));
      end Motor_Test;
    end Motor;

    package Integrated_Test
      model Motor_Inverter
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Motor_Inverter;

      model Integrated
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Integrated;
    end Integrated_Test;

    package Battery_Test
      model BatteryCoolingCircuit
        import Battery;


        Modelica.SIunits.Temperature T_InColdplate=coldPlate.summary.T_a
                                                                        "Fluid Temperature at coldplate inlet";
        Modelica.SIunits.Temperature T_OutColdplate=coldPlate.summary.T_b "Fluid Temperature at coldplate outlet";

        package CoolantMedium = Cooling.Media.Liquids.PropyleneGlycol47;
        Cooling.HeatExchangers.ColdPlates.SingleChannelMeandric coldPlate(
          redeclare package Medium = CoolantMedium,
          allowFlowReversal=true,
          L_x(displayUnit="mm") = 0.24,
          L_y(displayUnit="mm") = 0.28,
          H(displayUnit="mm") = 0.01,
          D_channel(displayUnit="mm") = 0.007,
          initializeCoreTemp=true,
          useDynamicMomentum=false,
          m_flowNominal=0.02,
          d_nominal=1044,
          m_flowInit=0.02,
          initializeCoolantTemp=true,
          Ts_coreInit=fill(
              288.15,
              coldPlate.N_x,
              coldPlate.N_y,
              1),
          N_y=batteryPack.N_y,
          coreInternalHeatConduction=false,
          redeclare record WallMaterial =
              DassaultSystemes.Common.MaterialProperties.SolidMaterials.Aluminium_0degC
              (                                                                                                      lambda_x=236, lambda_y=236),
          redeclare model HT = Cooling.Pipes.HeatTransfer.ConstAlpha ( alpha_set=1000),
          redeclare model DP = Cooling.Pipes.Friction.QuadraticNominalOpPoint,
          dp_nominal=50000,
          N_passPairs=div(batteryPack.N_x, 2),
             T_aCoolantInit=288.15,
          T_bCoolantInit=288.15) annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(transformation(extent={{110,20},{130,40}})));
        Modelica.Blocks.Sources.RealExpression dischargeCurrent(y=-20)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-22,52})));
        Battery.Packs.Discretized.DiscretizedPackCylindric batteryPack(
          N_y=4,
          homogeneousInitT=true,
          N_x=4,
          redeclare Battery.Packs.Discretized.ThermalSeparation.IdealCylindric
            thermalSeparation,
          T_homInit=293.15)
          annotation (Placement(transformation(extent={{40,50},{60,30}})));
        Modelica.Electrical.Analog.Sources.SignalCurrent currentSource
          annotation (Placement(transformation(extent={{52,150},{32,130}})));
        Modelica.Blocks.Logical.Switch switch
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-30,110})));
        Modelica.Blocks.Logical.Hysteresis cycling(
          pre_y_start=true,
          uLow=0.3,
          uHigh=0.6) "True if Discharging" annotation (Placement(transformation(extent={{10,58},{-10,78}})));
        Modelica.Blocks.Sources.RealExpression chargeCurrent(y=20)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={20,88})));
      //     N_x=batteryPack.N_x,
      //     N_y=batteryPack.N_y,
      //     N_cellElements=batteryPack.cell[1, 1].N_verticalElements,
        inner Cooling.Common.SystemSettings systemSettings annotation (Placement(transformation(extent={{100,-20},{120,0}})));
        Battery.Packs.Adapters.FromBus.MeanSOC meanSOC
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={52,68})));
        Cooling.Machines.LiquidPumpControlled pump(
          redeclare package Medium = Utilities.LH2CoolProp,
          Nparallel=1,
          m_flowNominal=0.02,
          allowFlowReversal=true,
          checkValve=false,
          m_flowInit=0.02,
          d_init=1044,
          n_set=2000,
          useSpeedInput=true,
          p_aInit=104325,
          p_bInit=164325,
          redeclare Cooling.Machines.Records.Data.PumpData.Default mapData(n_nominal=2000))
                          annotation (Placement(transformation(extent={{0,-66},
                  {20,-46}})));
        Cooling.Sources.LiquidPressureBoundary inlet1(
          redeclare package Medium = Utilities.LH2CoolProp,
          N=1,
          p_set=100000,
          T_set=293.15) annotation (Placement(transformation(extent={{-32,-66},
                  {-12,-46}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=
              temperatureSensor.y)
          annotation (Placement(transformation(extent={{-88,-46},{-22,-26}})));
        Cooling.Sources.LiquidPressureBoundary inlet2(
          redeclare package Medium = Utilities.LH2CoolProp,
          N=1,
          p_set=100000,
          T_set=293.15) annotation (Placement(transformation(extent={{-10,-10},
                  {10,10}},
              rotation=180,
              origin={88,-56})));
        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor
          temperatureSensor
          annotation (Placement(transformation(extent={{140,-6},{160,14}})));
      protected
        Battery.Common.Interfaces.HousingHeatPort housingHeatPort(
          N_x=batteryPack.N_x,
          N_y=batteryPack.N_y,
          N_z=batteryPack.N_z,
          pinHeatTransfer=true) annotation (Placement(transformation(extent={{20,-6},{80,14}}), iconTransformation(extent={{0,78},{60,
                  98}})));
      equation
        connect(currentSource.p,ground. p)
          annotation (Line(
            points={{52,140},{120,140},{120,140},{120,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSource.n,batteryPack. p)
          annotation (Line(
            points={{32,140},{-60,140},{-60,40},{40,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(batteryPack.n,ground. p)
          annotation (Line(
            points={{60,40},{120,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(cycling.y,switch. u2)
          annotation (Line(
            points={{-11,68},{-30,68},{-30,98}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(switch.u1,dischargeCurrent. y)
          annotation (Line(
            points={{-38,98},{-38,52},{-33,52}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(currentSource.i,switch. y)
          annotation (Line(
            points={{42,128},{42,128},{-30,128},{-30,121}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(chargeCurrent.y,switch. u3)
          annotation (Line(
            points={{9,88},{-22,88},{-22,98}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(batteryPack.packBus, meanSOC.packBus)
          annotation (Line(
            points={{50,50},{52,50},{52,58}},
            color={83,189,255},
            thickness=0.5));
        connect(meanSOC.y, cycling.u) annotation (Line(points={{52,79},{36,79},{36,68},{12,68}}, color={0,127,0}));
        connect(batteryPack.housingHeatPort, housingHeatPort) annotation (Line(points={{50,30},{50,4}}, color={191,0,0}));
        connect(coldPlate.heatPorts, housingHeatPort.bottom)
          annotation (Line(points={{50,-20},{50.0625,-20},{50.0625,4.05}}, color={127,0,0}));
        connect(pump.port_a,inlet1. port[1])
          annotation (Line(points={{0,-56},{-12,-56}}, color={255,127,36}));
        connect(realExpression.y,pump. n_in)
          annotation (Line(points={{-18.7,-36},{10,-36},{10,-46}},
                                                              color={0,0,127}));
        connect(pump.port_b, coldPlate.port_a) annotation (Line(points={{20,-56},
                {44,-56},{44,-40}},
                                color={255,127,36}));
        connect(coldPlate.port_b, inlet2.port[1]) annotation (Line(points={{56,-40},
                {56,-56},{78,-56}},
                                 color={255,127,36}));
        connect(housingHeatPort.top[1, 1], temperatureSensor.port) annotation (
            Line(points={{50.0625,4.05},{96,4.05},{96,4},{140,4}}, color={191,0,
                0}));
        annotation (experiment(StopTime=10000, __Dymola_NumberOfIntervals=5000), __Dymola_experimentSetupOutput,
          Documentation(info="<html>
<p>Demo model of a simple liquid coolant circuit including a coolant pump, a cold plate, a control valve and a coolant reservoir. A generic pipe component is used to model the heat sink. AT t=5000s, the speed of the pump is reduced, as a resilt, the temperature difference goes up.</p>
<p>The thermal load is a battery pack arranged in a 4x4 cell grid. The pack is charged and discharged with the given charge and discharge current, the SOC-swing can be set in the Cycling component. The battery pack model has a model for an insulation material inside the pack between the cells as well as a model for the housing. Heat transfer via pins is neglected.The coldplate is connected to the bottom of the cell housing via the system heat port. Use the command PlotTemperatures from the Command Menu to observe the temperature differences of fluid in the coldplate and the cells. </p>
</html>"),Diagram(coordinateSystem(extent={{-150,-150},{150,150}}, initialScale=0.1)),
          Icon(coordinateSystem(extent={{-100,-100},{100,100}}, initialScale=0.1),
              graphics={
              Ellipse(lineColor={0,86,134},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-98,-102},{100,98}}),
              Polygon(
                lineColor={0,86,134},
                fillColor={0,86,134},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-66,76},{-66,76},{96,16},{102,-10},{-64,-70},{-62,-82},
                    {-66,76}},
                smooth=Smooth.Bezier)}),                                           __Dymola_experimentFlags(Advanced(Define(DAEsolver=true, SparseActivate=true))));
      end BatteryCoolingCircuit;
    end Battery_Test;
  end Tests;
  annotation (uses(
      Modelica(version="3.2.3"),
      ElectrifiedPowertrains(version="1.3"),
      DymolaModels(version="1.0"),
      Cooling(version="1.3.1"),
      DassaultSystemes(version="1.3"),
      ThermalManagementDemos(version="1.1.3"),
      Battery(version="2.1.2")));
end CHEETA_Thermal;
