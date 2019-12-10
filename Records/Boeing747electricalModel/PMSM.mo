within CHEETA.Records.Boeing747electricalModel;
package PMSM
  record PMSM_2kVA "2 KVA PMSM"
    extends CHEETA.Records.Boeing747electricalModel.PMSM.PSM;
    extends SM_ReluctanceRotorData(Lmd=8.5e-3, Lmq=8.5e-3);
    import Modelica.Constants.pi;
    parameter Modelica.SIunits.Voltage VsOpenCircuit=127
      "Open circuit RMS voltage per phase @ fsNominal";
    parameter Modelica.Electrical.Machines.Losses.PermanentMagnetLossParameters
      permanentMagnetLossParameters(
      PRef=0,
      IRef=100,
      wRef=2*pi*fsNominal/p) "Permanent magnet loss parameter record"
      annotation (Dialog(tab="Losses"));
    annotation (
      defaultComponentName="smpmData",
      defaultComponentPrefixes="parameter",
      Documentation(info="<html>
<p>Basic parameters of synchronous induction machines with permanent magnet are predefined with default values.</p>
</html>"));
  end PMSM_2kVA;

  record SM_ReluctanceRotorData
    "Common parameters for synchronous induction machines with reluctance rotor"
    extends CHEETA.Records.Boeing747electricalModel.PMSM.InductionMachineData(
        Lssigma=0.1/(2*pi*fsNominal));
    import Modelica.Constants.pi;
    parameter Modelica.SIunits.Inductance Lmd=8.5e-3
      "Stator main field inductance per phase in d-axis"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    parameter Modelica.SIunits.Inductance Lmq=8.5e-3
      "Stator main field inductance per phase in q-axis"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    parameter Boolean useDamperCage=true "Enable / disable damper cage"
      annotation (Evaluate=true,Dialog(tab=
            "Nominal resistances and inductances", group="DamperCage"));
    parameter Modelica.SIunits.Inductance Lrsigmad=0.05/(2*pi*fsNominal)
      "Damper stray inductance in d-axis" annotation (Dialog(
        tab="Nominal resistances and inductances",
        group="DamperCage",
        enable=useDamperCage));
    parameter Modelica.SIunits.Inductance Lrsigmaq=Lrsigmad
      "Damper stray inductance in q-axis" annotation (Dialog(
        tab="Nominal resistances and inductances",
        group="DamperCage",
        enable=useDamperCage));
    parameter Modelica.SIunits.Resistance Rrd=0.04
      "Damper resistance in d-axis at TRef" annotation (Dialog(
        tab="Nominal resistances and inductances",
        group="DamperCage",
        enable=useDamperCage));
    parameter Modelica.SIunits.Resistance Rrq=Rrd
      "Damper resistance in q-axis at TRef" annotation (Dialog(
        tab="Nominal resistances and inductances",
        group="DamperCage",
        enable=useDamperCage));
    parameter Modelica.SIunits.Temperature TrRef=293.15
      "Reference temperature of damper resistances in d- and q-axis"
      annotation (Dialog(
        tab="Nominal resistances and inductances",
        group="DamperCage",
        enable=useDamperCage));
    parameter Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
      alpha20r=0 "Temperature coefficient of damper resistances in d- and q-axis"
      annotation (Dialog(
        tab="Nominal resistances and inductances",
        group="DamperCage",
        enable=useDamperCage));
    annotation (
      defaultComponentName="smrData",
      defaultComponentPrefixes="parameter",
      Documentation(info="<html>
<p>Basic parameters of synchronous induction machines with reluctance rotor are predefined with default values.</p>
</html>"));
  end SM_ReluctanceRotorData;

  record InductionMachineData
    "Common parameters for induction machines"
    extends Modelica.Icons.Record;
    import Modelica.Constants.pi;
    final parameter Integer m=3 "Number of phases";
    parameter Modelica.SIunits.Inertia Jr=0.089 "Rotor's moment of inertia";
    parameter Modelica.SIunits.Inertia Js=Jr "Stator's moment of inertia";
    parameter Integer p(min=1) = 4 "Number of pole pairs (Integer)";
    parameter Modelica.SIunits.Frequency fsNominal=60 "Nominal frequency";
    parameter Modelica.SIunits.Resistance Rs=0.2
      "Stator resistance per phase at TRef"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    parameter Modelica.SIunits.Temperature TsRef=293.15
      "Reference temperature of stator resistance"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    parameter Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
      alpha20s=0 "Temperature coefficient of stator resistance at 20 degC"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    parameter Real effectiveStatorTurns=1 "Effective number of stator turns";
    parameter Modelica.SIunits.Inductance Lszero=Lssigma
      "Stator zero sequence inductance"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    parameter Modelica.SIunits.Inductance Lssigma=3*(1 - sqrt(1 - 0.0667))/
        (2*pi*fsNominal) "Stator stray inductance per phase"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    parameter Modelica.Electrical.Machines.Losses.FrictionParameters frictionParameters(PRef=0,
        wRef=2*pi*fsNominal/p) "Friction loss parameter record"
      annotation (Dialog(tab="Losses"));
    parameter Modelica.Electrical.Machines.Losses.CoreParameters statorCoreParameters(
      final m=m,
      PRef=0,
      VRef=100,
      wRef=2*pi*fsNominal)
      "Stator core loss parameter record; all parameters refer to stator side"
      annotation (Dialog(tab="Losses"));
    parameter Modelica.Electrical.Machines.Losses.StrayLoadParameters strayLoadParameters(
      PRef=0,
      IRef=100,
      wRef=2*pi*fsNominal/p) "Stray load losses" annotation (Dialog(tab="Losses"));
    annotation (
      defaultComponentName="inductionMachineData",
      defaultComponentPrefixes="parameter",
      Documentation(info="<html>
<p>Basic parameters of induction machines are predefined with default values.</p>
</html>"));
  end InductionMachineData;

  record PSM
    extends Modelica.Icons.Record
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end PSM;
end PMSM;
