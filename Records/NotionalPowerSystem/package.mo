within CHEETA.Records;
package NotionalPowerSystem

  record SM_PermanentMagnetData
    "Common parameters for synchronous induction machines with permanent magnet"
    extends Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_ReluctanceRotorData(Lmd=0.3/(2*pi*fsNominal), Lmq=0.3/(2*pi*
          fsNominal));
    import Modelica.Constants.pi;
    parameter Modelica.SIunits.Voltage VsOpenCircuit=112.3
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
  end SM_PermanentMagnetData;

  record AIM_SquirrelCageData
    "Common parameters for asynchronous induction machines with squirrel cage"
    extends Modelica.Electrical.Machines.Utilities.ParameterRecords.InductionMachineData;
    import Modelica.Constants.pi;
    parameter Modelica.SIunits.Inductance Lm=3*sqrt(1 - 0.0667)/(2*pi*
        fsNominal) "Stator main field inductance per phase"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    parameter Modelica.SIunits.Inductance Lrsigma=3*(1 - sqrt(1 - 0.0667))/
        (2*pi*fsNominal)
      "Rotor stray inductance per phase (equivalent three phase winding)"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    parameter Modelica.SIunits.Resistance Rr=0.04
      "Rotor resistance per phase (equivalent three phase winding) at TRef"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    parameter Modelica.SIunits.Temperature TrRef=293.15
      "Reference temperature of rotor resistance"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    parameter Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20 alpha20r=0 "Temperature coefficient of rotor resistance at 20 degC"
      annotation (Dialog(tab="Nominal resistances and inductances"));
    annotation (
      defaultComponentName="aimcData",
      defaultComponentPrefixes="parameter",
      Documentation(info="<html>
<p>Basic parameters of asynchronous induction machines with squirrel cage are predefined with default values.</p>
</html>"));
  end AIM_SquirrelCageData;
end NotionalPowerSystem;
