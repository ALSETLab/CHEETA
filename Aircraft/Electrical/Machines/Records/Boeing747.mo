within CHEETA.Aircraft.Electrical.Machines.Records;
record Boeing747 "300 kVA machine for Boeing 747"
  import Modelica.Constants.pi;
  extends DymolaModels.Icons.Data.Record;
  extends
    ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Base.Linear(
    terminalConnection=DymolaModels.Electrical.MultiPhase.Choices.Wiring.D,
    p=4,
    Jr=0.089,
    Js=Jr,
    fs_nom=50,
    Rs=0.0179,
    Rr=0.04,
    Ts_ref=Modelica.SIunits.Conversions.from_degC(20),
    Tr_ref=Modelica.SIunits.Conversions.from_degC(20),
    Ts_operational=Modelica.SIunits.Conversions.from_degC(20),
    Tr_operational=Modelica.SIunits.Conversions.from_degC(20),
    alpha20s=0,
    alpha20r=0,
    Ls_sigma=0.15*(u_s_nom/i_s_nom)/w_nom,
    Ls_zero=Ls_sigma,
    Lr_sigma=3*(1 - sqrt(1 - 0.0667))/(2*pi*fs_nom),
    Lm=(2-0.15),
    statorCoreParameters=Modelica.Electrical.Machines.Losses.CoreParameters(
        m=3,
        PRef=0,
        VRef=u_s_nom,
        wRef=2*pi*fs_nom),
    strayLoadParameters=Modelica.Electrical.Machines.Losses.StrayLoadParameters(
        PRef=0,
        IRef=i_s_nom,
        wRef=w_nom),
    frictionParameters=Modelica.Electrical.Machines.Losses.FrictionParameters(PRef=0, wRef=w_nom),
    tau_nom=6.4,
    psi_nom=0.0203,
    w_nom=Modelica.SIunits.Conversions.from_rpm(10e3/tau_nom*p/2),
    i_s_nom= 300e3/(200*3) "motor phase current (RMS)",
    u_s_nom=200/sqrt(2) "line to line voltage (RMS)");

  annotation (Documentation(info="<html>
<p>Data from Example Modelica.Electrical.Machines.Examples.AsynchronousInductionMachines.AIMC_withLosses</p>
</html>"), defaultComponentName="data");
end Boeing747;
