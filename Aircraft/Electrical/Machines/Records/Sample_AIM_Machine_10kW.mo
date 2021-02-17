within CHEETA.Aircraft.Electrical.Machines.Records;
record Sample_AIM_Machine_10kW "1kW machine"
  import Modelica.Constants.pi;
  extends DymolaModels.Icons.Data.Record;
  extends
    ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Base.Linear(
    terminalConnection=DymolaModels.Electrical.MultiPhase.Choices.Wiring.D,
    p=12,
    Jr=6.44e-4,
    Js=10*Jr,
    fs_nom=50,
    Rs=0.8,
    Rr=0.04,
    Ts_ref=Modelica.Units.Conversions.from_degC(20),
    Tr_ref=Modelica.Units.Conversions.from_degC(20),
    Ts_operational=Modelica.Units.Conversions.from_degC(20),
    Tr_operational=Modelica.Units.Conversions.from_degC(20),
    alpha20s=0,
    alpha20r=0,
    Ls_sigma=20e-6,
    Ls_zero=Ls_sigma,
    Lr_sigma=3*(1 - sqrt(1 - 0.0667))/(2*pi*fs_nom),
    Lm=3*sqrt(1 - 0.0667)/(2*pi*fs_nom),
    statorCoreParameters=Modelica.Electrical.Machines.Losses.CoreParameters(
        m=3,
        PRef=0,
        VRef=u_s_nom,
        wRef=2*pi*fs_nom),
    strayLoadParameters=Modelica.Electrical.Machines.Losses.StrayLoadParameters(
        PRef=0,
        IRef=i_s_nom,
        wRef=w_nom),
    frictionParameters=Modelica.Electrical.Machines.Losses.FrictionParameters(
        PRef=0, wRef=w_nom),
    tau_nom=6.4,
    psi_nom=0.0203,
    w_nom=Modelica.Units.Conversions.from_rpm(10e3/tau_nom*p/2),
    i_s_nom=10e3/u_s_nom*sqrt(2)/3 "motor phase current (RMS)",
    u_s_nom=w_nom*0.0203/sqrt(2) "line to line voltage (RMS)");

  annotation (Documentation(info="<html>
<p>Data from Example Modelica.Electrical.Machines.Examples.AsynchronousInductionMachines.AIMC_withLosses</p>
</html>"), defaultComponentName="data");
end Sample_AIM_Machine_10kW;
