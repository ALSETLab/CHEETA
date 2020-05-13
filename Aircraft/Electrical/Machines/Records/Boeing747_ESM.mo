within CHEETA.Aircraft.Electrical.Machines.Records;
record Boeing747_ESM "300 kVA machine for Boeing 747"
  import Modelica.Constants.pi;
  extends DymolaModels.Icons.Data.Record;
  extends ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Records.Base.Linear(
    terminalConnection=DymolaModels.Electrical.MultiPhase.Choices.Wiring.Y,
    p=2,
    Js=0.02,
    Jr=0.29,
    fs_nom=50,
    u_s_openCircuit=100,
    Ie_openCircuit=10,
    Rs=0.0179,
    Ts_ref=20 + 273.15,
    alpha20s=0,
    Lmd=4*164.571e-6,
    Lmq=4*125.772e-6,
    Ls_sigma=0.1/(2*Modelica.Constants.pi*fs_nom),
    Tr_ref=20 + 273.15,
    alpha20r=0,
    useDamperCage=false,
    Lr_sigma_d=0.000134398,
    Lr_sigma_q=6.55344e-5,
    Re=1.08824,
    Te_ref=20 + 273.15,
    alpha20e=0,
    sigma_e=0.0513513,
    Ts_operational=Ts_ref,
    Tr_operational=Tr_ref,
    Te_operational=Te_ref,
    Rrd=0.00823727,
    Rrq=0.0376017,
    brushParameters=Modelica.Electrical.Machines.Losses.BrushParameters(V=0, ILinear=0.01),
    frictionParameters=Modelica.Electrical.Machines.Losses.FrictionParameters(PRef=0, wRef=100),
    statorCoreParameters=Modelica.Electrical.Machines.Losses.CoreParameters(
        m=3,
        PRef=0,
        VRef=100,
        wRef=100),
    strayLoadParameters=Modelica.Electrical.Machines.Losses.StrayLoadParameters(
        PRef=0,
        IRef=100,
        wRef=100),
    tau_nom=-196.7,
    w_nom=2*Modelica.Constants.pi,
    u_s_nom=230,
    i_s_nom=500/sqrt(3),
    psi_nom=Ie_openCircuit*Lmd*turnsRatio,
    P_nom=3e5);

  annotation (Documentation(info="<html>
<p>Data from Example Modelica.Electrical.Machines.Examples.AsynchronousInductionMachines.AIMC_withLosses</p>
</html>"), defaultComponentName="data");
end Boeing747_ESM;
