within CHEETA.Aircraft.Electrical.PowerElectronics.Examples.Records;
record VSI_20kW "20 kW VSI motor"
  extends
    ElectrifiedPowertrains.ElectricMachines.PSM.ElectroMechanical.Records.Base.Linear(
    terminalConnection=DymolaModels.Electrical.MultiPhase.Choices.Wiring.Y,
    fs_nom=400,
    p=4,
    Jr=0.00378,
    Js=10*Jr,
    Rs=7.3,
    Lmd=18e-3,
    Lmq=18e-3,
    Ls_sigma=16e-3,
    u_s_openCircuit=410/sqrt(3)*(750/1000),
    Ts_operational=20 + 273.15,
    Ts_ref=20 + 273.17,
    useDamperCage=false,
    alpha20s=3.93e-3,
    Ls_zero=Ls_sigma,
    permanentMagnetParameters=Modelica.Electrical.Machines.Losses.PermanentMagnetLossParameters(
        PRef=4,
        IRef=i_s_nom,
        wRef=2*fs_nom*Modelica.Constants.pi/p),
    statorCoreParameters=Modelica.Electrical.Machines.Losses.CoreParameters(
        m=3,
        PRef=25,
        VRef=u_s_nom,
        wRef=2*Modelica.Constants.pi*fs_nom),
    strayLoadParameters=Modelica.Electrical.Machines.Losses.StrayLoadParameters(
        PRef=15,
        IRef=i_s_nom,
        wRef=2*fs_nom*Modelica.Constants.pi/p,
        power_w=1),
    frictionParameters=Modelica.Electrical.Machines.Losses.FrictionParameters(PRef=12, wRef=2*fs_nom*Modelica.Constants.pi/p),
    i_s_nom=15,
    u_s_nom=320,
    tau_nom=17);
  annotation (defaultComponentName="data", Documentation(info="<html>
<p>Parameters are based on the data sheet values of the PE1R 90 L8 from VEM.</p>
</html>"));
end VSI_20kW;
