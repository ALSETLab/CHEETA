within CHEETA.Aircraft.Electrical.Machines.Records;
package CHEETA_Records
  extends Modelica.Icons.RecordsPackage;
  record CHEETA_1MW "1MW machine"
  import Modelica.Constants.pi;
    extends DymolaModels.Icons.Data.Record;
    extends
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Base.Linear(
  terminalConnection=DymolaModels.Electrical.MultiPhase.Choices.Wiring.D,
      p=8,
      Jr=0.29,
      Js=10*Jr,
      fs_nom=60,
      Rs=0.03,
      Rr=0.04,
      Ts_ref=Modelica.SIunits.Conversions.from_degC(20),
      Tr_ref=Modelica.SIunits.Conversions.from_degC(20),
      Ts_operational=Modelica.SIunits.Conversions.from_degC(20),
      Tr_operational=Modelica.SIunits.Conversions.from_degC(20),
      alpha20s=0,
      alpha20r=0,
      Ls_sigma=21e-6,
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
      frictionParameters=Modelica.Electrical.Machines.Losses.FrictionParameters(PRef=0, wRef=w_nom),
      tau_nom=6280.4,
      psi_nom=2.62,
      w_nom=Modelica.SIunits.Conversions.from_rpm(7000),
      i_s_nom=40111.16 "motor phase current (RMS)",
      u_s_nom=1635.62 "line to line voltage (RMS)");
                 //3*sqrt(1 - 0.0667)/(2*pi*fs_nom),
                    //0.15*(u_s_nom/i_s_nom)/w_nom,

    annotation (Documentation(info="<html>
<p>Data from Example Modelica.Electrical.Machines.Examples.AsynchronousInductionMachines.AIMC_withLosses</p>
</html>"),   defaultComponentName="data");
  end CHEETA_1MW;

  record CHEETA_1MW_controller "1 MW machine"
    extends
      ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Speed(
      redeclare final
        CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_1MW
        machineData,
      simpleCurrentTuning=true,
      speedTuningWithSO=true,
      discretize=false,
      i_s_max=4000);
    annotation (Documentation(info="<html>
<p>Controller parameters for the data set <a href=\"ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Data.Linear.MSL_18p5kW\">ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Data.Linear.MSL_18p5kW</a></p>
</html>"));
  end CHEETA_1MW_controller;

  record CHEETA_5MW_ESM "5MW machine for CHEETA"
    import Modelica.Constants.pi;
    extends
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Records.Base.Linear(
      terminalConnection=DymolaModels.Electrical.MultiPhase.Choices.Wiring.Y,
      p=8,
      Js=1.29,
      Jr=1.29,
      fs_nom=400,
      u_s_openCircuit=600,
      Ie_openCircuit=550,
      Rs=0.15,
      Ts_ref=20 + 273.15,
      alpha20s=0,
      Lmd=0.75/(2*Modelica.Constants.pi*fs_nom) - Ls_sigma,
      Lmq=Lmd,
      Ls_sigma=0.1/(2*Modelica.Constants.pi*fs_nom),
      Tr_ref=20 + 273.15,
      alpha20r=0,
      useDamperCage=false,
      Lr_sigma_d=0.05/(2*Modelica.Constants.pi*fs_nom),
      Lr_sigma_q=Lr_sigma_d,
      Re=4.05,
      Te_ref=20 + 273.15,
      alpha20e=0,
      sigma_e=2.5/100,
      Ts_operational=Ts_ref,
      Tr_operational=Tr_ref,
      Te_operational=Te_ref,
      Rrd=0.04,
      Rrq=Rrd,
      brushParameters=Modelica.Electrical.Machines.Losses.BrushParameters(V=0, ILinear=0.01),
      frictionParameters=Modelica.Electrical.Machines.Losses.FrictionParameters(PRef=0, wRef=Modelica.SIunits.Conversions.from_rpm(7000)),
      statorCoreParameters=Modelica.Electrical.Machines.Losses.CoreParameters(
          m=3,
          PRef=0,
          VRef=200,
          wRef=Modelica.SIunits.Conversions.from_rpm(7000)),
      strayLoadParameters=Modelica.Electrical.Machines.Losses.StrayLoadParameters(
          PRef=0,
          IRef=100,
          wRef=Modelica.SIunits.Conversions.from_rpm(7000)),
      tau_nom=6280.93,
      w_nom=Modelica.SIunits.Conversions.from_rpm(7000),
      u_s_nom=635.62,
      i_s_nom=5500,
      psi_nom=Ie_openCircuit*Lmd*turnsRatio,
      P_nom=5e6);       //(0.1*(u_s_nom/i_s_nom))/(2*pi*fs_nom),
               //(0.1*(u_s_nom/i_s_nom))/(2*pi*fs_nom),

    annotation (Documentation(info="<html>
<p>Data from Example Modelica.Electrical.Machines.Examples.AsynchronousInductionMachines.AIMC_withLosses</p>
</html>"),   defaultComponentName="data");
  end CHEETA_5MW_ESM;

  record CHEETA_555MVA_ESM "555MVA machine for CHEETA"
    import Modelica.Constants.pi;
    final parameter Modelica.SIunits.Impedance ZReference=u_s_nom/i_s_nom
      "Reference impedance";
    extends DymolaModels.Icons.Data.Record;
    parameter Real x0(start=0.1)
      "Stator stray inductance per phase (approximately zero impedance) [pu]";
    parameter Real xd(start=1.81)
      "Synchronous reactance per phase, d-axis [pu]";
    parameter Real xq(start=1.76)
      "Synchronous reactance per phase, q-axis [pu]";
    parameter Real xdTransient(start=0.3)
      "Transient reactance per phase, d-axis [pu]";
    parameter Real xdSubtransient(start=0.23)
      "Subtransient reactance per phase, d-axis [pu]";
    parameter Real xqSubtransient(start=0.25)
      "Subtransient reactance per phase, q-axis [pu]";
    parameter Modelica.SIunits.Time Ta(start=0.014171268)
      "Armature time constant";
    parameter Modelica.SIunits.Time Td0Transient(start=8)
      "Open circuit field time constant Td0'";
    parameter Modelica.SIunits.Time Td0Subtransient(start=0.03)
      "Open circuit subtransient time constant Td0'', d-axis";
    parameter Modelica.SIunits.Time Tq0Subtransient(start=0.07)
      "Open circuit subtransient time constant Tq0'', q-axis";
    final parameter Real xmd=xd - x0
      "Main field reactance per phase, d-axis [pu]";
    final parameter Real xmq=xq - x0
      "Main field reactance per phase, q-axis [pu]";
    final parameter Real xe=xmd^2/(xd - xdTransient)
      "Excitation reactance [pu]";
    final parameter Real xrd=xmd^2/(xdTransient - xdSubtransient)*(1 - (xmd/
        xe))^2 + xmd^2/xe "Damper reactance per phase, d-axis [pu]";
    final parameter Real xrq=xmq^2/(xq - xqSubtransient)
      "Damper reactance per phase, d-axis [pu]";
    final parameter Real rs=2/(1/xdSubtransient + 1/xqSubtransient)/(omega*Ta)
      "Stator resistance per phase at specification temperature [pu]";
    final parameter Real rrd=(xrd - xmd^2/xe)/(omega*Td0Subtransient)
      "Damper resistance per phase at specification temperature, d-axis [pu]";
    final parameter Real rrq=xrq/(omega*Tq0Subtransient)
      "Damper resistance per phase at specification temperature, q-axis [pu]";
    final parameter Real re=xe/(omega*Td0Transient)
      "Excitation resistance per phase at specification temperature [pu]";
    final parameter Modelica.SIunits.AngularVelocity omega=2*pi*fs_nom
      "Nominal angular frequency";
    extends
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Records.Base.Linear(
      terminalConnection=DymolaModels.Electrical.MultiPhase.Choices.Wiring.Y,
      p=8,
      Js=0.02,
      Jr=0.29,
      fs_nom=60,
      u_s_openCircuit=1000,
      Ie_openCircuit=4166.67,
      Rs=0.00179,
      Ts_ref=20 + 273.15,
      alpha20s=0,
      Lmd=xmd*ZReference/omega,
      Lmq=xmq*ZReference/omega,
      Ls_sigma=x0*ZReference/omega,
      Tr_ref=20 + 273.15,
      alpha20r=0,
      useDamperCage=false,
      Lr_sigma_d=(xrd - xmd)*ZReference/omega,
      Lr_sigma_q=(xrq - xmq)*ZReference/omega,
      Re=0.00138888894,
      Te_ref=20 + 273.15,
      alpha20e=0,
      sigma_e=1 - xmd/xe,
      Ts_operational=Ts_ref,
      Tr_operational=Tr_ref,
      Te_operational=Te_ref,
      Rrd=0.02,
      Rrq=0.02,
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
      u_s_nom=650,
      i_s_nom=7692.31,
      psi_nom=Ie_openCircuit*Lmd*turnsRatio,
      P_nom=5e6);

    annotation (Documentation(info="<html>
<p>Data from Example Modelica.Electrical.Machines.Examples.AsynchronousInductionMachines.AIMC_withLosses</p>
</html>"),   defaultComponentName="data");
  end CHEETA_555MVA_ESM;

  record CHEETA_5MW_ESM_Controller "5MW machine for CHEETA"
    extends ElectrifiedPowertrains.ElectricMachines.ESM.Controllers.Records.Base.Speed(
      redeclare CHEETA.Aircraft.Electrical.Machines.Records.CHEETA_Records.CHEETA_5MW_ESM machineData,
      Ve_max=2000,
      i_s_max=5000,
      Ie_max=6000,
      tau_max=10000);
  end CHEETA_5MW_ESM_Controller;

  record CHEETA_5MW_ESM2 "5MW machine for CHEETA 2"
    import Modelica.Constants.pi;
    extends
      ElectrifiedPowertrains.ElectricMachines.ESM.ElectroMechanical.Records.Base.Linear(
       terminalConnection=DymolaModels.Electrical.MultiPhase.Choices.Wiring.Y,
      p=8,
      Js=Jr "Estimated value",
      Jr=0.29,
      fs_nom=50,
      u_s_openCircuit=2*Modelica.Constants.pi*fs_nom*Lmd*Ie_openCircuit/(sqrt(2)*0.0404),
      Ie_openCircuit=1000,
      Rs=0.894,
      Ts_ref=20 + 273.15,
      alpha20s=0,
      Lmd= (1*(u_s_nom/i_s_nom))/(2*pi*fs_nom),
      Lmq= Lmd,
      Ls_sigma=(0.1*(u_s_nom/i_s_nom))/(2*pi*fs_nom),
      Tr_ref=20 + 273.15,
      alpha20r=0,
      useDamperCage=false,
      Lr_sigma_d=(0.1*(u_s_nom/i_s_nom))/(2*Modelica.Constants.pi*fs_nom),
      Lr_sigma_q=Lr_sigma_d,
      Re=35.25,
      Te_ref=20 + 273.15,
      alpha20e=0,
      sigma_e=0.25,
      Ts_operational=Ts_ref,
      Tr_operational=Tr_ref,
      Te_operational=Te_ref,
      Rrd=0.010139999,
      Rrq=Rrd,
      brushParameters=Modelica.Electrical.Machines.Losses.BrushParameters(V=205.25, ILinear=40),
      frictionParameters=Modelica.Electrical.Machines.Losses.FrictionParameters(
          PRef=2500,
          wRef=2*Modelica.Constants.pi*fs_nom/p,
          power_w=2),
      statorCoreParameters=Modelica.Electrical.Machines.Losses.CoreParameters(
          m=3,
          PRef=0,
          VRef=650,
          wRef=2*Modelica.Constants.pi*fs_nom),
      strayLoadParameters=Modelica.Electrical.Machines.Losses.StrayLoadParameters(
          PRef=0,
          IRef=200,
          wRef=2*Modelica.Constants.pi*fs_nom/p,
          power_w=4),
      tau_nom=50,
      i_s_nom=2564,
      u_s_nom=650,
      w_nom=Modelica.SIunits.Conversions.from_rpm(7000),
      psi_nom=Ie_openCircuit*Lmd*turnsRatio,
      P_nom=5e6);       //(0.1*(u_s_nom/i_s_nom))/(2*pi*fs_nom),
               //(0.1*(u_s_nom/i_s_nom))/(2*pi*fs_nom),

    annotation (Documentation(info="<html>
<p>Data from Example Modelica.Electrical.Machines.Examples.AsynchronousInductionMachines.AIMC_withLosses</p>
</html>"),   defaultComponentName="data");
  end CHEETA_5MW_ESM2;
end CHEETA_Records;
