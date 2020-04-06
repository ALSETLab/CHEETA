within CHEETA.Aircraft.Electrical.Machines.Examples;
package Boeing747

extends Modelica.Icons.ExamplesPackage;
  ElectrifiedPowertrains.ElectricDrives.AIM.ElectroMechanical.TorqueFOC electricDrive(
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.PWM.NoModulation
      modulationMethod(k3=1/6),
    redeclare
      ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.LinearSquirrelCage
      machine(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Data.Linear.MSL_18p5kW
        data),
    redeclare ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Torque
      controller(redeclare
        ElectrifiedPowertrains.ElectricMachines.AIM.Controllers.Records.Base.Torque
        data(
        redeclare
          ElectrifiedPowertrains.ElectricMachines.AIM.ElectroMechanical.Records.Data.Linear.MSL_18p5kW
          machineData,
        torqueTuningWithMO=true,
        fluxTuningWithMO=true,
        i_s_max=50,
        T_x=0.02,
        T_y=0.02)),
    redeclare
      ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.ConstantEfficiency
      inverter(redeclare
        ElectrifiedPowertrains.PowerElectronics.Inverters.Averaged.Electrical.Records.Data.ConstantEfficiency.Constant98percent
        data),
    useThermalPort=true)
                    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
end Boeing747;
