within CHEETA.Aircraft.Electrical.PowerElectronics.Examples.Records;
record VSI_PowerElectronics "VSI example inverter values"
  extends DymolaModels.Icons.Data.Record;
  extends
    ElectrifiedPowertrains.PowerElectronics.Inverters.Switched.Electrical.Records.Base.Ideal(
      R_on_IGBT = 1e-3,
      G_off_IGBT = 1e-6,
      R_on_diode = 1e-6,
      G_off_diode = 1e-6,
      V_knee = 0);

end VSI_PowerElectronics;
