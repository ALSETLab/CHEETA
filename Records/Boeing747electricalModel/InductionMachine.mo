within CHEETA.Records.Boeing747electricalModel;
package InductionMachine
  record IM_30KVA "30 KVA Induction Motor"
    extends AircraftPowerSystem.Records.Base.IM(
      p=8,
      fsNominal=400,
      J=0.01,
      Rs=0.2761,
      Lss=0.0002191,
      Lm=0.07614,
      Lr=0.0002191,
      Rr=0.16);
  end IM_30KVA;
end InductionMachine;
