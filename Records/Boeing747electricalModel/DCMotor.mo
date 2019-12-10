within CHEETA.Records.Boeing747electricalModel;
package DCMotor
  package FuelPumps
    record FuelPump_1
      extends AircraftPowerSystem.Records.Base.DC_Motor_Data(
        Ra=0.35,
        La=0.0001,
        Rf=240,
        Lf=12,
        Laf=1,
        J=0.0005);
    end FuelPump_1;

    record FuelPump_2
      extends AircraftPowerSystem.Records.Base.DC_Motor_Data(
        Ra=0.35,
        La=0.0001,
        Rf=240,
        Lf=12,
        Laf=1,
        J=0.0005);
    end FuelPump_2;
  end FuelPumps;
end DCMotor;
