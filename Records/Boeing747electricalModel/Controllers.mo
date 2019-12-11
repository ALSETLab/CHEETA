within CHEETA.Records.Boeing747electricalModel;
package Controllers

  record TypeI_AVR
    extends CHEETA.Records.Boeing747electricalModel.Base.AVR;
    extends Modelica.Icons.Record;
    parameter Real T_R = 2e-3 "Rate Filter Time Constant";
    parameter Real T_C = 0.001 "TGR Time Constant";
    parameter Real T_B = 0.001 "TGR Time Constant";
    parameter Real K_A = 50 "Regulator Gain";
    parameter Real T_A = 0.001 "Regulator Time Constant";
    parameter Real K_E = 1 "Exciter Gain";
    parameter Real T_E = 0.001 "Exciter Time Constant";
    parameter Real K_F = 0.001 "Rate Feedback Gain";
    parameter Real T_F = 0.1
                            "Rate Feedback Time Constant";
    parameter Real Vmax = 2 "Regulator Maximum Output";
    parameter Real Vmin = -2 "Regulator Minimum Output";
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end TypeI_AVR;
end Controllers;
