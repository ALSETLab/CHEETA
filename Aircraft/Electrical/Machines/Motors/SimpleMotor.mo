within CHEETA.Aircraft.Electrical.Machines.Motors;
model SimpleMotor "Simplified AC motor model"

  Modelica.Electrical.Analog.Basic.Inductor inductor(L=X_s)
                                                           "Reactance"
    annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
  parameter Modelica.SIunits.Resistance R=149
    "Effective resistance for hysteresis";
  Modelica.Electrical.Analog.Basic.Resistor resistor(R=R_trs)
    "Effective resistance for transport  ac loss"
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  parameter Modelica.SIunits.Resistance R_trs=1e-6
    "Effective resistance for transport  ac loss";
  parameter Modelica.SIunits.Inductance X_s=0.041 "Reactance";
  Modelica.Electrical.Analog.Basic.Resistor hysteresis(R=R_hyst)
    "Effective resistance for hysteresis ac loss" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,0})));
  parameter Modelica.SIunits.Resistance R_hyst=149
    "Effective resistance for hysteresis ac loss";
  Modelica.Electrical.Analog.Interfaces.PositivePin p1
                "Positive electrical pin"
    annotation (Placement(transformation(extent={{-114,-10},{-94,10}})));
  Modelica.Electrical.Analog.Basic.EMF emf(k=k) "back EMF"
    annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  parameter Modelica.SIunits.ElectricalTorqueConstant k=0.021
    "Transformation coefficient of back EMF";
  parameter Modelica.SIunits.Frequency f = 300 "Machine frequency";
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange1
                                                           "Flange"
    annotation (Placement(transformation(extent={{94,-10},{114,10}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{46,-42},{66,-22}})));

  Modelica.SIunits.CurrentDensity J_c = 1780 "Critical current density";
  Modelica.SIunits.Diameter D_o = 0.32e-3 "SC outer diameter";
  Modelica.SIunits.Diameter d_f = 0.01e-3 "Filament diameter";
  Real n = 114 "Number of filanments";
  Real lambda = 0.15 "SC fill factor";
  Real lambda_e = 0.49 "Effective fill factor";
  Modelica.SIunits.Resistivity rho = 3.65e-4 "Transverse resistivity";
  Modelica.SIunits.Resistivity rho_e = 1.25e-4 "Effective transverse resistivity";
  Modelica.SIunits.Length L = 5e-3 "Twist pitch";


  //Constants
  Real pi= Modelica.Constants.pi;
  Modelica.SIunits.PermeabilityOfVacuum mu_0 = 4*pi*10e-7;
  Modelica.SIunits.PermittivityOfVacuum epsilon_0 = 8.854e-12;
  //Use 6 for a flat tape, 4 for a circular sheath
  Real kappa = 6 "Pre-factor for eddy current power loss";

  //POWER LOSSES
  Real B_p;
  Real P_h "Hysteresis loss";
  Real P_e "Eddy current loss";
  Real P_c "Coupling loss";
  Real P_t "Transport current loss";
  Real B_m = 0.001 "Magnetic flux";



equation
  B_p = 0.4*mu_0*J_c*d_f;
  P_h = 4/3 * J_c*d_f*B_m*f*lambda;
  P_e = pi^2/(kappa*rho_e)*(D_o*B_m*f)^2;
  P_c = 1/(n*rho_e) * (L*B_m*f)^2;
  P_t = mu_0*f/pi;

  connect(inductor.p, resistor.n)
    annotation (Line(points={{-12,0},{-40,0}},   color={0,0,255}));
  connect(inductor.n, hysteresis.p)
    annotation (Line(points={{8,0},{18,0},{18,18},{40,18},{40,10}},
                                                       color={0,0,255}));
  connect(resistor.p, p1)
    annotation (Line(points={{-60,0},{-104,0}},   color={0,0,255}));
  connect(emf.p, hysteresis.p) annotation (Line(points={{72,10},{72,18},{40,18},
          {40,10}}, color={0,0,255}));
  connect(emf.flange, flange1)
    annotation (Line(points={{82,0},{104,0}}, color={0,0,0}));
  connect(hysteresis.n, ground.p) annotation (Line(points={{40,-10},{40,-16},{
          56,-16},{56,-22}}, color={0,0,255}));
  connect(emf.n, ground.p) annotation (Line(points={{72,-10},{72,-16},{56,-16},
          {56,-22}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
        Text(
          extent={{-62,28},{74,-20}},
          lineColor={28,108,200},
          textString="motor")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SimpleMotor;
