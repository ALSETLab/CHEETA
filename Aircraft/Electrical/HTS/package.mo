within CHEETA.Aircraft.Electrical;
package HTS "Models for the high temperature superconductor"

  model HTS_firstorder
    "Model for a transmission Line based on the pi-equivalent circuit"
    outer CHEETA.Aircraft.Electrical.SystemBase SysData;
    import Modelica.ComplexMath.conj;
    import Modelica.ComplexMath.real;
    import Modelica.ComplexMath.imag;
    import Modelica.ComplexMath.j;
    parameter Modelica.SIunits.PerUnit R "Resistance (pu)"
      annotation (Dialog(group="Line parameters"));
    parameter Modelica.SIunits.PerUnit X "Reactance (pu)"
      annotation (Dialog(group="Line parameters"));
    parameter Modelica.SIunits.PerUnit G "Shunt half conductance (pu)"
      annotation (Dialog(group="Line parameters"));
    parameter Modelica.SIunits.PerUnit B "Shunt half susceptance (pu)"
      annotation (Dialog(group="Line parameters"));
    parameter Real S_b=SysData.S_b
      "System base power (MVA)"
      annotation (Dialog(group="Line parameters", enable=false));
    parameter Modelica.SIunits.Time t1=Modelica.Constants.inf
      annotation (Dialog(group="Perturbation parameters"));
    parameter Modelica.SIunits.Time t2=Modelica.Constants.inf
      annotation (Dialog(group="Perturbation parameters"));
    parameter Integer opening=1 annotation (Dialog(group=
            "Perturbation parameters"), choices(
        choice=1 "Line opening at both ends",
        choice=2 "Line opening at sending end",
        choice=3 "Line opening at receiving end"));
    parameter Boolean displayPF=false "Display power flow results:" annotation (
        Dialog(
        group="Visualisation",
        __Dymola_compact=true,
        __Dymola_descriptionLabel=true), choices(checkBox=true));
    CHEETA.Types.ActivePowerMega P12;
    CHEETA.Types.ActivePowerMega P21;
    CHEETA.Types.ReactivePowerMega Q12;
    CHEETA.Types.ReactivePowerMega Q21;

    Modelica.Electrical.Analog.Interfaces.PositivePin r
      "Recieving end of the transmission line"
      annotation (Placement(transformation(extent={{-106,-10},{-86,10}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin s "Sending end of the line"
      annotation (Placement(transformation(extent={{86,-10},{106,10}})));
protected
    parameter Complex Y(re=G, im=B);
    parameter Complex Z(re=R, im=X);
  equation
    //Calculations for the power flow display
    P12 = real(s.v*conj(s.i))*S_b;
    P21 = -real(r.v*conj(r.i))*S_b;
    Q12 = imag(s.v*conj(s.i))*S_b;
    Q21 = -imag(r.v*conj(r.i))*S_b;
    //PI model with different line openings
    if time >= t1 and time < t2 then
      if opening == 1 then
        r.i = Complex(0);
        s.i = Complex(0);
      elseif opening == 2 then
        r.i = Complex(0);
        s.i = (r.v - r.i*Z)*Y;
      else
        r.i = Complex(0);
        r.v = (s.v - s.i*Z)*Y;
      end if;
    else
      s.v - r.v = Z*(s.i - s.v*Y);
      r.v - s.v = Z*(r.i - r.v*Y);
    end if;
    annotation (Icon(coordinateSystem(preserveAspectRatio=true, initialScale=0.1),
          graphics={Rectangle(
            extent={{-80,40},{80,-40}},
            lineColor={0,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),Rectangle(
            extent={{-60,20},{60,-20}},
            lineColor={0,0,255},
            fillColor={95,95,95},
            fillPattern=FillPattern.Solid),Text(
            visible=displayPF,
            extent={{-200,160},{-20,40}},
            lineColor={255,0,0},
            textString=DynamicSelect("0.0 MW",
              OpenIPSL.NonElectrical.Functions.displayPower(P12, " MW"))),Polygon(
            visible=displayPF,
            points=DynamicSelect({{-120,70},{-120,50},{-80,60},{-120,70}}, if P12
               >= 0 then {{-120,70},{-120,50},{-80,60},{-120,70}} else {{-80,70},
              {-80,50},{-120,60},{-80,70}}),
            lineColor={255,0,0},
            fillColor={255,0,0},
            fillPattern=FillPattern.Solid),Text(
            visible=displayPF,
            extent={{20,160},{200,40}},
            lineColor={255,0,0},
            textString=DynamicSelect("0.0 MW",
              OpenIPSL.NonElectrical.Functions.displayPower(P21, " MW"))),Polygon(
            visible=displayPF,
            points=DynamicSelect({{80,70},{80,50},{120,60},{80,70}}, if P21 >= 0
               then {{80,70},{80,50},{120,60},{80,70}} else {{120,70},{120,50},{
              80,60},{120,70}}),
            lineColor={255,0,0},
            fillColor={255,0,0},
            fillPattern=FillPattern.Solid),Text(
            visible=displayPF,
            extent={{-200,-40},{-20,-160}},
            lineColor={0,255,0},
            textString=DynamicSelect("0.0 Mvar",
              OpenIPSL.NonElectrical.Functions.displayPower(Q12, " Mvar"))),
            Polygon(
            visible=displayPF,
            points=DynamicSelect({{-120,-70},{-120,-50},{-80,-60},{-120,-70}},
              if Q12 >= 0 then {{-120,-70},{-120,-50},{-80,-60},{-120,-70}} else
              {{-80,-70},{-80,-50},{-120,-60},{-80,-70}}),
            lineColor={0,255,0},
            fillColor={0,255,0},
            fillPattern=FillPattern.Solid),Text(
            visible=displayPF,
            extent={{20,-40},{200,-160}},
            lineColor={0,255,0},
            textString=DynamicSelect("0.0 Mvar",
              OpenIPSL.NonElectrical.Functions.displayPower(Q21, " Mvar"))),
            Polygon(
            visible=displayPF,
            points=DynamicSelect({{80,-70},{80,-50},{120,-60},{80,-70}}, if Q21
               >= 0 then {{80,-70},{80,-50},{120,-60},{80,-70}} else {{120,-70},{
              120,-50},{80,-60},{120,-70}}),
            lineColor={0,255,0},
            fillColor={0,255,0},
            fillPattern=FillPattern.Solid),Text(
            extent={{-60,20},{60,-20}},
            lineColor={255,255,0},
            textString="%name")}), Documentation);
  end HTS_firstorder;
end HTS;
