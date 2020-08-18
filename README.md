# CHEETA: Cryogenic High-Efficiency ElectricaL Technologies for Aircraft Modelica Library
This is the repository for the CHEETA aircraft multi-domain models written in Modelica. 
## Scope 
Future increases in demand for high-speed mobility, sustainability, and profitability requires advancements in aviation technologies. This includes the research and exploration of alternative propulsion and energy sources, including fully electrified propulsion. Simulation-based studies are extremely valuable to determine which concepts and methods are most beneficial, which creates the need for this library! This library focuses on the design, modeling, and simulation of an aircraft power system considering multiple domains such as electrical, thermal, aerodynamics, and controls. 

## Goals
The goal for this project is to develop multiple engineering domain aircraft sytsem models to be used for system validation. This also includes developing well-defined models for novel components to be used in the CHEETA system.

## Previous publications on our work!
Many of the models and architectures shown in this library are discussed here: https://doi.org/10.2514/6.2020-3580

Please cite as: Podlaski, M., Vanfretti, L., Nademi, H., Ansell, P. J., Haran, K. S., and Balachandran, T., “Initial Steps in Modeling of CHEETA Hybrid Propulsion Aircraft Vehicle Power Systems using Modelica,” AIAA Propulsion and Energy 2020 Forum, 2020.

#Dependencies
To simulate many of the models in the library, you will need a license to access:
    - Dassault Systems Elecirtified Powertrains Library
    - Dassault Systems Hydrogen Library
    - Dassault Systems Battery Library
    - Dassault Systems Dymola Models Library 
    - Modelon Aircraft Dynamics Library
    - Modelon Electrification Library
We typically use Dymola to use the library, but the library should work in Wolfram System Modeler and OpenModelica.

# Contents
The library is divided into many subpackages:
    - Examples
    - Architectures
    - Aircraft
        - Airframes
        - Electrical
        - Mechanical
        - Controls
        - Thermal
    - Blocks
    - Types
    - Records
    - Atmospheres
    - Functions

Inside of the 'Examples' amd 'Architectures' packages, you can simulate the electrical architecture systems shown in the CHEETA initial modeling paper. How do I simuulate it?
    1. Open ``CHEETA/package.mo`` to open the library.
    2. Navigate to the package and open ``Architectures\Distributed_FuelCell_local_battery.mo`` and select it as ``Simulation model``.
    3. Check and simulate the model.
    
    ## Contributing
- Via pull requests.

## Copyright
(c) Meaghan Podlaski, Abhijit Khare, Luigi Vanfretti, and Hamed Nademi. Rensselaer Polytechnic Institute, Troy, NY.

    
