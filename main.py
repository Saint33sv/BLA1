import yaml
from mathModelQuadrotor import *
from simulator import *
from control import *
from message import StateVector



with open(r"config/quadModelConfig.yaml", 'r') as file:
    paramsQuadrotor = yaml.safe_load(file)
        
stateVector = StateVector()
mathModel = MatModelQuadrotor(paramsQuadrotor)
controlSystem = ControlSystem()
controlSystem.sed_desired_posotion(5.0, 5.0, 10.0)

simulator = Simulator(paramsQuadrotor['simulationTotalTime'], paramsQuadrotor['simulationStep'], mathModel, controlSystem, stateVector)
simulator.run()