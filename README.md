# Experimental validation of event-triggered stabilization over digital channels of plants with disturbances
This repository is for Python codes of the simulations used in the following paper: 

*M. Hedayatpour, M. J. Khojasteh, and M. Franceschetti, "Experimental validation of event-triggered stabilization over digital channels of plants with disturbances," 
Submitted to the 2019 IEEE International Conference on Automation and Robotics (ICRA), Montreal, Canada, 2019.* 

## Abstract
In the context of event-triggered control, the timing of the triggering events contains information about
the state of the system that can be used for stabilization. Therefore, at each triggering event, not only can information
be transmitted by the message content (data payload) but also by its timing. We demonstrate this in the context of
stabilization of a laboratory-scale inverted pendulum around its equilibrium point over a digital communication channel
with bounded unknown delay. Our event-triggering control strategy encodes timing information by transmitting in a
state-dependent fashion and can achieve stabilization using a data payload transmission rate lower than what the datarate theorem prescribes for classical periodic control policies
that do not exploit timing information. Through experimental results, we show that as the delay in the communication channel
increases, a higher data payload transmission rate is required to fulfill the proposed event-triggering policy requirements. This
confirms the theoretical intuition that a larger delay brings a larger uncertainty about the value of the state at the controller,
as less timing information is carried in the communication. We also investigate the robustness of the proposed control scheme
against additional disturbances.

## Running Experiments
To run the scripts and experiments, you would need the prototype robot presented in the first figure below. 
Unfortunately this robot is not currently publicly available, however, the architecture is fairly simple and 
the reader could probably be able to build a prototype. In case you have any questions about the robot, please 
do not hesitate to contact us via the following email:  mkhojast[at]eng.ucsd.edu

The first figure illustrates the prototype robot used in the paper. Second figure shows the architecture of the 
networked control system used in the experiments. 

<p align="center">
  <img width="600" src="https://github.com/mkhojas/Robot-Firmware/blob/master/figures/prototype.png">
</p>
<p align="center">
  <img width="600" src="https://github.com/mkhojas/Robot-Firmware/blob/master/figures/architecture.jpg">
</p>
