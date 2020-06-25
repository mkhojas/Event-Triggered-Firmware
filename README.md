# Experimental validation of event-triggered stabilization over digital channels of plants with disturbances
This repository is for Python codes of the simulations used in the following paper: 

*[Mojtaba Hedayatpour](https://mojcris.github.io/), [Mohammad Javad Khojasteh](http://www.its.caltech.edu/~mjkhojas/), and Massimo Franceschetti, "Theory and implementation of event-triggered stabilization over digital channels," 
58th IEEE Conference on Decision and Control (CDC), 4183– 4188, 2019.* 

## Abstract
In the context of event-triggered control, the timing of the triggering events carries information about the state of the system that can be used for stabilization. At each triggering event, not only can information be transmitted by the message content (data payload) but also by its timing. We demonstrate this in the context of stabilization of a laboratory- scale inverted pendulum around its equilibrium point over a digital communication channel with bounded unknown delay. Our event-triggering control strategy encodes timing informa- tion by transmitting in a state-dependent fashion and can achieve stabilization using a data payload transmission rate smaller than what the data-rate theorem prescribes for classical periodic control policies that do not exploit timing information. Through experimental results, we show that as the delay in the communication channel increases, a higher data payload trans- mission rate is required to fulfill the proposed event-triggering policy requirements. This confirms the theoretical intuition that a larger delay brings a larger uncertainty about the value of the state at the controller, as less timing information is carried in the communication. Our results also provide a novel encoding- decoding scheme to achieve input-to-state practically stability (ISpS) for nonlinear continuous-time systems under appropriate assumptions.

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
