created 2021-09-21 by MAB
original code created 2018-07-16 by MAB and SAB
code updated during summer 2021 as a hotfix until permanent solution implemented
	hotfix seems to have been working fine for a month now
	
# Tonneau refill system, prototype 0

## system diagram


## history

This first prototype was created in Switzerland in 2018 with limited resources.

It worked fine except for the fact that the overflow warning triggered unnecessarily sometimes.

The 2021 patch debounced the overflow sensor input, and seemed to solve the problem.
The "bouncing" of that input is believed to come from the long wires picking EMI up badly
enough to overwhelm the input pullup of the arduino.