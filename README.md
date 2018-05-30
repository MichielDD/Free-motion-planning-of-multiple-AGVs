# Free motion planning of multiple AGVs using a spline based-approach

This repository is part of the master thesis of Michiel De Deken, MSc. in Engineering: Logistics & Traffic @ KU Leuven: <br>
<b><br><br>
Free motion planning of multiple AGVs using a spline based-approach:<br>
<i>A multi-frame technique applied to a warehouse environment </i>
</b><br><br>
June 2018

## Author

* **Michiel De Deken**

## Acknowledgments

* Inspired by, and using code fragments out of the OMG-tools (https://github.com/meco-group/omg-tools)

## Installation
The code is written in Python 2.7 and requires the installation of the following packages to run:

`sudo apt-get install python-pip python-numpy python-scipy python-matplotlib`

To save simulation results in gif-format, you need [imagemagick](www.imagemagick.org). For Linux Debian users:

`sudo add-apt-repository main && apt-get update && install imagemagick `

For faster solving the free motion planning problems, the [HSL linear solvers](https://github.com/casadi/casadi/wiki/Obtaining-HSL) are deployed.

## Concept

Holonomic AGVs execute a transportation job in a simplified warehouse environment (symmetrically placed racks). Their trajectory is not based on predefined, uni/bidirectional pathways which can be reserved, but on the idea that the motion space is fully exploited. 

## Some results 

<table style="border: none; border-collapse: collapse;" border="0" cellspacing="0" cellpadding="0" width="100%" align="center">
<tr>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);" width="33%">
<img width=100% src="./phase4/finalgifs/v10r2c2s15.gif" alt="10 AGVs, 2x2 racks"/>
</td>
</tr>
<tr>
<td align="center" valign="center" bgcolor="#FFFFFF" width="33%">
<img width=100% src="./phase4/finalgifs/v10r2c2I.gif" alt="10 AGVs, 2x2 racks"/>
</td>
</tr>
<tr>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);" width="33%">
<img width=100% src="./phase4/finalgifs/v15r6c6s327.gif" alt="15 AGVs, 6x6 racks"/>
</td>
</tr>
<tr>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);" width="33%">
<img width=100% src="./phase4/finalgifs/v15r5c6s237.gif" alt="15 AGVs, 5x6 racks"/>
</td>
</tr>
</table>
