# rlc-oscillator

<p align="center"><a href="http://www.labcontrol.xyz/dokuwiki" target="_blank" rel="noopener"><img src="https://github.com/labcontrol-data/images/blob/main/logo.png"></a></p>

# [Labcontrol, Brazil](http://www.labcontrol.xyz/dokuwiki)

[**Labcontrol**](http://www.labcontrol.xyz/dokuwiki)  is a research laboratory located at Universidade Tecnológica Federal do Paraná (UTFPR), Brazil. UTFPR is a Brazilian public university located in the Paraná state, Brazil. [**Labcontrol**](http://www.labcontrol.xyz/dokuwiki)  develops research on Control Systems and Automation. The Scientific Director of Labcontrol is [Prof. Dr. Alessandro N. Vargas](http://www.anvargas.com). The projects hosted in [**Labcontrol**](http://www.labcontrol.xyz/dokuwiki)  are [described in this link (click here).](http://www.anvargas.com/blog)

About
============

This page provides information about the project developed in Labcontrol called ["Deadbeat condition for the control design of RLC Oscillator."](http://www.anvargas.com/blog/rlcoscillator.html)  Experiments were carried out in practice in a laboratory testbed, and the data contained in this GitHub repository were collected in those experiments. 

[![DOI](https://zenodo.org/badge/577329848.svg)](https://zenodo.org/badge/latestdoi/577329848)

**Please check more details about this project in the page detailing the ["Deadbeat condition for the control design of RLC Oscillator."](http://www.anvargas.com/blog/rlcoscillator.html)**


`maincode.m` is a MATLAB(R) script that calls Matlab and generates simulation data. The script also generates figures. The figures contain both simulation and real-time data collected in a laboratory testbed.

For more details about the experimental data, as long as the corresponding academic publications, please visit [the project page](http://www.anvargas.com/blog).


Installation
============

1. Extract the ZIP file (or clone the git repository) in your computer.
2. Add the folders `matlab-code/` and `data/` to your path in MATLAB/Octave: e.g. 
    - using the "Set Path" dialog in MATLAB, or 
    - by running the `addpath` function from your command window or `startup` script.

Make sure that you are running Matlab 2017a (or a newer version). Older versions may work, but it is uncertain.

Usage
=====

Typical usage of `maincode.m` consists of running it in your MATLAB. The code generates ten figures.

MATLAB
------
  1. Run `maincode.m`.

More information
================

* For more information about `maincode.m`, visit the author's page: [Prof. Alessandro N. Vargas](http://www.anvargas.com). You are welcome to help improving the code.
* You are free to use the data in your research. If you do so, please contact the author [Prof. Alessandro N. Vargas](http://www.anvargas.com) 
and let him know about your project. Depending on your research area, the author can help you interpret the data according to your application. The author can also suggest papers and books that can be helpful in your research.

[![DOI](https://zenodo.org/badge/577329848.svg)](https://zenodo.org/badge/latestdoi/577329848)

Citation
------
How to cite the data of this repository:

```
@misc{vargasGithub2021,
    author       = {A. N. Vargas},
    title        = {Data, source code, and documents for the RLC Oscillator}},
    month        = {Dec},
    year         = 2022,
    doi          = {10.5281/zenodo.577329848},
    version      = {1.0.1},
    publisher    = {Zenodo},
    url          = {https://zenodo.org/badge/latestdoi/577329848}
};
```



