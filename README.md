Behavioral Spherical Harmonic
==============
Behavioral spherical harmonic (BSH) is a novel approach to efficiently and compactly represent the directional-dependent behavior of agents. BSH is based on spherical harmonics to project the directional information of a group of multiple agents to a vector of few coefficients; thus, BSH drastically reduces the complexity of the directional evaluation, as it requires only few agent-group interactions instead of multiple agent-agent ones. 
The BSH model can efficiently model intricate behaviors such as long-range collision avoidance, reaching interactive performance, and avoiding agent congestion on challenging multi-groups scenarios. 
Furthermore, we demonstrate how both the innate parallelism and the compact coefficient representation of the BSH model are well suited for GPU architectures, showing performance analysis of our OpenCL implementation.


- Project's web page: http://bcosenza.github.io/bsh
- Subversion:         https://github.com/bcosenza/bsh
- Author's home:      http://www.biagiocosenza.com http://www.tu-berlin.de/?id=cosenza

If you use this code for research purpose, please cite the paper:

```
@inproceedings{Cosenza15,
  author    = {Biagio Cosenza},
  title     = {{Behavioral Spherical Harmonics for Long-Range Agents' Interaction}},
  booktitle = {Euro-Par 2015: Parallel Processing Workshops - PADABS},
  pages     = {392--404},
  year      = {2015}  
}
```

BSH's Visual Studio project has been tested with Visual Studio 2013 Community Edition and different NVIDIA GPUs.
The Behavioural Spherical Harmonics (BSH) code is licensed under BSD 3 (or "BSD Simplified").
For full license terms please see the LICENSE file distributed with this source code.
