<a id="readme-top"></a>


<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a>
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

  <h3 align="center">Speeed ECU</h3>

  <p align="center">
    Building an ECU for an Electric Formula SAE Vehicle
  </p>
</div>

<!-- ABOUT THE PROJECT -->
## About The Project
An Electronic Control Unit (ECU) is a crucial component in modern automotive vehicles, as it essentially acts as the brain by controlling the various electrical subsystems, including safety systems. Electric formula-style race cars need to compete at a high level, so many of these cars have four motors, one for each wheel. Because of this, control and safety systems become more complex and need a stronger ECU to support the vehicle. This project focuses on developing a custom ECU for an in-hub electric Formula SAE-style car. 

Some of the problems existing with current ECUs for a formula style competition, particularly Formula SAE, include cost, usability, and size. Commercial ECU’s sell for a few thousand dollars on average, so it is not ideal for a college club with limited funding. In addition, commercial ECU’s are designed for bigger commercial vehicles and not smaller formula style cars. Their size makes it harder to fit on the vehicle and their weight is non-negligible. They also come with unique programming libraries that may be hard to use or unnecessary for formula racing purposes and require a large overhead for training, especially for college students. 

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## How to Use

### Prerequisites

* [Git](https://git-scm.com/downloads)
* [STM32 Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html#tools-software)
* [KiCad](https://www.kicad.org/download/)


### Flashing Firmware

1. Clone the repo
   ```sh
   git clone https://github.com/shubmi02/Senior_Project-SpeeedECU.git
   ```
2. Open STM32 Cube IDE
3. Press "File" -> "Open Project From File System"
4. Press "Directory"
5. Locate "SpeeedECU" folder in the cloned Repository
6. Press "Finish"
7. Plug in to the ST-Link Micro-USB Header
8. Plug in USB into Computer
9. Press Green Run Button at the top bar of the STM32 Cube IDE 


<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ROADMAP -->
## Roadmap

- [ ] Test Throttle Position Sensor with Nucleo Board (Finish by 9/1)
- [ ] Determine Resistors required to create a linear slope for Throttle Percentages (Finish by 9/1)
- [ ] Test Brake Pressure Sensor with Nucleo Board (Finish by 9/1)
- [ ] Send PCB Revision A Board (Finish by 9/14)


<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Team Members

Brandon Louie - brandon.m.louie@sjsu.edu
<br>
Brandon Luong - brandon.luong01@sjsu.edu
<br>
Ajay Paramasivan - ajay.paramasivan@sjsu.edu
<br>
Shubham Mishra - shubham.mishra@sjsu.edu


<p align="right">(<a href="#readme-top">back to top</a>)</p>
