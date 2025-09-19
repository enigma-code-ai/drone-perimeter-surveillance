
# Autonomous Drone Surveillance System – CAD Files

This folder contains the **SolidWorks CAD models, assemblies, and drawings** for the Autonomous Drone Surveillance Project.  
The models are structured to represent the drone’s **flight system, surveillance subsystems, counter-drone modules, and support components**.

---

## 📁 Folder Structure

```
Drone_Surveillance_Project/
│
├── Parts/                # Individual CAD components (.sldprt)
│   ├── Drone_Frame.sldprt
│   ├── Propeller.sldprt
│   ├── Motor_Housing.sldprt
│   ├── Battery_Pack.sldprt
│   ├── Camera_Gimbal.sldprt
│   ├── Sensor_Housing.sldprt
│   ├── Landing_Gear.sldprt
│   ├── Speaker_Module.sldprt
│   └── Spotlight_Casing.sldprt
│
├── Assemblies/           # Subsystem and full-system assemblies (.sldasm)
│   ├── Drone_Main_Assembly.sldasm
│   ├── Camera_Gimbal_Assembly.sldasm
│   ├── Sensor_Pod_Assembly.sldasm
│   ├── CounterDrone_Module.sldasm
│   └── PowerSystem_Assembly.sldasm
│
└── Drawings/             # 2D manufacturing & documentation drawings (.slddrw)
├── Drone_Frame.slddrw
├── Propeller.slddrw
├── Camera_Gimbal.slddrw
├── Landing_Gear.slddrw
├── Sensor_Pod.slddrw
└── Full_Drone_Assembly.slddrw
```

---

## 🔧 File Types

- **Parts (`.sldprt`)** → Core components (frame, blades, mounts, housings).  
- **Assemblies (`.sldasm`)** → Functional subsystems and the full drone model.  
- **Drawings (`.slddrw`)** → Technical 2D drawings with dimensions, tolerances, BOMs, and exploded views.

---

## 📌 Notes for Use

- The CAD models are **conceptual** and optimized for demonstration, not manufacturing.  
- Materials (e.g., carbon fiber, ABS, aluminum) are set at part level for weight estimation.  
- Assemblies are built with **mates** for realistic kinematics (e.g., gimbal rotation, landing gear retraction).  
- Drawings include **exploded views** and **BOM tables** for easier part tracking.  

---

## 🚀 Future Work

- Add **detailed electronics housing** (flight controller, comms module).  
- Expand **deterrence modules** (net launcher, strobe light).  
- Integrate **aerodynamic simulations** (SolidWorks Flow Simulation).  
- Perform **FEA analysis** on arms, frame, and landing gear.  

---
