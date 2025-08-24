# klipper-laser
High performance laser engraving firmware

Adapt Klipper to laser engraving applications, with various optimizations, including:
- Combined movement of traditional X/Y gantry, and two-axis galvo
- Rapid PWM switching for laser engraving applications
- Optimization of consecutive scanning moves for raster engrave. Up to 4000mm/s with 0.05mm line spacing achieved (Raspberry Pi 4B)
- Dynamic switching between blue and red laser lasers in dual Laser Source
- HotPlug Roller
- Host Drive galvanometer mirrors via RP2040 PIO interface with XY-100 protocol
- Safety Collision Prevention
- Safety Door Control
- Swing Arm
- Z-Axis Button Control

Limitations and Challenges:
- 

Development led by: Jinqiang CHEN. Find me at: jinqiang@ecomedge.io.
Project started: July 10, 2024 by Leo QU. Initial Klipper commit pulled is 248d3db.
