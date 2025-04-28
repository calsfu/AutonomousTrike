# Hardware Report

How to set up and connect to external system 
## Steering

Steering is achieved using a 24V DC brushless gear motor (Model: NFP-36GP-555-EN) equipped with a built-in positional encoder. This motor is mounted directly onto the tricycle’s central steering joint in a direct-drive position. In this setup, the motor applies torque directly to rotate the front wheels based on the steering angle output determined by the autonomous navigation system.

**Control:**  
An Arduino Uno R3 (steering Arduino) receives steering commands from the processing unit (NVIDIA Orin Nano) via a ROS2 node. The Arduino interfaces with a motor controller, which regulates 24V power supplied to the steering motor. The steering system operates at full speed due to the need for rapid steering angle adjustment and response time.

**Feedback & Safety:**  
The motor's built-in positional encoder communicates with the Orin processing unit to enable precise control of steering. This feedback system prevents oversteering and ensures the steering angle stays within safe limits. Additionally, turn signal lights are integrated into the steering system through an Arduino controller.

---

## Braking

Braking is achieved using two high-torque servo motors, one for each front wheel’s drum brake. The servos are mounted adjacent to the wheels and activate the existing drum brakes by pulling nylon Micro Cord attached to the brake levers, mimicking the action of the manual brake line. Manual braking via handlebar levers remains fully functional inside the tricycle.

**Control:**  
A dedicated Arduino Uno R3 ("Braking Arduino") controls the high-torque servo motors via an Adafruit PCA9685 servo motor driver. This driver receives 6V power from the battery system to activate the servos. The Braking Arduino receives braking commands from the NVIDIA Orin Nano via a ROS2 node, subscribing to the `/control/brake` topic and communicating over a serial connection identified by the Arduino’s serial number. This setup ensures control continuity even if the Orin experiences temporary processing stalls.

**Feedback & Safety:**  
Brake lights are activated by the Braking Arduino via a MOSFET board when mechanized brakes engage. The system also includes an emergency braking feature based on OAK-D camera input and provides haptic feedback to users during braking.

---

## Assistance & Feedback

### Key Fob

**Functionality:**  
A button press on a portable 433 MHz RF key fob triggers a distinct beeping sound from the tricycle, allowing the user to locate it audibly.

**Components:**  
- Key fob transmitter  
- RF receiver module connected to a dedicated Arduino Uno R3  
- 90 dB Piezo buzzer

---

### Haptic Motors

**Functionality:**  
Vibrations alert the user to automated braking actions, serving as an immediate signal to stop pedaling. They may also indicate other system prompts or warnings.

**Components:**  
Three 3.3V, 12,000 RPM Eccentric Rotating Mass (ERM) vibration motors placed on the tricycle seat.

**Control:**  
Activated via a 4-Channel MOSFET board controlled by the Braking Arduino.

---

### Lights

**Purpose:**  
Lights enhance visibility and communicate tricycle actions (e.g., braking, turning).

**Components:**  
- **Brake Lights:** Rear-facing lights (likely red LEDs) indicate engagement of autonomous braking.  
- **Turn Signals:** Side lights that flash during steering maneuvers.  
- **Standard Lighting:** A front white LED headlight and rear red LED taillight for general visibility.

**Control:**  
- **Brake Lights:** Activated via a 12V MOSFET board controlled by the Braking Arduino, only during servo-actuated braking (not manual braking).  
- **Turn Signals:** Controlled by the Steering Arduino, flashing during steering in either Manual or Autonomous modes.

---

### Manual Braking

**Purpose:**  
The manual mechanical braking system remains fully functional, allowing the user to brake at any time if they feel uncomfortable.

**Components:**  
- **Handlebar:** Original handlebar (disconnected from steering pivot) placed freely in the user’s lap.

**Control:**  
The user simply squeezes the brake lever on the handlebar to gradually apply the brakes.



## Wiring Schematic: 

[Wiring Schematic](hardware_sup/WiringSchematic.pdf)


## Steering Mount Diagram:

[Steering Mount Diagram](hardware_sup/SteeringMountSketch.pdf)


## Shaft Collar Diagram:

[Shaft Collar Sketch](hardware_sup/ShaftCollarSketch.pdf)




## Bill of Materials

| Item | Quantity | Description                           | Unit Cost | Extended Cost |
|-----:|---------:|--------------------------------------|----------:|--------------:|
| 1    | 1        | Stock Aluminum                       | 19.96     | 31.33         |
| 2    | 3        | Haptic Motors                         | 0.49      | 1.47          |
| 3    | 1        | Solid Core 20 AWG                     | 16.99     | 16.99         |
| 4    | 1        | Wagos 28pc                            | 20.95     | 25.25         |
| 5    | 1        | M3 Lock Nuts                          | 5.99      | 6.36          |
| 6    | 5        | DC-DC Step Down                       | 10.49     | 52.45         |
| 7    | 1        | Barrel Jack                           | 9.99      | 12.8          |
| 8    | 1        | Speaker                               | 15.98     | 15.98         |
| 9    | 1        | Motor Controller                      | 29.54     | 29.54         |
| 10   | 1        | Unitree L1 Lidar                      | 249.99    | 249.99        |
| 11   | 1        | Adhesive Cable Wire Clips             | 6.99      | 6.99          |
| 12   | 1        | Double-Sided Tape                     | 17.99     | 17.99         |
| 13   | 1        | Wago Inline                           | 25.9      | 25.9          |
| 14   | 1        | Wire Crimp Kit                        | 10.06     | 10.06         |
| 15   | 1        | Inline Fuses                          | 12.49     | 12.49         |
| 16   | 1        | 4-Channel MOSFET PWM                  | 8.49      | 8.49          |
| 17   | 2        | Arduino Screw Terminal Hat            | 18.99     | 37.98         |
| 18   | 1        | Screw Terminal Block Connectors       | 14.99     | 14.99         |
| 19   | 1        | Battery Charger                       | 54.99     | 54.99         |
| 20   | 2        | 12 V Lead Acid Battery                | 44.99     | 89.98         |
| 21   | 2        | Arduino Uno R3 Case                   | 5         | 10            |
| 22   | 1        | 16 AWG 120ft Wire                     | 26.39     | 26.39         |
| 23   | 1        | 18 AWG 13.2ft Wire                    | 15.59     | 15.59         |
| 24   | 1        | NFP-36GP-555-EN Gear Motor            | 45        | 45            |
| 25   | 1        | NVIDIA Orin Nano                      | 249       | 249           |
| 26   | 1        | Truck Trailer Pigtail Connectors      | 9.99      | 9.99          |
| 27   | 2        | DS5160 60 kg-cm Servo Motor           | 30        | 60            |
| 28   | 1        | Keyfob                                | 10        | 10            |
| 29   | 1        | Adafruit PCA9685 Servo Motor Driver   | 7         | 7             |
| 30   | 1        | Velomobile                            | 2000      | 2000          |
| 31   | 1        | Oak-D                                 | 249       | 249           |
| 32   | 1        | Vectornav VN-200                      | 3500      | 3500          |

**Total Cost:** 6903.99

## Vendor Information

  

|Vendor | Details	| Items Purchased |
|---------------:|---------------------:|-----------------:|
|McMaster-Carr|Business-to-business supplier of hardware, tools, raw materials, industrial materials, and maintenance equipment. | 1 |
Amazon | E-commerce seller and distributor. | 2-23, 26-29 | 
NFP-Motor | Motor manufacturer. | 24 |
Nvidia |Computing technology company.| 25 |
Donated | Provided by client | 30-32 |
## Data Sheets

### Steering Motor 
[ Link ](hardware_sup/datasheets/SteeringMotor.pdf)
### Servo Motor
[ Link ](hardware_sup/datasheets/ServoMotor.pdf)
### Servo Motor Controller
[ Link ](hardware_sup/datasheets/ServoMotorController.pdf)
### Steering Motor Controller
[ Link ](hardware_sup/datasheets/SteeringMotorController.pdf)
### Jetson Orin
[ Link ](hardware_sup/datasheets/Orin.pdf)
### Battery
[ Link ](hardware_sup/datasheets/Battery.pdf)
### OAK-D
[ Link ](hardware_sup/datasheets/OAKD.pdf)
### VectorNav VN-200
[ Link ](hardware_sup/datasheets/VN200.pdf)

## Power Requirements
The vehicle is fully sustained off of battery power. The battery specifications are (2) 12V SLA batteries in series with 12Ah capacity. Anderson Powerpole connectors should be connected to the power distribution board at all times unless charging. The onboard Arduino R2 Uno should be powered at all times and can be tested with the external key fob accessory.

When depleted of battery power, the user should use an SLA battery charger at a charging rate safe for the batteries, and a battery balance to ensure both batteries are at the same state of charge before use.

Batteries are located behind the seat and connected with F2 terminals. Batteries should have an output of at least 24V and no more than 36V. Current should never reach above 8A from the battery, but if this is the case, multiple inline fuses exist at critical points with one directly from the battery output.

## Appendix – Photos of enclosure
### Enclosure

[Enclosure](hardware_sup/EnclosurePicture.jpg)

### Brakes

[Brakes](hardware_enclosure/BrakePicture.png)

### Steering

[Steering](hardware_enclosure/MotorPicture.png)

### System

[System](hardware_enclosure/AssembledSystem.png)
