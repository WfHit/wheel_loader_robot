# Wheel Loader Robot - Complete System Architecture

This document provides a comprehensive logical architecture diagram for the w    class ModeManager,ChassisFollower,EndEffFollower,UBridge mainBoard
    class UProxyF,WCF,BC,EKFF,HBRF,QEF1,QEF2,LSF,AS5600F frontBoard
    class UProxyR,WCR,BoomC,LampC,EKFR,HBRR,QER,LSR,AS5600R rearBoard
    class VLA,Remote external
    class HBRF,QEF1,QEF2,LSF,AS5600F,HBRR,QER,LSR,AS5600R hardware
    class UBridge,UProxyF,UProxyR communicationoader robot system based on the complete codebase analysis.

## Overall System Architecture

````mermaid
graph TB
    %% External Systems
    subgraph "External Interface Layer"
        VLA[VLA System<br/>Vision-Language-Action<br/>AI Controller]
        Remote[Remote Operator<br/>Manual Override<br/>RC/Joystick Control]
    end

    %% Domain Control Board (CUAV X7+)
    subgraph "Domain Control Board (CUAV X7+)"
        subgraph "Mode Manager Layer"
            ModeManager[Mode Manager<br/>VLA/Remote Switching<br/>State Management]
        end

        subgraph "Trajectory Follower Layer"
            ChassisFollower[Chassis Follower<br/>Wheel Speed Control<br/>Position Tracking]
            EndEffFollower[End Effector Follower<br/>Bucket + Boom Control<br/>6DOF Trajectory Execution]
        end

        subgraph "Communication Layer"
            UBridge[uORB UART Bridge<br/>Distributed Messaging<br/>Multi-Board Coordinator]
        end
    end

    %% Front Control Board (NXT-Dual-Front)
    subgraph "Front Control Board (NXT-Dual-WL-Front)"
        subgraph "Front Communication"
            UProxyF[uORB UART Proxy<br/>Transparent Messaging<br/>Board Type: Front]
        end

        subgraph "Front Control Modules"
            WCF[Wheel Controller<br/>Front Axle<br/>Drivetrain Controller]
            BC[Bucket Control<br/>Dual-Linkage Kinematics<br/>Stage 1: Drive + Stage 2: Tilt]
            EKFF[EKF Front<br/>State Estimation<br/>Position/Velocity Fusion]
        end

        subgraph "Front Hardware Layer"
            HBRF[H-Bridge Front<br/>DRV8701 Motor Driver<br/>Instance 0]
            QEF1[Encoder Front Wheel<br/>Quadrature Position/Velocity<br/>Instance 0]
            QEF2[Encoder Bucket<br/>Quadrature Position/Velocity<br/>Instance 1]
            LSF[Limit Sensors Front<br/>Bucket Load/Dump Limits<br/>Instance 0-1]
            AS5600F[AS5600 Magnetic<br/>Encoder<br/>Boom Angle Feedback]
        end
    end

    %% Rear Control Board (NXT-Dual-Rear)
    subgraph "Rear Control Board (NXT-Dual-WL-Rear)"
        subgraph "Rear Communication"
            UProxyR[uORB UART Proxy<br/>Transparent Messaging<br/>Board Type: Rear]
        end

        subgraph "Rear Control Modules"
            WCR[Wheel Controller<br/>Rear Axle<br/>Drivetrain Controller]
            BoomC[Boom Control<br/>Lift Kinematics<br/>Electric Actuator]
            LampC[Lamp Controllers<br/>Load + Driver Lamps<br/>Status Display]
            EKFR[EKF Rear<br/>State Estimation<br/>Position/Velocity Fusion]
        end

        subgraph "Rear Hardware Layer"
            HBRR[H-Bridge Rear<br/>DRV8701 Motor Driver<br/>Instance 1]
            QER[Encoder Rear Wheel<br/>Quadrature Position/Velocity<br/>Instance 0]
            LSR[Limit Sensors Rear<br/>Boom Up/Down Limits<br/>Instance 2-3]
            AS5600R[AS5600 Magnetic<br/>Encoder<br/>Boom Position]
        end
    end

    %% External Interfaces
    VLA -.->|6DOF End Effector<br/>Trajectories| ModeManager
    Remote -.->|Manual Control<br/>RC/Joystick| ModeManager

    %% Domain Board Internal Flow - Layered Architecture
    ModeManager --> ChassisFollower
    ModeManager --> EndEffFollower
    ChassisFollower --> UBridge
    EndEffFollower --> UBridge

    %% Distributed Communication (UART)
    UBridge -.->|UART1<br/>921600 baud<br/>CRC32 Protocol| UProxyF
    UBridge -.->|UART2<br/>921600 baud<br/>CRC32 Protocol| UProxyR

    %% Front Board Control Flow
    UProxyF --> WCF
    UProxyF --> BC
    UProxyF --> EKFF
    WCF --> HBRF
    BC --> HBRF
    QEF1 --> WCF
    QEF1 --> EKFF
    QEF2 --> BC
    QEF2 --> EKFF
    LSF --> BC
    AS5600F --> BC
    AS5600F --> EKFF

    %% Rear Board Control Flow
    UProxyR --> WCR
    UProxyR --> BoomC
    UProxyR --> LampC
    UProxyR --> EKFR
    WCR --> HBRR
    BoomC --> HBRR
    QER --> WCR
    QER --> EKFR
    LSR --> BoomC
    AS5600R --> BoomC
    AS5600R --> EKFR

    %% Feedback Loops
    HBRF -.->|Motor Status<br/>Current/Temp| UProxyF
    HBRR -.->|Motor Status<br/>Current/Temp| UProxyR
    UProxyF -.->|System State<br/>Telemetry| UBridge
    UProxyR -.->|System State<br/>Telemetry| UBridge
    UBridge -.->|Aggregated Status<br/>Health Monitoring| ModeManager

    %% Styling
    classDef mainBoard fill:#e1f5fe,stroke:#01579b,stroke-width:2px
    classDef frontBoard fill:#f3e5f5,stroke:#4a148c,stroke-width:2px
    classDef rearBoard fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef safety fill:#ffebee,stroke:#b71c1c,stroke-width:2px
    classDef external fill:#fff3e0,stroke:#e65100,stroke-width:2px
    classDef hardware fill:#f5f5f5,stroke:#424242,stroke-width:2px
    classDef communication fill:#e0f2f1,stroke:#00695c,stroke-width:2px

    class VLAMode,RemoteMode,ModeManager,TrajFollower,UBridge mainBoard
    class UProxyF,WCF,BC,HBRF,QEF1,QEF2,LSF,AS5600F frontBoard
    class UProxyR,WCR,BoomC,LampC,HBRR,QER,LSR,AS5600R rearBoard
    class VLA,Remote,Mission external
    class HBRF,QEF1,QEF2,LSF,AS5600F,HBRR,QER,LSR,AS5600R hardware
    class UBridge,UProxyF,UProxyR communication
````

## Key Architectural Components

### **1. Domain Control System - Layered Architecture**

#### **Mode Manager Layer**
- **Mode Manager**: Centralized VLA/Remote mode switching and state management, processes external commands and coordinates system-wide state transitions

#### **Trajectory Follower Layer**
- **Chassis Follower**: Wheel speed control and position tracking for mobile platform navigation
- **End Effector Follower**: Bucket and boom control with 6DOF trajectory execution for work implement operations

#### **Communication Layer**
- **uORB UART Bridge**: Central communication hub coordinating with distributed NXT boards, handles multi-board messaging with CRC32 protocol

### **2. Communication System**

#### **uORB UART Bridge (Domain Board)**
- **Purpose**: Central communication hub coordinating with distributed NXT boards
- **Protocol**: CRC32 validated, 921600 baud UART communication
- **Features**: Dynamic topic registry, heartbeat monitoring, board-type awareness
- **Architecture**: Single hub managing front and rear board communication

#### **uORB UART Proxy (NXT Boards)**
- **Purpose**: Transparent uORB messaging on distributed control boards
- **Board Types**: Front (bucket + front wheel), Rear (boom + rear wheel)
- **Features**: Intelligent topic filtering, automatic reconnection, local caching

### **3. Control Modules**

#### **Boom Control**
- **Purpose**: Electric actuator control for boom lifting
- **Features**: Sophisticated kinematics, motion planning, safety management
- **Components**: Hardware interface, motion controller, state manager, kinematics

#### **Bucket Control**
- **Purpose**: Dual-linkage kinematics for bucket positioning
- **Stages**: Stage 1 (drive) + Stage 2 (tilt) with boom compensation
- **Features**: Forward/inverse kinematics, trajectory planning, limit integration

#### **Chassis Control**
- **Drivetrain Controllers**: Individual wheel speed control with PID

### **4. Hardware Abstraction Layer**

#### **H-Bridge Drivers (DRV8701)**
- **Multi-instance**: Instance 0 (front), Instance 1 (rear)
- **Features**: PWM control, direction switching, limit sensor integration
- **Safety**: Current monitoring, thermal protection, emergency stop

#### **Quadrature Encoders**
- **Instances**: Front wheel, bucket, rear wheel
- **Features**: Position/velocity feedback, direction detection, configurable resolution
- **Performance**: 100Hz update rate, event-based reset capability

#### **Sensors**
- **AS5600 Magnetic Encoders**: Boom position feedback
- **Limit Sensors**: Load/dump (bucket), up/down (boom)
- **Navigation**: GPS, IMU, UWB for positioning and orientation

## Communication Flow

### **Command Flow (Top-Down) - Layered Processing**
1. **External Input** → VLA system (autonomous) or Remote operator (manual) commands
2. **Mode Manager Layer** → Mode Manager processes external inputs and coordinates state transitions
3. **Trajectory Follower Layer** → Chassis Follower and End Effector Follower execute motion commands
4. **Communication Layer** → uORB Bridge forwards commands to appropriate NXT boards
5. **Local Control** → Board-specific controllers execute motion commands
6. **Hardware Interface** → Direct actuator control (H-bridges)

### **Feedback Flow (Bottom-Up)**
1. **Sensor Data** → Encoders, limit switches, status feedback
2. **Local Processing** → Controller-specific state estimation and health monitoring
3. **Aggregation** → uORB Proxy consolidates board status
4. **Communication Layer** → uORB Bridge provides system-wide status to domain board
5. **Trajectory Follower Layer** → Chassis and End Effector Followers receive system feedback
6. **Mode Manager Layer** → Mode Manager adapts based on system state and external commands

## Key Design Principles

### **Layered Architecture**
The domain control board uses a clear 3-layer architecture: Mode Manager Layer (VLA/Remote switching), Trajectory Follower Layer (chassis and end-effector control), and Communication Layer (distributed messaging).

### **Dual-Mode Operation**
Seamless switching between VLA autonomous mode and Remote manual mode with centralized mode management and dedicated trajectory followers for different subsystems.

### **Distributed Intelligence**
Control intelligence is distributed across boards while maintaining centralized coordination through the layered Domain Control Board, providing both performance and reliability.

### **Industrial Robustness**
All components are designed for 24/7 operation in harsh construction environments with comprehensive fault tolerance.

This layered architecture enables seamless dual-mode operation (VLA autonomous/Remote manual) while maintaining industrial-grade reliability through focused trajectory following and efficient distributed communication.
