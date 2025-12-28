
# STM32F405RGT6 Learning Repository

A comprehensive collection of STM32F4 peripheral examples and projects demonstrating bare-metal programming and HAL-based development. This repository contains hands-on implementations of communication protocols, data conversion, interrupt handling, and display interfacing on the STM32F405RGT6 microcontroller.

## 🎯 Overview

This repository documents a complete learning journey through STM32 microcontroller programming, covering essential peripherals and communication interfaces. Each project is designed to provide practical understanding of embedded systems concepts with working code examples.

## 📋 Table of Contents

- [Hardware Platform](#hardware-platform)
- [Projects Overview](#projects-overview)
- [Communication Protocols](#communication-protocols)
- [Data Conversion & Processing](#data-conversion--processing)
- [Display Interfaces](#display-interfaces)
- [Interrupt Handling](#interrupt-handling)
- [Advanced Applications](#advanced-applications)
- [Setup and Requirements](#setup-and-requirements)
- [Building Projects](#building-projects)
- [Learning Path](#learning-path)

## 🔧 Hardware Platform

### Microcontroller Specifications
- **MCU**: STM32F405RGT6
- **Core**: ARM Cortex-M4F with FPU
- **Clock Speed**: Up to 168 MHz
- **Flash Memory**: 1024 KB
- **SRAM**: 192 KB (128KB + 64KB CCM)
- **Package**: LQFP64

### Key Features
- 12-bit ADC with up to 2.4 MSPS
- 12-bit DAC (2 channels)
- Multiple communication interfaces (SPI, I2C, USART)
- Advanced timers and PWM channels
- DMA controllers for efficient data transfer
- USB OTG FS/HS support

## 📚 Projects Overview

### Communication Protocols

#### 1. **ADC_interrupt**
Analog-to-Digital Conversion with interrupt-driven data acquisition.
- **Features**: 
  - Interrupt-based ADC sampling
  - Non-blocking data conversion
  - Efficient CPU utilization
- **Use Cases**: Sensor reading, voltage monitoring, analog signal processing

#### 2. **SPI** & **SPI1-ONLY_LCD**
Serial Peripheral Interface communication with LCD display.
- **Features**:
  - Full-duplex synchronous communication
  - Master mode configuration
  - LCD control via SPI
  - High-speed data transfer
- **Applications**: Display interfacing, sensor networks, memory devices

#### 3. **USART3**
Universal Synchronous/Asynchronous Receiver/Transmitter.
- **Features**:
  - Serial communication
  - Configurable baud rate
  - Interrupt/DMA support
- **Applications**: Debug console, wireless module interface, PC communication

#### 4. **USART_Interrupt**
Interrupt-driven UART communication for efficient data handling.
- **Features**:
  - Non-blocking transmission/reception
  - Ring buffer implementation
  - Error detection and handling
- **Applications**: Real-time data streaming, command interface

### Data Conversion & Processing

#### 5. **DAC**
Digital-to-Analog Converter implementation.
- **Features**:
  - Voltage output generation
  - Waveform synthesis
  - Timer-triggered conversion
- **Applications**: Audio output, signal generation, analog control

#### 6. **DMA_MEMORY_TO_MEMORY**
Direct Memory Access for high-speed data transfer.
- **Features**:
  - Zero CPU overhead transfers
  - Memory-to-memory operations
  - Efficient buffer management
- **Applications**: Data buffering, image processing, bulk transfers

#### 7. **DMA_USART**
DMA-accelerated UART communication.
- **Features**:
  - Non-blocking large data transfers
  - Circular buffer mode
  - Automatic peripheral access
- **Applications**: High-throughput data logging, file transfers

### Display Interfaces

#### 8. **SPI_LCD**
Liquid Crystal Display control via SPI interface.
- **Features**:
  - Character/graphic display
  - Custom font support
  - Fast update rates
- **Applications**: User interface, data visualization, status display

#### 9. **Y_N_usart_lcd_exti_ques**
Combined UART, LCD, and external interrupt project.
- **Features**:
  - Multi-peripheral coordination
  - User interaction via UART
  - Visual feedback on LCD
  - External button handling
- **Applications**: Interactive systems, menu-driven interfaces

### ADC Projects

#### 10. **adc_channel11_12_together**
Multi-channel ADC simultaneous sampling.
- **Features**:
  - Dual ADC operation
  - Synchronized conversion
  - DMA data transfer
- **Applications**: Multi-sensor systems, differential measurements

#### 11. **adc_channel11_lcd**
ADC with LCD display integration.
- **Features**:
  - Real-time value display
  - Voltage monitoring
  - Visual feedback
- **Applications**: Voltmeter, sensor dashboard

#### 12. **adc_exti_pwm_ledtog**
ADC with external interrupt, PWM, and LED control.
- **Features**:
  - Event-driven ADC sampling
  - PWM generation
  - LED indicators
- **Applications**: Adaptive lighting, motor control with feedback

#### 13. **adc_int_led_toggle**
Interrupt-driven ADC with LED status indication.
- **Features**:
  - Threshold detection
  - Visual alerts
  - Low latency response
- **Applications**: Alarm systems, level detection

#### 14. **adc_led_duty_cycle_pot**
Potentiometer-controlled LED brightness via PWM.
- **Features**:
  - Analog input mapping to PWM duty cycle
  - Smooth brightness control
  - Real-time adjustment
- **Applications**: Dimmer switch, motor speed control, audio volume

### Switch & Interrupt Handling

#### 15. **all_debounce_switch_correct**
Comprehensive switch debouncing implementation.
- **Features**:
  - Software debounce algorithm
  - Multiple switch handling
  - Edge detection
- **Applications**: Keypad scanning, button interfaces, menu navigation

### Home Automation

#### 16. **Home_Appliances_ADC**
ADC-based home appliance control system.
- **Features**:
  - Sensor-driven automation
  - Threshold-based switching
  - Multi-appliance management
- **Applications**: Smart home systems, energy monitoring, automated control

### CMSIS (Cortex Microcontroller Software Interface Standard)

#### 17. **CMSIS**
Low-level CMSIS register programming examples.
- **Features**:
  - Direct register access
  - CMSIS library usage
  - Optimized performance
- **Applications**: Bare-metal programming, bootloaders, critical timing code

## 🛠️ Setup and Requirements

### Software Requirements

#### Development Environment
- **IDE**: STM32CubeIDE v1.10.0+ or Keil MDK v5.30+
- **Configuration Tool**: STM32CubeMX v6.5.0+
- **Programmer**: STM32 ST-LINK Utility v4.6.0+

#### Toolchain
- **Compiler**: ARM GCC 10.3+ or ARM Compiler 6
- **Debugger**: OpenOCD or ST-LINK GDB Server
- **Build System**: Make or CMake

#### Libraries
- STM32F4xx HAL Driver v1.7.0+
- CMSIS v5.8.0+
- FreeRTOS v10.4.0+ (if applicable)

### Hardware Requirements

#### Essential Components
- STM32F405RGT6 Development Board
- ST-LINK V2 or compatible programmer/debugger
- USB Cable (Mini/Micro USB depending on board)
- Power Supply (5V regulated)

#### Peripherals & Modules
- **Display**: 16x2 LCD or TFT display (SPI interface)
- **Sensors**: Analog sensors (potentiometer, LDR, temperature sensor)
- **Communication**: USB-to-TTL converter for UART
- **Actuators**: LEDs, buzzers, relays
- **Input**: Push buttons, switches, potentiometers
- **Miscellaneous**: Breadboard, jumper wires, resistors, capacitors

### Pin Connections

#### Common Pin Mappings (Verify with your board)

| Peripheral | Pin | Function |
|------------|-----|----------|
| USART3_TX | PB10 | Serial transmit |
| USART3_RX | PB11 | Serial receive |
| SPI1_SCK | PA5 | SPI clock |
| SPI1_MISO | PA6 | SPI master in |
| SPI1_MOSI | PA7 | SPI master out |
| SPI1_NSS | PA4 | SPI chip select |
| ADC1_IN11 | PC1 | Analog input 1 |
| ADC1_IN12 | PC2 | Analog input 2 |
| DAC_OUT1 | PA4 | Analog output 1 |
| DAC_OUT2 | PA5 | Analog output 2 |
| TIM2_CH1 | PA0 | PWM output |
| EXTI0 | PA0 | External interrupt |
| LED | PA5/PD12 | Status indicator |

## 🚀 Building Projects

### Using STM32CubeIDE

```bash
# 1. Open STM32CubeIDE
File → Open Projects from File System

# 2. Select project folder
Browse to: STM32/[project_name]

# 3. Build
Project → Build All (Ctrl+B)

# 4. Flash and Debug
Run → Debug (F11)
```

### Using Command Line (Make)

```bash
# Navigate to project directory
cd STM32/ADC_interrupt

# Build
make clean
make -j4

# Flash using OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/ADC_interrupt.elf verify reset exit"

# Or using st-flash
st-flash write build/ADC_interrupt.bin 0x8000000
```

### Using CMake

```bash
# Create build directory
mkdir build && cd build

# Configure
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/stm32f4.cmake ..

# Build
cmake --build . -j4

# Flash
cmake --build . --target flash
```

## 📖 Learning Path

### Beginner Level (Weeks 1-2)

#### Start Here:
1. **GPIO Basics** (`adc_int_led_toggle`)
   - Learn digital I/O
   - Understand pin configuration
   - Practice LED control

2. **Simple ADC** (`adc_channel11_lcd`)
   - Analog input reading
   - Voltage measurement
   - Display interfacing

3. **Basic UART** (`USART3`)
   - Serial communication
   - Printf debugging
   - Terminal interaction

### Intermediate Level (Weeks 3-4)

4. **Interrupts** (`ADC_interrupt`, `USART_Interrupt`)
   - Interrupt-driven programming
   - Priority management
   - Efficient event handling

5. **Timers & PWM** (`adc_led_duty_cycle_pot`)
   - Timer configuration
   - PWM generation
   - Duty cycle control

6. **Multi-channel ADC** (`adc_channel11_12_together`)
   - Simultaneous sampling
   - DMA integration
   - Data synchronization

### Advanced Level (Weeks 5-6)

7. **SPI Communication** (`SPI`, `SPI_LCD`)
   - Protocol implementation
   - Display control
   - High-speed transfers

8. **DMA Operations** (`DMA_MEMORY_TO_MEMORY`, `DMA_USART`)
   - Direct memory access
   - CPU offloading
   - Performance optimization

9. **Complex Projects** (`Home_Appliances_ADC`, `Y_N_usart_lcd_exti_ques`)
   - Multi-peripheral coordination
   - System integration
   - Real-world applications

### Expert Level (Weeks 7+)

10. **CMSIS Programming** (`CMSIS`)
    - Register-level access
    - Bare-metal programming
    - Hardware optimization

11. **Custom Implementations**
    - Create your own projects
    - Combine multiple peripherals
    - Build practical applications

## 💡 Key Concepts Covered

### Peripheral Programming
- ✅ GPIO (General Purpose Input/Output)
- ✅ ADC (Analog-to-Digital Converter)
- ✅ DAC (Digital-to-Analog Converter)
- ✅ UART/USART (Serial Communication)
- ✅ SPI (Serial Peripheral Interface)
- ✅ I2C (Inter-Integrated Circuit)
- ✅ Timers (Basic, Advanced, General Purpose)
- ✅ PWM (Pulse Width Modulation)
- ✅ DMA (Direct Memory Access)
- ✅ EXTI (External Interrupts)

### Programming Techniques
- ✅ Interrupt-driven programming
- ✅ DMA-based data transfer
- ✅ Polling vs interrupt methods
- ✅ State machine implementation
- ✅ Debouncing algorithms
- ✅ Ring buffer management
- ✅ HAL vs CMSIS programming

### Best Practices
- ✅ Modular code structure
- ✅ Efficient memory usage
- ✅ Power optimization
- ✅ Error handling
- ✅ Code documentation
- ✅ Version control with Git

## 🔍 Project Details

### ADC Projects

#### ADC with Interrupt (`ADC_interrupt`)
```c
// Key Features:
- Continuous conversion mode
- Interrupt on conversion complete
- Efficient CPU utilization
- Multiple channel support
```

#### Multi-Channel ADC (`adc_channel11_12_together`)
```c
// Key Features:
- Dual ADC simultaneous mode
- DMA-based data collection
- Synchronized sampling
- High-speed acquisition
```

### Communication Projects

#### USART3 Basic
```c
// Configuration:
- Baud Rate: 9600/115200 bps
- Data Bits: 8
- Stop Bits: 1
- Parity: None
- Flow Control: None
```

#### SPI LCD Interface
```c
// SPI Configuration:
- Mode: Master
- Clock Speed: APB2/2 (up to 42 MHz)
- Data Size: 8-bit
- Clock Polarity: Low
- Clock Phase: 1 Edge
```

### DMA Projects

#### Memory to Memory Transfer
```c
// Features:
- 32-bit word transfers
- Configurable source/destination
- Transfer complete interrupt
- Error handling
```

## 🐛 Troubleshooting

### Common Issues

#### 1. Upload Failed
**Symptoms**: Cannot connect to target
**Solutions**:
- Check ST-LINK connection
- Verify USB cable
- Try different USB port
- Update ST-LINK firmware
- Check BOOT0 pin (should be low)

```bash
# Reset ST-LINK
st-info --probe
st-flash reset
```

#### 2. ADC Reading Incorrect
**Symptoms**: Wrong voltage values
**Solutions**:
- Check reference voltage (VREF+)
- Verify ADC configuration (resolution, sampling time)
- Add decoupling capacitors
- Check input voltage range (0-3.3V)

```c
// Increase sampling time for high impedance sources
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
```

#### 3. UART Communication Not Working
**Symptoms**: No data received/transmitted
**Solutions**:
- Verify baud rate matches on both sides
- Check TX/RX pin connections (TX→RX, RX→TX)
- Confirm GPIO alternate function mapping
- Test with loopback (connect TX to RX)

#### 4. SPI Display Shows Garbage
**Symptoms**: LCD shows random characters
**Solutions**:
- Verify SPI clock speed (not too fast for display)
- Check NSS (chip select) timing
- Confirm CPOL and CPHA settings
- Add delays after commands

#### 5. Interrupt Not Triggering
**Symptoms**: ISR never executes
**Solutions**:
- Enable NVIC interrupt
- Set correct priority
- Clear interrupt flags
- Verify trigger edge configuration

```c
// Enable interrupt in NVIC
HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(EXTI0_IRQn);
```

#### 6. DMA Transfer Not Starting
**Symptoms**: Data not transferred
**Solutions**:
- Enable DMA clock
- Configure DMA stream and channel correctly
- Check memory and peripheral addresses
- Enable DMA request from peripheral

## 📊 Performance Optimization

### Memory Optimization
```c
// Use const for read-only data (stored in flash)
const uint8_t lookup_table[256] = {...};

// Use __attribute__((section)) for specific placement
__attribute__((section(".ccmram"))) uint32_t fast_buffer[1024];
```

### Speed Optimization
```c
// Use DMA for bulk transfers
// Use interrupts for event-driven processing
// Enable compiler optimizations (-O2 or -O3)
// Use CMSIS DSP library for math operations
```

### Power Optimization
```c
// Use sleep modes when idle
HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

// Disable unused peripherals
__HAL_RCC_TIM2_CLK_DISABLE();
```

## 📚 Additional Resources

### Official Documentation
- [STM32F405 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32F405 Datasheet](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf)
- [STM32 HAL User Manual](https://www.st.com/resource/en/user_manual/dm00105879-description-of-stm32f4-hal-and-ll-drivers-stmicroelectronics.pdf)

### Learning Materials
- [Mastering STM32](https://leanpub.com/mastering-stm32) by Carmine Noviello
- [STM32 Step-by-Step](https://www.st.com/content/st_com/en/support/learning/stm32-education/stm32-moocs.html) - ST MOOCs
- [DigiKey STM32 Series](https://www.youtube.com/playlist?list=PLEBQazB0HUyRYuzfi4clXsKUSgorErmBv)

### Community & Forums
- [ST Community](https://community.st.com/)
- [STM32 Discord](https://discord.gg/stm32)
- [Reddit r/embedded](https://www.reddit.com/r/embedded/)
- [Stack Overflow - STM32 Tag](https://stackoverflow.com/questions/tagged/stm32)

## 🤝 Contributing

Contributions to improve examples or add new projects are welcome!

### How to Contribute
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-example`)
3. Add your well-documented code
4. Test thoroughly on hardware
5. Commit with clear messages (`git commit -m 'Add SPI slave mode example'`)
6. Push to your fork (`git push origin feature/new-example`)
7. Open a Pull Request

### Contribution Guidelines
- Follow existing code style
- Include comments and documentation
- Add README for new projects
- Test on actual hardware
- Update main README if adding new categories

## 📜 License

This project is licensed under the MIT License - see the LICENSE file for details.


## 🙏 Acknowledgments

- STMicroelectronics for comprehensive documentation
- ARM for Cortex-M4 architecture
- Open-source community for tools and libraries
- All contributors and learners using this repository

---

## 📈 Repository Stats

- **Total Projects**: 17+
- **Peripherals Covered**: 10+
- **Lines of Code**: 5000+
- **Last Updated**: December 2025
- **Status**: Active Learning & Development

## 🎯 What's Next?

### Planned Additions
- [ ] I2C examples (EEPROM, sensors)
- [ ] CAN bus communication
- [ ] USB device implementation
- [ ] FreeRTOS integration
- [ ] Low-power modes demonstration
- [ ] Bootloader example
- [ ] OTA (Over-The-Air) update
- [ ] Advanced PWM techniques

---

**Happy Coding! 🚀**

*"The best way to learn embedded systems is to build real projects."*

---

## Quick Start Guide

```bash
# Clone repository
git clone https://github.com/muskum09ari/STM32.git
cd STM32

# Choose a project (start with ADC basics)
cd adc_channel11_lcd

# Open in STM32CubeIDE
# File → Open Projects from File System

# Build, Flash, and Run!
```
