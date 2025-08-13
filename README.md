# MCP7940N-ZephyrDriver
Driver for the MCP7940N RTC in Zephyr — supports standard RTC time (not Unix) and alarm configuration.

## Features
- Read and write standard RTC time (calendar date & time, not Unix epoch)
- Configure alarms
- Vcc pin control for enabling/disabling the RTC’s main power supply
- Built for the [Zephyr RTOS](https://zephyrproject.org/)
- Supports I²C communication with MCP7940N

## Requirements
- [Zephyr RTOS](https://docs.zephyrproject.org/)
- I²C-capable MCU supported by Zephyr
- MCP7940N Real-Time Clock IC

## Installation & Setup

Follow these steps to integrate the MCP7940N driver into your Zephyr project (tested with **nRF Connect SDK v2.5.2**):

### 1. Copy Driver Files
Place the following files into the corresponding Zephyr directories:

| File | Destination Path |
|------|------------------|
| `rtc_mcp7940.c` | `ncs\v2.5.2\zephyr\drivers\rtc` |
| `rtc_mcp7940.h` | `ncs\v2.5.2\zephyr\include\zephyr\drivers\rtc` |
| `microchip,mcp7940.yaml` | `ncs\v2.5.2\zephyr\dts\bindings\rtc` |
| `Kconfig.mcp7940` | `ncs\v2.5.2\zephyr\drivers\rtc` |

### 2. Modify Zephyr Build Files
Edit the following Zephyr files to register the new driver:

- **`ncs\v2.5.2\zephyr\drivers\rtc\CMakeLists.txt`**  
  Add:
  - **`zephyr_library_sources_ifdef(CONFIG_RTC_MCP7940 rtc_mcp7940.c)`**

- **`ncs\v2.5.2\zephyr\drivers\rtc\Kconfig`**  
  Add:
  - **`source "drivers/rtc/Kconfig.mcp7940"`**

### 3. DeviceTree Configuration
Add the following to your board’s DeviceTree (.dts or .overlay) file:

    &i2c0 {
        status = "okay";
        pinctrl-0 = <&i2c0_default>;
        pinctrl-1 = <&i2c0_sleep>;
        pinctrl-names = "default", "sleep";
        clock-frequency = <100000>;
    
        mcp7940: mcp7940@6f {
            compatible = "microchip,mcp7940";
            reg = <0x6f>;
            status = "okay";
            label = "MCP7940N_RTC";
            int-gpios = <&gpio0 18 GPIO_ACTIVE_LOW>;  // Adjust to match your interrupt pin
            vcc-gpios = <&gpio1 3 GPIO_ACTIVE_HIGH>;  // Adjust to match your VCC control pin
        };
    };

### 4. Project Configuration
Add the following to your project’s prj.conf:

    CONFIG_I2C=y
    CONFIG_RTC=y
    CONFIG_RTC_MCP7940=y
    CONFIG_RTC_ALARM=y

### 5. Build & Run
Rebuild your project, flash it to the target board, and use the provided sample code to test:

    #include <zephyr/kernel.h>
    #include <zephyr/device.h>
    #include <zephyr/drivers/i2c.h>
    #include <zephyr/drivers/rtc.h>
    #include <zephyr/pm/device.h>
    #include <time.h>

    static const struct device *ext_rtc = DEVICE_DT_GET(DT_NODELABEL(mcp7940));

    struct rtc_time current_time = {
        .tm_year = 2025 - 1900,
        .tm_mon = 7,
        .tm_mday = 25,
        .tm_hour = 16,
        .tm_min = 30,
        .tm_sec = 0,
    };

    struct rtc_time alarm_time = {
        .tm_year = 2025 - 1900,
        .tm_mon = 7,
        .tm_mday = 25,
        .tm_hour = 16,
        .tm_min = 30,
        .tm_sec = 30,
    };

    static void alarm_callback(const struct device *dev, uint16_t id, void *user_data) {
        //Led blink
    }


    int ext_rtc_init(void){
        if (!device_is_ready(ext_rtc)) {
            return -ENODEV;
        }
    
        rtc_alarm_set_callback(ext_rtc, 0, alarm_callback, NULL);
    
        int rc = rtc_set_time(ext_rtc, &current_time);
    
        rc = rtc_alarm_set_time(ext_rtc, 0,
            RTC_ALARM_TIME_MASK_SECOND,
            &alarm_time);
            
        return rc;
    }

    int extRtcHandler_getTime(struct rtc_time *read_time){
        if (!device_is_ready(ext_rtc)) {
            return -ENODEV;
        }
    
        int rc = rtc_get_time(ext_rtc, read_time);
        printk("Current RTC time: %04d-%02d-%02d %02d:%02d:%02d\r\n",
			          read_time->tm_year+1900, read_time->tm_mon, read_time->tm_mday,
			          read_time->tm_hour, read_time->tm_min, read_time->tm_sec);
        return rc;
    }
