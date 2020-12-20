## I²C/TWI (Inter-Integrated Circuit, bzw. Two-Wire-Interface)
***

> [⇧ **Home**](https://github.com/iotkitv4/intro)

![](https://raw.githubusercontent.com/iotkitv4/intro/main/images/I2C.png) 

[I²C-Bus mit einem Master und drei Slaves](http://de.wikipedia.org/wiki/I%C2%B2C)

- - -

I²C, für englisch Inter-Integrated Circuit, im Deutschen gesprochen als I-Quadrat-C oder englisch I-Squared-C oder I-2-C, ist ein von Philips Semiconductors (heute NXP Semiconductors) entwickelter serieller Datenbus.

Der Bus wurde 1982 von Philips eingeführt zur Geräte internen Kommunikation zwischen ICs in z.B. CD-Spielern und Fernsehgeräten.

I²C ist als **Master-Slave-Bus** konzipiert. Ein Datentransfer wird immer durch einen Master (die MCU auf dem Board) initiiert; der über eine Adresse angesprochene Slave (die Sensoren auf dem Board) reagiert darauf. Mehrere Master sind möglich (Multimaster-Mode). Im Multimaster-Mode können zwei Master-Geräte direkt miteinander kommunizieren, dabei arbeitet ein Gerät als Slave.

Der **Bustakt** wird immer vom Master ausgegeben. Für die verschiedenen Modi ist jeweils ein maximal erlaubter Bustakt vorgegeben. In der Regel können aber auch beliebig langsamere Taktraten verwendet werden, falls diese vom Master-Interface unterstützt werden. Bestimmte ICs (z.B. Analog-Digital-Umsetzer) benötigen jedoch eine bestimmte, minimale Taktfrequenz, um ordnungsgemäss zu funktionieren.

Eine **Standard-I²C-Adresse ist das erste vom Master gesendete Byte**, wobei die ersten sieben Bit die eigentliche Adresse darstellen und das achte Bit (R/W-Bit) dem Slave mitteilt, ob er Daten vom Master empfangen soll (LOW), oder Daten an den Master zu übertragen hat (HIGH). I²C nutzt daher einen Adressraum von 7 Bit, was bis zu **112 Knoten auf einem Bus** erlaubt (16 der 128 möglichen Adressen sind für Sonderzwecke reserviert).

Das Protokoll des I²C-Bus ist von der Definition her recht einfach, aber physikalisch auch recht störanfällig. Auch ist er **ungeeignet zur Überbrückung größerer Entfernungen**. Der Bus kann jedoch mit speziellen Treibern auf ein höheres Strom- und/oder Spannungslevel umgesetzt werden, wodurch der Störabstand und die mögliche Leitungslänge steigen.

Der Bus braucht zur Terminierung zwei Widerstände von ca. 2.2K - 4.7K Ohm (je nach Länge der Kabel, siehe 6-Achsen-Sensor) und alle Geräte müssen an der gleichen Ground Leitung angeschlossen sein.

**Die I2C Sensoren und Aktoren auf dem Board sind anhand der Hexadecimalen Nummer, z.B. 0x78 neben dem OLED Display, zu erkennen.**

### Anwendungen 

*   Ansprechen von Peripherie Bausteinen wie Temperatursensor, [3-axis Beschleunigungsmesser (Accelerometer) and Magnetfeldstärkenmessgerät (Magnetometer)](http://developer.mbed.org/users/JimCarver/code/FXOS8700Q/)
*   Ansprechen von Analog/Digital and Digital/Analog IC&#039;s, z.B. [PCF8591](http://developer.mbed.org/users/wim/notebook/pcf8591-i2c-4-channel-8-bit-ad-and-1-channel-8-bit/)
*   Verbinden von Boards, auch über grössere Distanzen. Siehe [www.mikrokontroller.net](http://www.mikrocontroller.net/articles/I%C2%B2C) und [RocNet](http://wiki.rocrail.net/doku.php?id=rocnet:rocnet-prot-de)

## Beispiele

**IoTKit K64F und DISCO_L475VG_IOT01A Board**

* [Capacitive digital sensor for relative humidity and temperature (HTS221)](#HTS221)
* [High-performance 3-axis magnetometer (LIS3MDL)](#LIS3MDL)
* [3D accelerometer and 3D gyroscope (LSM6DSL)](#LSM6DSL)

**IoTKit K64F Board**

* [Proximity sensor, gesture and ambient light sensing (ALS) module (VL6180X)](#vl6180x)

**DISCO_L475VG_IOT01A Board** - Anordnung der Sensoren siehe [Hardware](../hw#disco-l475vg-iot01a)

* [Time-of-Flight and gesture-detection sensor (VL53L0X)](#vl53l0x)

**Verfügbar auf dem IoTKit V3 (small)**

* [BMP180](#bmp180)
* [APDS9930](#APDS9930)

## HTS221
***

> [⇧ **Nach oben**](#beispiele)

Der HTS221 ist ein ultrakompakter Sensor für relative Feuchte und Temperatur. Es enthält ein Sensorelement und einen Mischsignal-ASIC, um die Messinformationen über digitale serielle Schnittstellen bereitzustellen.

Das Sensorelement besteht aus einer Polymer-Dielektrikum-Planar-Kondensatorstruktur, die in der Lage ist, Schwankungen der relativen Feuchtigkeit zu detektieren, und wird unter Verwendung eines speziellen ST-Prozesses hergestellt.

### Beispiel(e)

Das Beispiel [HTS221](main.cpp) gibt Temperatur und Luftfeuchtigkeit aus.

### Links

* [Produktseite](https://www.st.com/en/mems-and-sensors/hts221.html)

## LIS3MDL
***

> [⇧ **Nach oben**](#beispiele)

Der LIS3MDL ist ein ultra-low-power Hochleistungs-Drei-Achsen-Magnetsensor.

Der LIS3MDL hat vom Benutzer wählbare Vollskalen von ± 4 / ± 8 / ± 12 / ± 16 Gauss.

Die Vorrichtung kann konfiguriert werden, um Unterbrechungssignale für die Magnetfelderfassung zu erzeugen.

Der LIS3MDL enthält eine serielle I2C-Busschnittstelle, die Standard und Fast Mode (100 kHz und 400 kHz) und SPI serielle Standardschnittstelle unterstützt.

### Beispiel(e)

Das Beispiel LIS3MDL erkennt den Nord- oder Südpool eines Magneten und fungiert als einfacher eCompass welcher die Himmelsrichtungen anzeigt. 

<details><summary>main.cpp</summary>  

    #include "mbed.h"
    #include "lis3mdl_class.h"
    #include "OLEDDisplay.h"
    
    // UI
    OLEDDisplay oled( MBED_CONF_IOTKIT_OLED_RST, MBED_CONF_IOTKIT_OLED_SDA, MBED_CONF_IOTKIT_OLED_SCL );
    
    static DevI2C devI2c( MBED_CONF_IOTKIT_I2C_SDA, MBED_CONF_IOTKIT_I2C_SCL );
    static LIS3MDL magnetometer(&devI2c);
    
    int main()
    {
        uint8_t id;
        int32_t axes[3];
        float heading;
    
        /* Init all sensors with default params */
        magnetometer.init(NULL);
        magnetometer.read_id(&id);
        oled.clear();
        oled.printf("LIS3MDL = 0x%X\n", id);
    
        while (true)
        {
            oled.cursor( 1, 0 );
            magnetometer.get_m_axes( axes );
            printf( "LIS3DML [mag/mgauss] x=%6ld, y=%6ld, z=%6ld, diff=%6ld\n", axes[0], axes[1], axes[2], axes[0] + axes[1] );
    
            // Magnet mit North oder South anliegend
            if  ( axes[0] > 2000  )
            {
                oled.printf( "Magnet north" );
                continue;
            }
            if ( axes[0] < -2000 )
            {
                oled.printf( "Magnet south" );
                continue;
            }
    
    /*      Berechnung Winkel @see https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/coding (fehlerhaft)
            heading = atan2f(axes[1], axes[0]) * 180.0f / M_PI;
            heading = floorf( heading * 100.0f + 0.5f) / 100.0f;       // Rounds number to two decimal digits
            heading = (heading < 0.0f) ? (heading + 360.0f) : heading; // Change negative value to be in range <0,360)
    */
            // Vereinfachte Variante  eCompass: basiert auf Differenz axes[0]=East=x, axes=[1]=North=y
            int diff = axes[0] + axes[1];
            diff = (diff < 0) ? diff * -1 : diff;
            if  ( diff > 350 && diff < 370 )
                oled.printf( "north %6ld", diff );
            else if ( diff > 450 && diff < 470 )
                oled.printf( "west  %6ld", diff );
            else if ( diff > 50 && diff < 70 )
                oled.printf( "east  %6ld", diff );
            else if ( diff > 130 && diff < 150 )
                oled.printf( "south %6ld", diff );
            else
                oled.printf( "      %6ld", diff );
    
            thread_sleep_for( 500 );
        }
    }

</p></details>

### Links

* [Produktseite](https://www.st.com/en/mems-and-sensors/lis3mdl.html)

## LSM6DSL
 ***

> [⇧ **Nach oben**](#beispiele)

Das LSM6DSL ist ein System-in-Package mit einem digitalen 3D-Beschleunigungssensor und einem digitalen 3D-Gyroskop mit einer Leistung von 0,65 mA im Hochleistungsmode und ermöglicht durchgehend Low-Power-Funktionen für ein optimales Bewegungserlebnis für den Verbraucher.

Das LSM6DSL unterstützt die wichtigsten Betriebssystemanforderungen und bietet reale, virtuelle und Batch-Sensoren mit 4 kByte für die dynamische Datenaufzeichnung.

Die ST-Familie von MEMS-Sensormodulen nutzt die robusten und ausgereiften Fertigungsprozesse, die bereits für die Herstellung von mikrobearbeiteten Beschleunigungssensoren und Gyroskopen verwendet werden.

Das LSM6DSL verfügt über einen vollständigen Beschleunigungsbereich von ± 2 / ± 4 / ± 8 / ± 16 g und einen Winkelgeschwindigkeitsbereich von ± 125 / ± 245 / ± 500 / ± 1000 / ± 2000 dps.

Hohe Robustheit gegenüber mechanischen Schocks macht den LSM6DSL zur bevorzugten Wahl von Systementwicklern für die Herstellung und Herstellung von zuverlässigen Produkten.

### Beispiel(e)

#### LSM6DSL

Erkennt den jeweiligen Neigungswinkel und zeigt diesen auf dem Display an.

<details><summary>main.cpp</summary>  

    #include "mbed.h"
    #include "OLEDDisplay.h"
    #include "LSM6DSLSensor.h"
    
    // UI
    OLEDDisplay oled( MBED_CONF_IOTKIT_OLED_RST, MBED_CONF_IOTKIT_OLED_SDA, MBED_CONF_IOTKIT_OLED_SCL );
    
    static DevI2C devI2c( MBED_CONF_IOTKIT_I2C_SDA, MBED_CONF_IOTKIT_I2C_SCL );
    static LSM6DSLSensor acc(&devI2c,LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW); // low address
    
    int main()
    {
        uint8_t id;
        uint8_t xl = 0;
        uint8_t xh = 0;
        uint8_t yl = 0;
        uint8_t yh = 0;
        uint8_t zl = 0;
        uint8_t zh = 0;
        char report[256];
    
        oled.clear();
        oled.printf( "6D Orientation\n" );
    
        /* Init all sensors with default params */
        acc.init(NULL);
        acc.enable_g();
        acc.enable_x();
        acc.enable_6d_orientation();
    
        acc.read_id(&id);
        printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);
    
    
        while (true)
        {
            oled.clear();
    
            acc.get_6d_orientation_xl(&xl);
            acc.get_6d_orientation_xh(&xh);
            acc.get_6d_orientation_yl(&yl);
            acc.get_6d_orientation_yh(&yh);
            acc.get_6d_orientation_zl(&zl);
            acc.get_6d_orientation_zh(&zh);
    
            printf( "\nxl %d, xh %d, yl %d, yh %d, zl %d, zh %d\n", xl, xh, yl, yh, zl, zh );
    
            if ( xh )
            {
              sprintf( report, "    _____________\n" \
                               " * |_____________|\n" );
            }
    
            else if ( xl )
            {
              sprintf( report, " _____________\n" \
                               "|_____________| *\n" );
            }
            else if ( yh)
            {
              sprintf( report, " _____________\n" \
                               "|______v______|\n" );
            }
            else if ( yl )
            {
              sprintf( report, " _____________\n" \
                               "|______^______|\n" );
            }
            else if ( zh )
            {
              sprintf( report, " ______*______ \n" \
                               "|_____________|\n" );
            }
    
            else if ( zl )
            {
              sprintf( report,  " ____________\n" \
                                "|____________|\n" \
                                "      *       \n" );
            }
            else
            {
              sprintf( report, "None of the 6D orientation axes is set in LSM6DSL - accelerometer.\r\n" );
            }
            oled.printf( report );
            printf( (char*) report );
            thread_sleep_for( 1000 );
        }
    }

</p></details>

#### LSM6DSL_Pedometer

Zählt die Schritte und zeigt diese auf dem Display an.

<details><summary>main.cpp</summary>  

    /**
     * Pedometer
     * @see https://github.com/stm32duino/LSM6DSL/tree/master/examples
     */
    
    #include "mbed.h"
    #include "OLEDDisplay.h"
    #include "LSM6DSLSensor.h"
    
    // UI
    OLEDDisplay oled( MBED_CONF_IOTKIT_OLED_RST, MBED_CONF_IOTKIT_OLED_SDA, MBED_CONF_IOTKIT_OLED_SCL );
    
    static DevI2C devI2c( MBED_CONF_IOTKIT_I2C_SDA, MBED_CONF_IOTKIT_I2C_SCL );
    static LSM6DSLSensor acc_gyro(&devI2c,LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW); // low address
    uint16_t step_count = 0;
    
    int main()
    {
        uint8_t id;
        LSM6DSL_Event_Status_t status;
    
        oled.clear();
        oled.printf( "Pedometer\n" );
    
        acc_gyro.init(NULL);
        acc_gyro.enable_x();
        acc_gyro.enable_pedometer();
    
        acc_gyro.read_id(&id);
        printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);
    
        while (true) 
        {
            acc_gyro.get_event_status( &status );
            if  ( status.StepStatus )
            {
                step_count++;
                oled.cursor( 1, 0 );
                
                oled.printf( "steps %6d\n", step_count );
            }
            thread_sleep_for( 1000 );
        }
    }

</p></details>

#### LSM6DSL_Tilt

Erkennt wenn das Board schief gehalten wird.

<details><summary>main.cpp</summary>  

    /**
     * Tilt
     * @see https://github.com/stm32duino/LSM6DSL/tree/master/examples
     */
    
    #include "mbed.h"
    #include "OLEDDisplay.h"
    #include "LSM6DSLSensor.h"
    
    // UI
    OLEDDisplay oled( MBED_CONF_IOTKIT_OLED_RST, MBED_CONF_IOTKIT_OLED_SDA, MBED_CONF_IOTKIT_OLED_SCL );
    
    static DevI2C devI2c( MBED_CONF_IOTKIT_I2C_SDA, MBED_CONF_IOTKIT_I2C_SCL );
    static LSM6DSLSensor acc_gyro( &devI2c, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW ); // low address
    uint16_t tap_count = 0;
    uint16_t tilt_count = 0;
    
    int main()
    {
        uint8_t id;
        LSM6DSL_Event_Status_t status;
    
        oled.clear();
        oled.printf( "Schieflage\n" );
    
        acc_gyro.init(NULL);
        acc_gyro.enable_x();    
        acc_gyro.enable_g();
        //acc_gyro.enable_single_tap_detection();
        acc_gyro.enable_tilt_detection();
    
        acc_gyro.read_id(&id);
        printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);
    
        while (true) {
            //acc_gyro.get_event_status( &status );
            //if  ( status.TapStatus ) {
            //    tap_count++;
            //    oled.cursor( 1, 0 );
            //    oled.printf( "tap %6d\n", tap_count );
            //    printf( "tap %6d\n", tap_count );
            //}
            acc_gyro.get_event_status( &status );
            if  ( status.TiltStatus ) {
                tilt_count++;
                oled.cursor( 2, 0 );
                oled.printf( "tilt %6d\n", tilt_count );
                printf( "tilt %6d\n", tilt_count );
            }
            thread_sleep_for( 100 );
        }
    }

</p></details>

#### LSM6DSL_SingleTap

Erkennt wenn jemand auf das Board tippt.

<details><summary>main.cpp</summary>  

    /**
     * Tap Status
     * @see https://github.com/stm32duino/LSM6DSL/tree/master/examples
     */
    
    #include "mbed.h"
    #include "OLEDDisplay.h"
    #include "LSM6DSLSensor.h"
    
    // UI
    OLEDDisplay oled( MBED_CONF_IOTKIT_OLED_RST, MBED_CONF_IOTKIT_OLED_SDA, MBED_CONF_IOTKIT_OLED_SCL );
    
    static DevI2C devI2c( MBED_CONF_IOTKIT_I2C_SDA, MBED_CONF_IOTKIT_I2C_SCL );
    static LSM6DSLSensor acc_gyro( &devI2c, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW ); // low address
    uint16_t tap_count = 0;
    
    int main()
    {
        uint8_t id;
        LSM6DSL_Event_Status_t status;
    
        oled.clear();
        oled.printf( "Schlagen\n" );
    
        acc_gyro.init(NULL);
        acc_gyro.enable_x();    
        acc_gyro.enable_g();
        acc_gyro.enable_single_tap_detection();
    
        acc_gyro.read_id(&id);
        printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);
    
        while (true) {
            acc_gyro.get_event_status( &status );
            if  ( status.TapStatus ) {
                tap_count++;
                oled.cursor( 1, 0 );
                oled.printf( "tap %6d\n", tap_count );
                printf( "tap %6d\n", tap_count );
            }
            thread_sleep_for( 100 );
        }
    }

</p></details>

### Links

* [Produktseite](https://www.st.com/en/mems-and-sensors/hts221.html)
* [Weitere Beispiele](https://github.com/stm32duino/LSM6DSL/tree/master/examples)


## VL6180X
***

> [⇧ **Nach oben**](#beispiele)

Der VL6180X ist das neueste Produkt, das auf der patentierten FlightSense ™ -Technologie von ST basiert . Dies ist eine bahnbrechende Technologie, die es ermöglicht, die absolute Entfernung unabhängig von der Zielreflektion zu messen. Anstatt die Entfernung durch Messung der vom Objekt reflektierten Lichtmenge zu messen (die maßgeblich von Farbe und Oberfläche beeinflusst wird), misst der VL6180X genau die Zeit, die das Licht braucht, um zum nächsten Objekt zu gelangen und zum Sensor zurückzusenden (Zeit Flug).

Der VL6180X kombiniert einen IR-Strahler, einen Bereichssensor und einen Umgebungslichtsensor in einem gebrauchsfertigen 3-in-1-Reflow-Gehäuse. Der VL6180X ist einfach zu integrieren und erspart dem Endprodukthersteller lange und kostspielige optische und mechanische Designoptimierungen.

Das Modul ist für den Betrieb mit geringer Leistung ausgelegt. Entfernungs- und ALS-Messungen können automatisch in benutzerdefinierten Intervallen durchgeführt werden. Mehrere Schwellenwert- und Interrupt-Schemata werden unterstützt, um Host-Operationen zu minimieren.


### Beispiel(e)

Das Beispiel VL6180X zeigt die Lichtstärke in LUX und die Entfernung eines Gegenstandes, z.B. der Hand, zum VL6180X an.

<details><summary>main.cpp</summary>  

    /**
     * Infrarot Abstand- und Licht-Sensor
     *
     * Abstand-Sensor geht offiziell bis 100 mm
     */
    
    #include "mbed.h"
    #include <VL6180x.h>
    
    #define VL6180X_ADDRESS 0x52
    
    VL6180xIdentification identification;
    // mbed uses 8bit addresses shift address by 1 bit left
    VL6180x sensor( PTE0, PTE1, VL6180X_ADDRESS);
    // enable, verhindert Init Fehler
    DigitalOut sensor_ce( PTE3 );
    
    void printIdentification(struct VL6180xIdentification *temp)
    {
        printf("Model ID = ");
        printf("%d\n",temp->idModel);
    
        printf("Model Rev = ");
        printf("%d",temp->idModelRevMajor);
        printf(".");
        printf("%d\n",temp->idModelRevMinor);
    
        printf("Module Rev = ");
        printf("%d",temp->idModuleRevMajor);
        printf(".");
        printf("%d\n",temp->idModuleRevMinor);
    
        printf("Manufacture Date = ");
        printf("%d",((temp->idDate >> 3) & 0x001F));
        printf("/");
        printf("%d",((temp->idDate >> 8) & 0x000F));
        printf("/1");
        printf("%d\n",((temp->idDate >> 12) & 0x000F));
        printf(" Phase: ");
        printf("%d\n",(temp->idDate & 0x0007));
    
        printf("Manufacture Time (s)= ");
        printf("%d\n",(temp->idTime * 2));
        printf("\n\n");
    }
    
    int main()
    {
        sensor_ce = 1;
        sensor.getIdentification(&identification); // Retrieve manufacture info from device memory
        printIdentification(&identification); // Helper function to print all the Module information
    
        if(sensor.VL6180xInit() != 0) {
            printf("FAILED TO INITALIZE\n"); //Initialize device and check for errors
        };
        sensor.VL6180xDefautSettings(); //Load default settings to get started.
    
        thread_sleep_for(1000); // delay 1s
    
        while(1) 
        {
            //Get Ambient Light level and report in LUX
            printf("Ambient Light Level (Lux) = %f\n",sensor.getAmbientLight(GAIN_1) );
    
            //Get Distance and report in mm
            printf("Distance measured (mm) = %d\n", sensor.getDistance() );
    
            thread_sleep_for( 500 );
        }
    }

</p></details>

### Links

* [Produktseite](https://www.st.com/en/imaging-and-photonics-solutions/vl6180x.html)

## VL53L0X
***

> [⇧ **Nach oben**](#beispiele)

Das VL53L0X ist ein Time-of-Flight (ToF)  -Laser-Entfernungsmodul der neuen Generation.

Es bietet eine genaue Abstandsmessung unabhängig von den Zielreflexionen im Gegensatz zu herkömmlichen Technologien. 

Es misst absolute Distanzen bis zu 2 m und setzt damit neue Maßstäbe in Bezug auf das Leistungsspektrum und öffnet die Tür für verschiedene neue Anwendungen.

### Beispiel(e)

Das Beispiel VL53L0X zeigt die Entfernung eines Gegenstandes, z.B. der Hand, zum VL53L0X an.

<details><summary>main.cpp</summary>  

    /** Veraendert die Helligkeit je nach Abstand von VL53L0X
     */
    #include "mbed.h"
    
    // DigitalIn button( USER_BUTTON1 );
    // DigitalOut buzzer( D3 );
    
    // Sensoren
    #include "VL53L0X.h"
    
    DevI2C devI2c( PB_11, PB_10 );
    DigitalOut shutdown_pin( PC_6 );
    VL53L0X range( &devI2c, &shutdown_pin, PC_7 );
    PwmOut led1( LED2 );
    
    /**
     *  Hauptprogramm
     */
    int main()
    {
        range.init_sensor( VL53L0X_DEFAULT_ADDRESS );
        led1 = 0.0f;
    
        while ( true )
        {
            uint32_t distance;
            int status = range.get_distance( &distance );
            // Methode macht keinen Check ob Sensore Ready, deshalb nur gueltige Werte auswerten
            if ( status == VL53L0X_ERROR_NONE )
            {
                printf( "VL53L0X [mm]:            %6ld\r\n", distance );
                led1 = distance * 0.0005;
    
            }
            else
            {
                printf( "VL53L0X [mm]:                --\r\n" );
            }
            thread_sleep_for( 100 );
        }
    }

</p></details>

### Links

* [Produktseite](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html)

## BMP180
***

Der BMP180 Präzisionssensor von Bosch ist die beste kostengünstige (~60cent) Messlösung zur Messung von Luftdruck und Temperatur.

Der BMP180 liefert Temperatur und Luftdruck zurück.

Für Details siehe [Datenblatt](https://www.mouser.ch/datasheet/2/783/BST-BMP180-DS000-1509579.pdf).

### Anwendungen 

* Überwachung von Temperatur und Luftdruck
* Kann auch als Höhenmesser verwendet werden


### Beispiel(e)

**Compilieren**

| Umgebung/Board    | Link/Befehl                      |
| ----------------- | -------------------------------- |
| Online Compiler | [BMP180Sensor](https://os.mbed.com/compiler/#import:/teams/IoTKitV3/code/BMP180Sensor/) |
| CLI (nucleo_f303re) | `mbed compile -m nucleo_f303re -f --source . --source ../IoTKitV3/i2c/BMP180` |


## ADPS9930
***

Der APDS-9930 von [Broadcom](https://www.digikey.ch/de/supplier-centers/b/broadcom) ist ein digitaler, I²C-kompatibler Umgebungslichtsensor (ALS) und Näherungssensor mit IR-LED.

Der APDS-9930 kann Objekte bis 100mm erfassen. Er erkennt Bewegungen links, rechts, nah, fern.

Für Details siehe [Datenblatt](http://www.makerfabs.com/desfile/files/APDS-9930_2015-11-13.pdf).

### Anwendungen 

* Erfassung des Menschlichen Auges via Smartphone
* Erkennung von [Gesten](https://www.youtube.com/watch?v=A3QRyixnEl8).

### Beispiel(e)


| Umgebung/Board    | Link/Befehl                      |
| ----------------- | -------------------------------- |
| Online Compiler | folgt |
| CLI (nucleo_f303re) | folgt |

## Übungen
***

> [⇧ **Nach oben**](#beispiele)

| Übung                     | Lösung       |
| ------------------------- | ------------ |
| **6D Lagesensor und Servo** steuert mit dem 6 Lagensensor die Position des Servos<br>**Anwendungen**: Ausgleich der Lage z.B. bei einem Zug. | [Lösung](01-Uebung/src/main.cpp) |
| **6D Lagsensor und Schrittmotor** bringt mit dem 6 Lagesensor den ersten oder zweiten Schrittmotor vor- oder rückwärts zum laufen. | [Lösung](02-Uebung/src/main.cpp) |
| **Abstandssensor und Motor** Motor Geschwindigkeit anhand des Abstandes zum Abstandssensor setzen. Wie näher desto schneller. | [Lösung](03-Uebung/src/main.cpp) | 

