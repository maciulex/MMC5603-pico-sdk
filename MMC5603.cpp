#ifndef MMC5603_F
#define MMC5603_F

//https://cdn-learn.adafruit.com/assets/assets/000/113/957/original/MMC5603NJ_RevB_7-12-18.pdf?1659554945

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

namespace MMC5603
{
    i2c_inst_t *I2C_PORT = i2c0;
    const uint boundrate   = 10*1000; 
    const uint8_t I2C_PIN_SDA = 0;
    const uint8_t I2C_PIN_SCL = 1;
    const uint8_t ADDRESS = 0x30;
    
    bool BLOCKING_LIMIT = true;
    const uint8_t BLOCKING_MAX_TRIES = 254;
    const uint16_t TIME_BEETWEEN_STATUS_REQUEST_IN_BLOCKING_US = 1000;

    const uint16_t MINIMUM_DELAY_BETWEEN_COMMANDS_I2C_US = 100 ;

    struct DATA
    {
        uint8_t rawDataX[3];
        uint8_t rawDataY[3];
        uint8_t rawDataZ[3];

        int32_t data32bitX;
        int32_t data32bitY;
        int32_t data32bitZ;

        bool axisX_active = true;
        bool axisY_active = true;
        bool axisZ_active = true;


        uint8_t rawTemp[1];
        float temp;
    };

    DATA magnetometrData;

    struct {
        const uint8_t X_OUT0 = 0x00;
        const uint8_t X_OUT1 = 0x01;
        const uint8_t X_OUT2 = 0x06;

        const uint8_t Y_OUT0 = 0x02;
        const uint8_t Y_OUT1 = 0x03;
        const uint8_t Y_OUT2 = 0x07;

        const uint8_t Z_OUT0 = 0x04;
        const uint8_t Z_OUT1 = 0x05;
        const uint8_t Z_OUT2 = 0x08;

        const uint8_t T_OUT = 0x09;
 
        const uint8_t C0 = 0x1B;
        const uint8_t C1 = 0x1C;
        const uint8_t C2 = 0x1D;

        const uint8_t ODR    = 0x1A;
        const uint8_t STATUS = 0x18;

        const uint8_t ID = 0x39;
    } REGISTERS;

    //INTERNAL CONTORL 0
    enum TAKE_MEASURMENT                {TAKE_MAGNETIC           = 0b0000'0001, TAKE_TEMPERATURE   = 0b0000'0010, TAKE_TEMP_MAGN             = 0b0000'0011, TAKE_NONE   = 0b0000'0000};
    enum SET_RESET                      {DO_RESET                = 0b0001'0000, DO_SET             = 0b0000'1000, DO_SET_RESET               = 0b0001'1000, DO_NOTHING  = 0b0000'0000};
    enum AUTO_SR_SELF_TEST              {SET_PERIODIC_MEASURMENT = 0b0010'0000, SET_AUTO_SELF_TEST = 0b0100'0000, SET_PERIODIC_AND_AUTO_TEST = 0b0110'0000, SET_NOTHING= 0b0000'0000};
    //Writing a 1 into this location will start the calculation of the measurement period according to the ODR. This bit should be set before continuous-mode measurements are started. This bit is selfcleared after the measurement period is calculated by internal circuits. 
    enum CALCULATION_OF_MEASURMENT_TIME {CALCULATE_TIME          = 0b1000'0000, DO_NOT_CALCULATE   = 0b0000'0000};

    //INTERNAL CONTORL 1
    enum BANDWITH_SELECTION {ms6p6     = 0b0000'0000, ms3p5         = 0b0000'0001, ms2p0  = 0b0000'0010, ms1p2 = 0b0000'0011};
    enum ACTIVATE_AXIS      {XYZ       = 0b0000'0000, XY            = 0b0001'0000, XZ     = 0b0000'1000, YZ    = 0b0000'0100, X = 0b0001'1000, Y = 0b0001'0100, Z = 0b0000'1100};
    enum DC_FOR_SELF_TEST   {DC_NORMAL = 0b0010'0000, DC_INVERSE    = 0b0100'0000, DC_NONE= 0b0000'0000};
    enum SOFTWARE_RESET     {DO_S_RESET= 0b1000'0000, DO_NOT_S_RESET= 0b0000'0000};

    //INTERNAL CONTORL 2
    enum PERIODICAL_MEASURMENTS  {p1 = 0b0000'0000, p25 = 0b0000'0001, p75 = 0b0000'0010, p100 = 0b0000'0011, p250 = 0b0000'0100, p500 = 0b0000'0101, p1000 = 0b0000'0110, p2000 = 0b0000'0111};
    enum ACTIVATE_PERIODICAL_MES {ACTIVATE_PERIODICAL  = 0b0000'1000, DISABLE_PERIODICAL      = 0b0000'0000};
    enum ENTER_CONTINOUS_MODE    {ACTIVATE_CONTINOUS   = 0b0001'0000, DISABLE_CONTINOUS       = 0b0000'0000};
    enum HIGHT_POWER_MODE        {ACTIVATE_HI_POW      = 0b1000'0000, DISABLE_HI_POW          = 0b0000'0000};

    //ODR  | BW == BANDWITH_SELECTION
    //BW | Automatic SET/RESET | No SET/RESET | ODR range
    //00 | 75Hz                 | 150 Hz      | 1~75
    //01 | 150 Hz               | 255 Hz      | 1~150
    //10 | 255 Hz               | 255 Hz      | 1~255
    //11 | 255Hz hpower=0: 255 Hz; hpower=1: 1000 Hz 1~255

    enum BASIC_MODES {MAX_UPDATE, FAST_UPDATE, SLOW_UPDATE, MANUAL_UPDATE};
    
    void getAll(bool blocking = true);
    void getTemp(bool blocking = true, uint8_t statusReg = 255);
    void getAllAxis(bool blocking = true, uint8_t statusReg = 255);
    void getX();
    void getY();
    void getZ();
    void getID();
   
    void printActiveAxis();
    void printTemp();

    void setBasicSettings(BASIC_MODES mode);
    void SET_ODR(uint8_t odr);
    void SET_INTERNAL_CONTROL_0(TAKE_MEASURMENT t, SET_RESET r, AUTO_SR_SELF_TEST a, CALCULATION_OF_MEASURMENT_TIME c);
    void SET_INTERNAL_CONTROL_1(BANDWITH_SELECTION b, ACTIVATE_AXIS a, DC_FOR_SELF_TEST d, SOFTWARE_RESET s);
    void SET_INTERNAL_CONTROL_2( PERIODICAL_MEASURMENTS p, ACTIVATE_PERIODICAL_MES a, ENTER_CONTINOUS_MODE c, HIGHT_POWER_MODE h);

    bool I2C_TALK(uint8_t *src, uint8_t *dst, uint8_t size);
    bool I2C_ORDER(uint8_t *order, uint8_t size);
    
    void initI2C();
    void init(bool i2c = true);
    
    bool I2C_ORDER(uint8_t *order, uint8_t size) {
        for (int i = 0; i < size; i+=2) {
            i2c_write_blocking(I2C_PORT, ADDRESS, &order[i], 2, false);
            sleep_us(MINIMUM_DELAY_BETWEEN_COMMANDS_I2C_US);
        }
        return true;
    }

    bool I2C_TALK(uint8_t *src, uint8_t *dst, uint8_t size) {
        for (int i = 0; i < size; i++) {
            i2c_write_blocking(I2C_PORT, ADDRESS, &src[i], 1, true);
            i2c_read_blocking (I2C_PORT, ADDRESS, &dst[i], 1, false);
            sleep_us(MINIMUM_DELAY_BETWEEN_COMMANDS_I2C_US);
        }
        return true;
    }

    void setBasicSettings(BASIC_MODES mode) {
        switch (mode) {
            case BASIC_MODES::MAX_UPDATE:

            break;
            case BASIC_MODES::FAST_UPDATE:
                SET_ODR(125);
                SET_INTERNAL_CONTROL_1(            
                    BANDWITH_SELECTION::ms2p0,
                    ACTIVATE_AXIS     ::XYZ,
                    DC_FOR_SELF_TEST  ::DC_NONE,
                    SOFTWARE_RESET    ::DO_NOT_S_RESET);

                SET_INTERNAL_CONTROL_0(
                    TAKE_MEASURMENT               ::TAKE_TEMP_MAGN,
                    SET_RESET                     ::DO_NOTHING,
                    AUTO_SR_SELF_TEST             ::SET_AUTO_SELF_TEST,
                    CALCULATION_OF_MEASURMENT_TIME::CALCULATE_TIME);

                SET_INTERNAL_CONTROL_2(            
                    PERIODICAL_MEASURMENTS  ::p1,
                    ACTIVATE_PERIODICAL_MES ::DISABLE_PERIODICAL,
                    ENTER_CONTINOUS_MODE    ::ACTIVATE_CONTINOUS,
                    HIGHT_POWER_MODE        ::ACTIVATE_HI_POW);
                printf("SET FAST MODE\n");
            break;
            case BASIC_MODES::SLOW_UPDATE:

            break;
            case BASIC_MODES::MANUAL_UPDATE:

            break;
        }
    }

    void SET_ODR(uint8_t odr) {
        uint8_t order[2] {REGISTERS.ODR, odr};
        I2C_ORDER(order, 2);
    }

    void SET_INTERNAL_CONTROL_0(
            TAKE_MEASURMENT                 t,
            SET_RESET                       r,
            AUTO_SR_SELF_TEST               a,
            CALCULATION_OF_MEASURMENT_TIME  c
        ) {
        uint8_t order[2] {REGISTERS.C0, (t | r | a | c)};
        I2C_ORDER(order, 2);
    }

    void SET_INTERNAL_CONTROL_1(
            BANDWITH_SELECTION b,
            ACTIVATE_AXIS      a,
            DC_FOR_SELF_TEST   d,
            SOFTWARE_RESET     s
        ) {

        if (a & 0b0000'0100) magnetometrData.axisX_active = false;
        else magnetometrData.axisX_active = true;

        if (a & 0b0000'1000) magnetometrData.axisY_active = false;
        else magnetometrData.axisY_active = true;
        
        if (a & 0b0001'0000) magnetometrData.axisZ_active = false;
        else magnetometrData.axisZ_active = true;

        if (s) magnetometrData = {};

        uint8_t order[2] {REGISTERS.C1, (b | a | d | s)};
        I2C_ORDER(order, 2);
    }

    void SET_INTERNAL_CONTROL_2(
            PERIODICAL_MEASURMENTS   p,
            ACTIVATE_PERIODICAL_MES  a,
            ENTER_CONTINOUS_MODE     c,
            HIGHT_POWER_MODE         h
        ) {
        uint8_t order[2] {REGISTERS.C2, (p | a | c | h)};
        I2C_ORDER(order, 2);
    }

    void getX() {
        uint8_t message[3] {REGISTERS.X_OUT0, REGISTERS.X_OUT1, REGISTERS.X_OUT2};
        I2C_TALK(message, magnetometrData.rawDataX, 3);

        magnetometrData.data32bitX = ((magnetometrData.rawDataX[0] << 12) | (magnetometrData.rawDataX[1] << 4) | (magnetometrData.rawDataX[2] >> 4));
        magnetometrData.data32bitX -= (uint32_t)1 << 19;
    }

    void getY() {
        uint8_t message[3] {REGISTERS.Y_OUT0, REGISTERS.Y_OUT1, REGISTERS.Y_OUT2};
        I2C_TALK(message, magnetometrData.rawDataY, 3);

        magnetometrData.data32bitY = ((magnetometrData.rawDataY[0] << 12) | (magnetometrData.rawDataY[1] << 4) | (magnetometrData.rawDataY[2] >> 4));
        magnetometrData.data32bitY -= (uint32_t)1 << 19;
    }

    void getZ() {
        uint8_t message[3] {REGISTERS.Z_OUT0, REGISTERS.Z_OUT1, REGISTERS.Z_OUT2};
        I2C_TALK(message, magnetometrData.rawDataZ, 3);
         
        magnetometrData.data32bitZ = ((magnetometrData.rawDataZ[0] << 12) | (magnetometrData.rawDataZ[1] << 4) | (magnetometrData.rawDataZ[2] >> 4));
        magnetometrData.data32bitZ -= (uint32_t)1 << 19;
    }
    
    uint8_t getStatusRegister() {
        uint8_t message[1] {REGISTERS.STATUS}, result[1];

        I2C_TALK(message, result, 1);

        return result[0];
    }

    void getTemp(bool blocking, uint8_t statusReg) {
        if (blocking) {
            if (statusReg == 255) statusReg = getStatusRegister();
            uint8_t counter = 0;
            while (counter < BLOCKING_MAX_TRIES) {
                if (statusReg & 0b1000'0000) {
                    printf("MEASURMENT READY T\n");
                    break;
                }

                sleep_us(TIME_BEETWEEN_STATUS_REQUEST_IN_BLOCKING_US);
                statusReg = getStatusRegister();

                if (BLOCKING_LIMIT) counter++;
            }
            if (!(counter < BLOCKING_MAX_TRIES)) {
                printf("BLOCKING TIMEOUT\n");
            }
        }

        uint8_t message[1] {REGISTERS.T_OUT};
        I2C_TALK(message, magnetometrData.rawTemp, 1);
        magnetometrData.temp = -75 + (magnetometrData.rawTemp[0]*0.8);
    }

    void getAllAxis(bool blocking, uint8_t statusReg) {
        if (blocking) {
            if (statusReg == 255) statusReg = getStatusRegister();
            uint8_t counter = 0;
            while (counter < BLOCKING_MAX_TRIES) {
                if (statusReg & 0b0100'0000) {
                    printf("MEASURMENT READY M\n");
                    break;
                }
                sleep_us(TIME_BEETWEEN_STATUS_REQUEST_IN_BLOCKING_US);
                statusReg = getStatusRegister();

                if (BLOCKING_LIMIT) counter++;
            }
            if (!(counter < BLOCKING_MAX_TRIES)) {
                printf("BLOCKING TIMEOUT\n");
            }
        }

        if (magnetometrData.axisX_active) getX();
        if (magnetometrData.axisY_active) getY();
        if (magnetometrData.axisZ_active) getZ();
    }
        
    void getAll(bool blocking) {
        getAllAxis(blocking);
        getTemp(blocking);
    }

    //can be used to test if communication is working expected output D16 aka 0b0001'0000, 0x10
    void getID() {
        uint8_t message[1] {REGISTERS.ID};
        uint8_t result [1];
        I2C_TALK(message, result, 1);
        printf("ID: %i\n", result[0]);
    }

    void printActiveAxis() {
        printf("Axis: \n");
        if (magnetometrData.axisX_active) printf("\tAxis x: %f\n", ((float)magnetometrData.data32bitX * 0.00625));
        if (magnetometrData.axisY_active) printf("\tAxis y: %f\n", ((float)magnetometrData.data32bitY * 0.00625));
        if (magnetometrData.axisZ_active) printf("\tAxis z: %f\n", ((float)magnetometrData.data32bitZ * 0.00625));
        printf("\n");
    }

    void printTemp() {
        printf("Temp: %.2fC\n\n", magnetometrData.temp);
    }

    void initI2C() {
        i2c_init(I2C_PORT, boundrate);

        gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
        gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_PIN_SDA);
        gpio_pull_up(I2C_PIN_SCL);
    }

    void init(bool i2c) {
        if (i2c) initI2C();
    }

} // namespace MMC5603



#endif