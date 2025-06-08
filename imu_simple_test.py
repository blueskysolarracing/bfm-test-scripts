from periphery import I2C,GPIO
from time import sleep

ID_REG = 0x00
CHIP_ID = 0xA0
BNO055_ADDR = 0x29 #COM3 is connected to ground
OPR_MODE_REG = 0x3D #register for operation mode
NDOF_MODE = 0x0C #  
UNIT_SEL_REG = 0x3B
UNIT_MODE = 0x00 
ACC_DATA_X_LSB = 0x08
MAG_DATA_X_LSB = 0x0E
GYR_DATA_X_LSB = 0x14
EUL_DATA_X_LSB = 0x1A
QUA_DATA_W_LSB = 0x20
LIA_DATA_X_LSB = 0x28
GRV_DATA_X_LSB = 0x2E
TEMP = 0x34
CALIB_STAT_REG = 0x35
PAGE_ID_REG = 0x07


i2c = I2C("/dev/i2c-4") #run i2cdetect -l to find available i2c bus, this case shoudl be 3? CONFIRM
gpio_out_imu_reset = GPIO("/dev/gpiochip4",21,"out") #SENSOR_IMU_RST pin on toradex
#gpio_out_imu_reset.write(True)
#sleep(1)

def reset():
    gpio_out_imu_reset.write(False)
    sleep(0.05)
    gpio_out_imu_reset.write(True)
    sleep(1)

def select_page(page):
    try:
        msg = I2C.Message([PAGE_ID_REG, page], read=False)
        i2c.transfer(BNO055_ADDR, [msg])
        sleep(0.01)  # Add a small delay for the page selection to take effect
    except Exception as e:
        print(f"Page Select Error: {e}")

def read_register(register, length=1):
        try:
            write_msg = I2C.Message([register], read=False)
            read_msg = I2C.Message([0x00]*length, read=True)
            i2c.transfer(BNO055_ADDR, [write_msg, read_msg])
            return read_msg.data
        except IOError as e:
            print(f"I2c Read Erro: {e}")

def write_register(register, data):
        try:
            msg = I2C.Message([register, data], read=False)
            i2c.transfer(BNO055_ADDR, [msg])
        
        except Exception as e:
            print(f"I2C Write Error: {e}")

def set_op_mode():
    #page 22 for all possible operation modes
    write_register(OPR_MODE_REG, NDOF_MODE)
    sleep(0.05)

def set_units():
    # Set acceleration to m/s^2, angular rate to dps, Euler angles to degrees, temp to Celsius
    write_register(UNIT_SEL_REG, UNIT_MODE)
    sleep(0.05)
    
def verify_config():
    select_page(0)
    print(read_register(PAGE_ID_REG))
    chip_id =read_register(ID_REG)[0] 
    print(chip_id, chip_id== CHIP_ID)
    
    set_units()
    units = read_register(UNIT_SEL_REG)[0]
    print(units,units==UNIT_MODE)
    
    #set_op_mode()
    op_mode = read_register(OPR_MODE_REG)[0]&0x0F
    print(op_mode, op_mode==NDOF_MODE)
    return chip_id == CHIP_ID and op_mode == NDOF_MODE and units == UNIT_MODE

reset()
print(verify_config())
reset()
