#/2424/2424/2424/2424page 55 register map, power on is page 0 register map, can switch by toggling PAGE_ID register
#page 99 interface: select standard i2c interface ps1 0 (pin5), ps0 0 (pin6)
#page 100 i2c protocal: bno055 default i2c address when input pin com3 (pin 17) HIGH is 0x29, if LOW then 0x28


#additional
#axis remap if mounted not in default orientation,  offest to calibrate errors

#for calibration check out page 51

#questions:
#configure/check sda, scl on device tree?

#toradex pinout spreadsheet imu output: toradex pinout 203 SENSOR_I2C3_SDA, 201 SENSOR_I2C3_SCL
'''
i2c3_sda_pin = 203
i2c3_scl_pin = 201
'''

from periphery import I2C, GPIO
from time import sleep
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

i2c = I2C("/dev/i2c-4") #run i2cdetect -l to find available i2c bus, this case shoudl be 3? CONFIRM

SYS_TRIG_REG = 0x3F
#7th bit is rst int


#gpio_out_imu_reset = GPIO("/dev/gpiochip4",21,"out") #SENSOR_IMU_RST pin on toradex
BNO055_ADDR = 0x29 #COM3 is connected to ground
OPR_MODE_REG = 0x3D #register for operation mode
NDOF_MODE = 0x0C #
CONFIG_MODE = 0x00
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
PWR_MODE_REG = 0x3E
NORMAL_POWER_MODE = 0x00
SYS_CLK_STAT_REG = 0x38


# def write_register(register, data):
#     #try:
#         msg = I2C.Message([register, data], read=False)
#         i2c.transfer(BNO055_ADDR, [msg])
#     #except IOError as e:
#     #    logger.error(f"I2C write error: {e}")
#     #    raise
# 
# def read_register(register, length=1):
# 
#     for i in range(5):
#         try:
#             #val = 0b01000000
#             #write_register(SYS_TRIG_REG,val)
# 
#             write_msg = I2C.Message([register], read=False)
#             read_msg = I2C.Message([0x00]*length, read=True)
#             i2c.transfer(BNO055_ADDR, [write_msg, read_msg])
# 
#             return read_msg.data
#         except IOError as e:
#             logger.error(f"register {register} I2C read error: {e}")
#             pass
# 
# 
# def close():
#     i2c.close()
#     #gpio_out_imu_reset.close()
# 
# '''
# def reset():
#     gpio_out_imu_reset.write(False)
#     sleep(0.5)
#     gpio_out_imu_reset.write(True)
#     logger.info("reset")
#     sleep(1)
# '''
# def reset():
#      write_register(SYS_TRIG_REG,0x20)
#      sleep(3)
#      
# 
# def set_op_mode(mode):
#     #page 22 for all possible operation modes
#     write_register(OPR_MODE_REG, mode)
#     sleep(0.1)
#     logger.info("operation mode set")
# 
# def set_power_mode(mode):
#     write_register(PWR_MODE_REG, mode)
#     sleep(0.05)
#     logger.info("operation mode set")
# 
# def set_units():
#     # Set acceleration to m/s^2, angular rate to dps, Euler angles to degrees, temp to Celsius
#     write_register(UNIT_SEL_REG, UNIT_MODE)
#     sleep(0.05)
#     logger.info("units set")
# 
# def verify_config():
#     mode = read_register(OPR_MODE_REG, 1)[0] & 0x0F #lower 4 bits of OPR_MODE_REG represent the mode
#     units = read_register(UNIT_SEL_REG, 1)[0]
#     logger.info(f"Current mode: {mode}, Units config: {units}")
#     return mode == NDOF_MODE and units == UNIT_MODE
# 
# def check_calibration():
#     calib_status = read_register(CALIB_STAT_REG, 1)[0]
#     sys_calib = (calib_status >> 6) & 0x03
#     gyro_calib = (calib_status >> 4) & 0x03
#     accel_calib = (calib_status >> 2) & 0x03
#     mag_calib = calib_status & 0x03
#     logger.info(f"Calibration - Sys: {sys_calib}/3, Gyro: {gyro_calib}/3, Accel: {accel_calib}/3, Mag: {mag_calib}/3")
#     return True #sys_calib == 3 and gyro_calib == 3 and accel_calib == 3 and mag_calib == 3
# 
# def set_external_crystal(use_external_crystal=True):
#     write_register(OPR_MODE_REG,0x00) #config mode
#     sleep(1)
#     write_register(0x07,0) #page number 
#     if use_external_crystal:
#         write_register(SYS_TRIG_REG,0x80)
#     else:
#         write_register(SYS_TRIG_REG,0x00)
#     sleep(1)
#     while (read_register(SYS_CLK_STAT_REG,1)[0] & 0x01):
#          sleep(0.1)
#     if not (read_register(SYS_TRIG_REG,1)[0] & 0x80):
#          raise Exception("EXTERNAL CRYSTAL BROKEN")
#          
# 
#     write_register(OPR_MODE_REG,NDOF_MODE)
#     sleep(1)
#     print("DONE SETTING  CRYST")
# 
# 
#      
# 
# def initialize():
#     #try:
# 
#         chip_id = read_register(0x00)
#         logger.info(f"chip id: {str(chip_id)}") #should be 0xA0
#         set_op_mode(CONFIG_MODE)
#         reset()
#         sleep(1)  # Wait for reset to complete
#         chip_id = read_register(0x00)
#         logger.info(f"chip id: {str(chip_id)}") #should be 0xA0
#         set_units()
#         set_power_mode(NORMAL_POWER_MODE)
# 
#         write_register(0x07,0) #page 0
#         write_register(SYS_TRIG_REG,0) #self test
#         sleep(0.01)
#         set_op_mode(NDOF_MODE)
# 
#         #setting power mode and external crystal?
#         if not verify_config():
#             raise Exception("Failed to set correct mode or units")
#         logger.info("Initialization complete. Waiting for calibration...")
#         #while not check_calibration():
#          #   sleep(1)
#         #logger.info("Sensor fully calibrated and ready")
#     #except Exception as e:
#     #    logger.error(f"Initialization failed: {str(e)}")
#     #    raise
# 
# 
# def twos_comp(val, bits=16):
#     if val & (1 << (bits - 1)):
#         val -= (1 << bits)
#     return val
# 
# def read_vector(register):
#     data = read_register(register, 6) #6
#     #return data
#     return [
#         twos_comp((data[1] << 8) | data[0]),
#         twos_comp((data[3] << 8) | data[2]),
#         twos_comp((data[5] << 8) | data[4])
#     ]
# 
# def read_quaternion():
#         data = read_register(QUA_DATA_W_LSB, 8) #8
#         #return data
#         return [
#         twos_comp((data[1] << 8) | data[0]),
#         twos_comp((data[3] << 8) | data[2]),
#         twos_comp((data[5] << 8) | data[4]),
#         twos_comp((data[7] << 8) | data[6])
#     ]
# 
# def read_temperature():
#     return read_register(TEMP)[0]
# 
# def read_all_data():
#     #sleep(0.1)
#     accel = [x / 100.0 for x in read_vector(ACC_DATA_X_LSB)] #1m/s^2 = 100 lsb
#     
#     sleep(0.01)
#     mag = [x / 16.0 for x in read_vector(MAG_DATA_X_LSB)] #1uT = 16 lsb
#     gyro = [x / 16.0 for x in read_vector(GYR_DATA_X_LSB)] #1dps = 16 lsb
#     euler = [x / 16.0 for x in read_vector(EUL_DATA_X_LSB)] #1 degree = 16 lsb
#     quat = [x / (1 << 14) for x in read_quaternion()] #1 LSB = 1/16384
#     lia = [x / 100.0 for x in read_vector(LIA_DATA_X_LSB)] #1m/s^2 = 100 lsb
#     grv = [x / 100.0 for x in read_vector(GRV_DATA_X_LSB)] #1m/s^2 = 100 lsb
#     temp = read_temperature() #1 degree = 1 lsb
#     sleep(0.01)
#     return {
#         'acceleration': accel,
#         'magnetometer': mag,
#         'gyroscope': gyro,
#         'euler': euler,
#         'quaternion': quat,
#         'linear_acceleration': lia,
#         'gravity': grv,
#         'temperature': temp
#     }
# 
# def main():
#     #try:
#         initialize()
#         sleep(1)
#         set_external_crystal(True)
# 
#         #st_result = read_register(0x36,1)
#         #if (st_result[0] & 0x0F == 0x0F):
#             #print("st_result passed")
#         #else:
#             #print("st_result_failed")
#         count = 0
#         while True:
#             count+=1
#             print(count)
#             data = read_all_data()
#             print("Acceleration (m/s^2):", data['acceleration'])
#             print("Magnetometer (uT):", data['magnetometer'])
#             print("Gyroscope (dps):", data['gyroscope'])
#             print("Euler Angles (degrees):", data['euler'])
#             print("Quaternion:", data['quaternion'])
#             print("Linear Acceleration (m/s^2):", data['linear_acceleration'])
#             print("Gravity Vector (m/s^2):", data['gravity'])
#             print("Temperature (°C):", data['temperature'])
#             print("--------------------")
#     #except KeyboardInterrupt:
#     #    print("Interrupted by user")
#     #except Exception as e:
#     #    print("Error:", str(e))
#     #finally:
#     #    i2c.close()
#     #    gpio_out_imu_reset.close()

def main():
    from iclib.bno055 import BNO055, Register, OperationMode, Unit
    from unittest.mock import MagicMock
    from time import sleep
    bno055 = BNO055(i2c, MagicMock(direction='out', inverted=True))
    # bno055.select_operation_mode(False, False, False)
    # bno055.reset()
    # bno055.select_units(Unit.MS2, Unit.DPS, Unit.DEGREES, Unit.CELSIUS)
    # bno055.write(Register.PWR_MODE, 0x00)
    # bno055.write(Register.PAGE_ID, 0x00)
    # bno055.write(Register.SYS_TRIG, 0x00)
    # sleep(1)
    # bno055.write(Register.OPR_MODE, OperationMode.NDOF)

    while True:
        print(bno055.quaternion)
        sleep(1)

if __name__ == "__main__":
    main()

#https://python-periphery.readthedocs.io/en/latest/i2c.html  #function information
#https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf #tells read/write,register information

