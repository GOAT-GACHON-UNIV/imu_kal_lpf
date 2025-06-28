import time
from mpu6050 import mpu6050

mpu = mpu6050(0x68)

try:
    while(True):
        accel = mpu.get_accel_data()
        velocity = mpu.get_gyro_data()
        
        print('Angular Acceleration')
        print(' x=', accel['x'])
        print(' y=', accel['y'])
        print(' z=', accel['z'])

        print('Angular Velocity')
        print(' x=', velocity['x'])
        print(' y=', velocity['y'])
        print(' z=', velocity['z'])
        time.sleep(0.2)

except KeyboardInterrupt:
    print('00')
      
