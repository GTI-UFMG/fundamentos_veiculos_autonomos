# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2025/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import time
import numpy as np

# try import smbus2 then fallback to smbus
try:
    import smbus2 as smbus
except Exception:
    try:
        import smbus
    except Exception as e:
        raise ImportError(
            "No smbus package found. Install 'python3-smbus' via apt or 'smbus2' via pip."
        ) from e

########################################
# classe da IMU
########################################
class IMU:
    """
    IMU class for MPU-9250/9265:
    - reads accel (m/s^2), gyro (deg/s) and mag (uT)
    - automatic accel and gyro calibration at init
    - mag initialized via AK8963 fuse rom adjustments
    - provides getAccel(), getGyro(), getMag(), getEuler()
    """
    
    # MPU registers
    PWR_MGMT_1 = 0x6B
    INT_PIN_CFG = 0x37
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    # AK8963 (mag) I2C address and regs
    MAG_ADDR = 0x0C
    MAG_WIA = 0x00
    MAG_ST1 = 0x02
    MAG_HXL = 0x03
    MAG_CNTL1 = 0x0A
    MAG_ASAX = 0x10

    # scales
    G = 9.80665           # m/s^2 per g
    LSB_PER_G = 16384.0   # accel full scale +-2g
    LSB_PER_DPS = 131.0   # gyro full scale +-250 deg/s
    MAG_UT_LSB = 0.15     # microtesla per LSB for AK8963 16-bit

    ########################################
    # construtor
    def __init__(self, address=0x68, bus_id=4, samples=100, sample_delay=0.01, do_wake=True):
        self.address = address
        self.bus_id = bus_id
        self.samples = samples
        self.sample_delay = sample_delay

        # offsets/biases
        self.offset_ax = 0.0
        self.offset_ay = 0.0
        self.offset_az = 0.0

        self.bias_gx = 0.0
        self.bias_gy = 0.0
        self.bias_gz = 0.0

        # mag adjustment scalars from fuse rom (1.0 = no change)
        self.mag_adj_x = 1.0
        self.mag_adj_y = 1.0
        self.mag_adj_z = 1.0

        # open I2C bus
        self.bus = smbus.SMBus(self.bus_id)

        if do_wake:
            try:
                # wake up device
                self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
                time.sleep(0.05)
            except Exception as e:
                raise RuntimeError(f"Error waking IMU at address 0x{self.address:02x}: {e}")

        # enable I2C bypass so we can talk directly to AK8963
        try:
            # set INT_PIN_CFG bit 1 (BYPASS_EN)
            self.bus.write_byte_data(self.address, self.INT_PIN_CFG, 0x02)
            time.sleep(0.01)
        except Exception:
            # not fatal, mag init may fail later
            pass

        # init magnetometer
        try:
            self._init_mag()
            self.mag_ok = True
        except Exception:
            self.mag_ok = False

        # calibrate accel and gyro
        self._calibrate_accel_and_gyro()

    ########################################
    # low level helpers
    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            val = -((65535 - val) + 1)
        return val

    ########################################
    # low level helpers
    def _read_accel_raw(self):
        x = self._read_word(self.ACCEL_XOUT_H)
        y = self._read_word(self.ACCEL_XOUT_H + 2)
        z = self._read_word(self.ACCEL_XOUT_H + 4)
        return x, y, z

    ########################################
    # low level helpers
    def _read_gyro_raw(self):
        x = self._read_word(self.GYRO_XOUT_H)
        y = self._read_word(self.GYRO_XOUT_H + 2)
        z = self._read_word(self.GYRO_XOUT_H + 4)
        return x, y, z

    ########################################
    # accel and gyro calibration
    def _calibrate_accel_and_gyro(self):
        """
        Collect samples while device is stationary to compute:
         - accel offsets in g
         - gyro biases in deg/s
        """
        sx = sy = sz = 0.0
        sgx = sgy = sgz = 0.0
        cnt = 0
        for _ in range(self.samples):
            ax_raw, ay_raw, az_raw = self._read_accel_raw()
            gx_raw, gy_raw, gz_raw = self._read_gyro_raw()

            # convert to g and deg/s
            ax = ax_raw / self.LSB_PER_G
            ay = ay_raw / self.LSB_PER_G
            az = az_raw / self.LSB_PER_G

            gx = gx_raw / self.LSB_PER_DPS
            gy = gy_raw / self.LSB_PER_DPS
            gz = gz_raw / self.LSB_PER_DPS

            sx += ax
            sy += ay
            sz += az

            sgx += gx
            sgy += gy
            sgz += gz

            cnt += 1
            time.sleep(self.sample_delay)

        if cnt == 0:
            raise RuntimeError("No samples collected for calibration")

        ox = sx / cnt
        oy = sy / cnt
        oz = sz / cnt

        # adjust z offset subtracting gravity if sensor is oriented with z ~ +1g
        if abs(oz) > 0.8:
            oz = oz - (1.0 if oz > 0 else -1.0)

        self.offset_ax = ox
        self.offset_ay = oy
        self.offset_az = oz

        self.bias_gx = sgx / cnt
        self.bias_gy = sgy / cnt
        self.bias_gz = sgz / cnt

    ########################################
    # magnetometer init and read
    def _init_mag(self):
        """
        Initialize AK8963:
         - power down
         - enter fuse rom access to read ASA
         - compute adjustment scales
         - set continuous 16-bit mode (100 Hz)
        """
        # power down
        self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CNTL1, 0x00)
        time.sleep(0.01)

        # enter fuse rom access mode
        self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CNTL1, 0x0F)
        time.sleep(0.01)

        # read ASAX, ASAY, ASAZ
        data = [self.bus.read_byte_data(self.MAG_ADDR, self.MAG_ASAX + i) for i in range(3)]
        # compute adjustment: (ASA - 128)/256 + 1
        self.mag_adj_x = ((data[0] - 128) / 256.0) + 1.0
        self.mag_adj_y = ((data[1] - 128) / 256.0) + 1.0
        self.mag_adj_z = ((data[2] - 128) / 256.0) + 1.0

        # power down
        self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CNTL1, 0x00)
        time.sleep(0.01)

        # set continuous measurement mode 2, 16-bit (0x16)
        self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CNTL1, 0x16)
        time.sleep(0.01)

    ########################################
    def _read_mag_raw(self):
        """
        Read raw mag data (signed 16-bit) from AK8963 registers HXL..HZL.
        Returns tuple (mx, my, mz) raw counts or raises on read error.
        """
        # read ST1 and check data ready bit
        st1 = self.bus.read_byte_data(self.MAG_ADDR, self.MAG_ST1)
        # read 6 bytes HXL..HZL
        data = [self.bus.read_byte_data(self.MAG_ADDR, self.MAG_HXL + i) for i in range(6)]
        # combine little-endian
        mx = data[1] << 8 | data[0]
        my = data[3] << 8 | data[2]
        mz = data[5] << 8 | data[4]
        # convert to signed
        if mx >= 0x8000:
            mx = -((65535 - mx) + 1)
        if my >= 0x8000:
            my = -((65535 - my) + 1)
        if mz >= 0x8000:
            mz = -((65535 - mz) + 1)
        return mx, my, mz

    ########################################
    def getAccel_g(self):
        ax_raw, ay_raw, az_raw = self._read_accel_raw()
        ax = ax_raw / self.LSB_PER_G - self.offset_ax
        ay = ay_raw / self.LSB_PER_G - self.offset_ay
        az = az_raw / self.LSB_PER_G - self.offset_az
        return ax, ay, az

    ########################################
    def getAccel(self):
        ax, ay, az = self.getAccel_g()
        return ax * self.G, ay * self.G, az * self.G

    ########################################
    def getGyro(self):
        gx_raw, gy_raw, gz_raw = self._read_gyro_raw()
        gx = gx_raw / self.LSB_PER_DPS - self.bias_gx
        gy = gy_raw / self.LSB_PER_DPS - self.bias_gy
        gz = gz_raw / self.LSB_PER_DPS - self.bias_gz
        return gx, gy, gz

    ########################################
    def getMag(self):
        """
        Returns magnetometer vector in microtesla (uT).
        Applies factory adjustment values.
        """
        if not self.mag_ok:
            raise RuntimeError("Magnetometer not initialized")
        mx_raw, my_raw, mz_raw = self._read_mag_raw()
        # apply adjustment and convert to uT
        mx = mx_raw * self.MAG_UT_LSB * self.mag_adj_x
        my = my_raw * self.MAG_UT_LSB * self.mag_adj_y
        mz = mz_raw * self.MAG_UT_LSB * self.mag_adj_z
        return mx, my, mz

    ########################################
    # Euler angles
    def getEuler(self, degrees=True):
        """
        Returns (roll, pitch, yaw).
        roll  = rotation around X axis  (deg)
        pitch = rotation around Y axis  (deg)
        yaw   = heading from magnetometer (deg, 0..360)
        If degrees=False, returns radians.
        Note: yaw requires a working magnetometer and good mag calibration.
        """
        # read accel in g
        ax, ay, az = self.getAccel_g()

        # compute roll and pitch (radians)
        # roll: atan2(ay, az)
        roll = np.atan2(ay, az)

        # pitch: atan2(-ax, sqrt(ay^2 + az^2))
        pitch = np.atan2(-ax, np.sqrt(ay * ay + az * az))

        yaw = None
        # compute yaw using magnetometer if available
        if self.mag_ok:
            try:
                mx, my, mz = self.getMag()  # in uT
                # convert uT to plain numbers for trig (units cancel out)
                # tilt compensation
                cos_pitch = np.cos(pitch)
                sin_pitch = np.sin(pitch)
                cos_roll = np.cos(roll)
                sin_roll = np.sin(roll)

                # reference formulas for tilt compensation
                mx_comp = mx * cos_pitch + mz * sin_pitch
                my_comp = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch

                # heading
                heading = np.atan2(my_comp, mx_comp)  # radians

                # convert to degrees and normalize
                if degrees:
                    yaw = np.degrees(heading)
                    yaw = yaw % 360.0
                else:
                    yaw = heading % (2 * np.pi)
            except Exception:
                yaw = None

        # convert roll/pitch to degrees if requested
        if degrees:
            roll_d = np.degrees(roll)
            pitch_d = np.degrees(pitch)
            return roll_d, pitch_d, yaw
        else:
            return roll, pitch, yaw

    ########################################
    def recalibrate(self, samples=None, sample_delay=None):
        """
        Force recalibration for accel and gyro.
        """
        if samples is not None:
            self.samples = samples
        if sample_delay is not None:
            self.sample_delay = sample_delay
        self._calibrate_accel_and_gyro()
    
    ########################################
    # fecha IMU
    def close(self):
        None

########################################
# main test
########################################
if __name__=="__main__":

    import matplotlib.pyplot as plt
    plt.ion()
    fig = plt.figure(1)

    imu = IMU()
    time.sleep(0.2)
    
    roll = []
    pitch = []
    yaw = []
    ts = []
    m = 50

    # funcao para testar o encoder por 10s
    t0 = time.time()
    while (time.time() - t0) <= 30.0:
        
        '''ax, ay, az = imu.getAccel()   # m/s^2
        gx, gy, gz = imu.getGyro()    # deg/s
        mx, my, mz = imu.getMag()     # uT
        print(f"Accel: {ax:.2f} {ay:.2f} {az:.2f} m/s2 | Gyro: {gx:.2f} {gy:.2f} {gz:.2f} deg/s | Mag: {mx} {my} {mz} uT | Euler: R={roll:.2f} P={pitch:.2f} Y={yaw}")
        time.sleep(0.1)'''
        
        r, p, y = imu.getEuler()  # degrees, yaw may be None if mag fails
        roll.append(r)
        pitch.append(p)
        yaw.append(y)
        ts.append(time.time() - t0)
        
        if len(ts) % 2 == 0:
            plt.clf()
            plt.subplot(311)
            plt.plot(ts[-m:], roll[-m:], 'r', label='Roll')
            plt.ylim([-30, 30])
            plt.ylabel('Roll [deg]')
            plt.subplot(312)
            plt.plot(ts[-m:], pitch[-m:], 'b', label='Pitch')
            plt.ylim([-30, 30])
            plt.ylabel('Pitch [deg]')
            plt.subplot(313)
            plt.plot(ts[-m:], yaw[-m:], 'g', label='yaw')
            plt.ylim([0.0, 360.0])
            plt.ylabel('Yaw [deg]')
            plt.xlabel('Time [s]')
            
            plt.pause(0.1)
            plt.show()
        
        print(f"Roll: {roll[-1]:.2f} | Pitch: {pitch[-1]:.2f} | Yaw: {yaw[-1]:.2f}")
        time.sleep(0.1)
        
    imu.close()
