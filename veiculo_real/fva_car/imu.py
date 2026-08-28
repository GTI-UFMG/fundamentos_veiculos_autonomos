# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import time
import numpy as np
import json
from pathlib import Path

# try import smbus2 then fallback to smbus
try:
    import smbus2 as smbus
except ImportError:
    try:
        import smbus
    except ImportError as e:
        raise ImportError("No smbus package found. Install 'python3-smbus' via apt or 'smbus2' via pip.") from e
        
########################################
# classe da IMU
########################################
class IMU:
	"""
	IMU class for MPU-9250/9265:
	- reads accel (m/s^2), gyro (deg/s) and mag (uT)
	- automatic accel and gyro calibration at init
	- mag initialized via AK8963 fuse rom adjustments
	- provides get_accel(), get_gyro(), get_mag(), get_euler()
	- provides calibrate_mag() and load_mag_cal()
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
	def __init__(	self, 
					address=0x68, 
					bus_id=4, 
					samples=100, 
					sample_delay=0.01, 
					do_wake=True,
					auto_calibrate=True,
					mag_cal_file=None):
						
		self.address = address
		self.bus_id = bus_id
		self.samples = samples
		self.sample_delay = sample_delay
		if mag_cal_file is None:
			self.mag_cal_file = (Path(__file__).resolve().parent/ "calibracao"/ "mag_cal.json")
		else:
			self.mag_cal_file = Path(mag_cal_file)

		# offsets/biases for accel (in g)
		self.offset_ax = 0.0
		self.offset_ay = 0.0
		self.offset_az = 0.0

		# gyro biases (deg/s)
		self.bias_gx = 0.0
		self.bias_gy = 0.0
		self.bias_gz = 0.0

		# mag adjustment scalars from fuse rom (1.0 = no change)
		self.mag_adj_x = 1.0
		self.mag_adj_y = 1.0
		self.mag_adj_z = 1.0

		# mag calibration (hard-iron bias in counts, scale per axis)
		self.mag_bias_counts = None   # tuple (mx_bias, my_bias, mz_bias)
		self.mag_scale = None         # tuple (sx, sy, sz)

		# open I2C bus
		self.bus = smbus.SMBus(self.bus_id)
		if do_wake:
			try:
				# wake up device
				self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
				time.sleep(0.05)
			except Exception as e:
				raise RuntimeError(f"Error waking IMU at address 0x{self.address:02x}: {e}")

		# init magnetometer			
		try:
			self._init_mag()
			self.mag_ok = True
		except Exception as e:
			self.mag_ok = False
			print(f"Magnetometro indisponivel: {e}")

		# try load mag calibration file if present
		if self.mag_ok and self.mag_cal_file.is_file():
			self.load_mag_cal(self.mag_cal_file)

		# calibrate accel and gyro
		if auto_calibrate:
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

		"""
		Calibra acelerometro e giroscopio assumindo:
		- sensor parado;
		- IMU aproximadamente nivelada;
		- eixo Z alinhado com a gravidade.
		"""
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
		 - ensure bypass enabled (tries to set INT_PIN_CFG)
		 - read ASA fuse values
		 - set continuous measurement mode 2, 16-bit (100 Hz)
		"""
		# ensure bypass: set BYPASS_EN bit (bit 1)
		try:
			cur = self.bus.read_byte_data(self.address, self.INT_PIN_CFG)
			self.bus.write_byte_data(self.address, self.INT_PIN_CFG, cur | 0x02)
		except OSError:
			self.bus.write_byte_data(self.address, self.INT_PIN_CFG, 0x02)
		time.sleep(0.01)

		# try read WIA
		wia = self.bus.read_byte_data(self.MAG_ADDR, self.MAG_WIA)
		# expected 0x48 for AK8963; not fatal if different
		if wia != 0x48:
			raise RuntimeError(f"Magnetometro AK8963 nao encontrado (WIA=0x{wia:02X})")

		# power down
		self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CNTL1, 0x00)
		time.sleep(0.01)

		# enter fuse rom access mode to read ASAx/ASAy/ASAz
		self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CNTL1, 0x0F)
		time.sleep(0.01)

		# read ASAX, ASAY, ASAZ
		data = [self.bus.read_byte_data(self.MAG_ADDR, self.MAG_ASAX + i) for i in range(3)]
		# compute adjustment: (ASA - 128)/256 + 1
		self.mag_adj_x = ((data[0] - 128) / 256.0) + 1.0
		self.mag_adj_y = ((data[1] - 128) / 256.0) + 1.0
		self.mag_adj_z = ((data[2] - 128) / 256.0) + 1.0

		# power down then set continuous measurement 2, 16-bit (0x16)
		self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CNTL1, 0x00)
		time.sleep(0.01)
		self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CNTL1, 0x16)
		time.sleep(0.01)

	########################################
	def _read_mag_raw(self):
		"""
		Robust read of AK8963:
		 - check ST1 DRDY bit (optional)
		 - read 7 bytes HXL..HZL + ST2
		 - check overflow
		 - return signed 16-bit (mx,my,mz) counts
		"""
		# read ST1
		st1 = self.bus.read_byte_data(self.MAG_ADDR, self.MAG_ST1)
		# dado ainda nao disponivel
		if not (st1 & 0x01):
			return None
			
		# attempt read 7 bytes: HXL..HZL + ST2
		data = [self.bus.read_byte_data(self.MAG_ADDR, self.MAG_HXL + i) for i in range(7)]
		mx = (data[1] << 8) | data[0]
		my = (data[3] << 8) | data[2]
		mz = (data[5] << 8) | data[4]
		st2 = data[6]

		# check overflow (HOFL bit = bit 3 of ST2)
		if st2 & 0x08:
			raise RuntimeError("Magnetometer overflow (data invalid)")

		# convert to signed
		if mx >= 0x8000:
			mx = -((65535 - mx) + 1)
		if my >= 0x8000:
			my = -((65535 - my) + 1)
		if mz >= 0x8000:
			mz = -((65535 - mz) + 1)
		return mx, my, mz

	########################################
	# magnetometer calibration and helpers
	def calibrate_mag(self, samples=600, delay=0.02, filename=None):
		"""
		Calibrate magnetometer using min/max method.
		Rotate device in all directions while collecting samples.
		Saves calibration to filename if provided.
		Returns calibration dict.
		"""
		if not getattr(self, "mag_ok", False):
			raise RuntimeError("Magnetometer not initialized.")

		print("Starting magnetometer calibration. Rotate device in all directions.")
		mx_min = [float("inf")] * 3
		mx_max = [float("-inf")] * 3

		cnt = 0
		while cnt < samples:
			raw = self._read_mag_raw()
			if raw is None:
				time.sleep(delay)
				continue
			mx, my, mz = raw

			vals = (mx, my, mz)
			for i, v in enumerate(vals):
				if v < mx_min[i]:
					mx_min[i] = v
				if v > mx_max[i]:
					mx_max[i] = v

			cnt += 1
			if cnt % 100 == 0:
				print(f"  samples: {cnt}/{samples}")
			time.sleep(delay)

		# compute bias (counts) and scale
		bias = [(mx_max[i] + mx_min[i]) / 2.0 for i in range(3)]
		rng = [(mx_max[i] - mx_min[i]) for i in range(3)]
		avg_rng = (rng[0] + rng[1] + rng[2]) / 3.0 if (rng[0] + rng[1] + rng[2]) != 0 else 1.0
		scale = [ (avg_rng / r) if r != 0 else 1.0 for r in rng ]

		# bias in uT for information
		bias_uT = [
			bias[0] * self.MAG_UT_LSB * self.mag_adj_x,
			bias[1] * self.MAG_UT_LSB * self.mag_adj_y,
			bias[2] * self.MAG_UT_LSB * self.mag_adj_z,
		]

		cal = {
			"bias_counts": {"mx": bias[0], "my": bias[1], "mz": bias[2]},
			"bias_uT": {"mx": bias_uT[0], "my": bias_uT[1], "mz": bias_uT[2]},
			"scale": {"sx": scale[0], "sy": scale[1], "sz": scale[2]},
		}

		# store internally (counts)
		self.mag_bias_counts = (cal["bias_counts"]["mx"], cal["bias_counts"]["my"], cal["bias_counts"]["mz"])
		self.mag_scale = (cal["scale"]["sx"], cal["scale"]["sy"], cal["scale"]["sz"])

		# save file if requested
		if filename:
			with open(filename, "w") as f:
				json.dump(cal, f, indent=2)
			print(f"Calibration saved to {filename}")

		print("Calibration done.")
		return cal

	########################################
	def load_mag_cal(self, filename):
		"""
		Load mag calibration JSON created by calibrate_mag.
		Sets mag_bias_counts and mag_scale.
		"""
		with open(filename, "r") as f:
			cal = json.load(f)
		self.mag_bias_counts = (
			cal["bias_counts"]["mx"],
			cal["bias_counts"]["my"],
			cal["bias_counts"]["mz"],
		)
		self.mag_scale = (
			cal["scale"]["sx"],
			cal["scale"]["sy"],
			cal["scale"]["sz"],
		)
		print(f"Mag calibration loaded: {filename}")

	########################################
	def get_accel_g(self):
		ax_raw, ay_raw, az_raw = self._read_accel_raw()
		ax = ax_raw / self.LSB_PER_G - self.offset_ax
		ay = ay_raw / self.LSB_PER_G - self.offset_ay
		az = az_raw / self.LSB_PER_G - self.offset_az
		return ax, ay, az

	########################################
	def get_accel(self):
		ax, ay, az = self.get_accel_g()
		return ax * self.G, ay * self.G, az * self.G

	########################################
	def get_gyro(self):
		gx_raw, gy_raw, gz_raw = self._read_gyro_raw()
		gx = gx_raw / self.LSB_PER_DPS - self.bias_gx
		gy = gy_raw / self.LSB_PER_DPS - self.bias_gy
		gz = gz_raw / self.LSB_PER_DPS - self.bias_gz
		return gx, gy, gz

	########################################
	def get_mag(self):
		"""
		Returns magnetometer vector in microtesla (uT).
		Applies mag calibration (bias_counts and scale) if present, then factory adj.
		"""
		if not getattr(self, "mag_ok", False):
			raise RuntimeError("Magnetometer not initialized")

		raw = self._read_mag_raw()
		if raw is None:
			return None
		mx_raw, my_raw, mz_raw = raw

		# apply hard-iron bias and scale in counts if present
		if self.mag_bias_counts is not None and self.mag_scale is not None:
			mx = (mx_raw - self.mag_bias_counts[0]) * self.mag_scale[0]
			my = (my_raw - self.mag_bias_counts[1]) * self.mag_scale[1]
			mz = (mz_raw - self.mag_bias_counts[2]) * self.mag_scale[2]
		else:
			mx, my, mz = mx_raw, my_raw, mz_raw

		# apply factory adjustment and convert to uT
		mx_uT = mx * self.MAG_UT_LSB * self.mag_adj_x
		my_uT = my * self.MAG_UT_LSB * self.mag_adj_y
		mz_uT = mz * self.MAG_UT_LSB * self.mag_adj_z

		return mx_uT, my_uT, mz_uT

	########################################
	# Euler angles
	def get_euler(self, degrees=True):
		"""
		Returns (roll, pitch, yaw).
		roll  = rotation around X axis
		pitch = rotation around Y axis
		yaw   = heading from magnetometer, 0..360 (requires mag)
		"""
		# read accel in g
		ax, ay, az = self.get_accel_g()

		roll = np.arctan2(ay, az)
		pitch = np.arctan2(-ax, np.sqrt(ay * ay + az * az))

		# compute yaw using magnetometer if available
		yaw = None
		if self.mag_ok:
			mag = self.get_mag() # in uT
			if mag is not None:
				mx, my, mz = mag
				
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
				heading = np.arctan2(my_comp, mx_comp)  # radians

				if degrees:
					yaw = (np.degrees(heading) + 360.0) % 360.0
				else:
					yaw = heading % (2 * np.pi)
						
		# convert roll/pitch to degrees if requested
		if degrees:
			return np.degrees(roll), np.degrees(pitch), yaw
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
		self.bus.close()

########################################
# main test
########################################
if __name__=="__main__":

	import matplotlib.pyplot as plt
	plt.ion()
	fig = plt.figure(1)

	imu = IMU(auto_calibrate=False)
	time.sleep(0.2)
	
	roll = []
	pitch = []
	yaw = []
	ts = []
	m = 50

	# funcao para testar o encoder por 10s
	t0 = time.time()
	while (time.time() - t0) <= 30.0:
		
		'''ax, ay, az = imu.get_accel()   # m/s^2
		gx, gy, gz = imu.get_gyro()    # deg/s
		mx, my, mz = imu.get_mag()     # uT
		print(f"Accel: {ax:.2f} {ay:.2f} {az:.2f} m/s2 | Gyro: {gx:.2f} {gy:.2f} {gz:.2f} deg/s | Mag: {mx} {my} {mz} uT | Euler: R={roll:.2f} P={pitch:.2f} Y={yaw}")
		time.sleep(0.1)'''
		
		r, p, y = imu.get_euler()  # degrees, yaw may be None if mag fails
		roll.append(r)
		pitch.append(p)
		yaw.append(y if y is not None else np.nan)
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
		
		yaw_text = f"{yaw[-1]:.2f}" if not np.isnan(yaw[-1]) else "N/A"
		print(f"Roll: {roll[-1]:.2f} | Pitch: {pitch[-1]:.2f} | Yaw: {yaw_text}")
		time.sleep(0.1)
		
	imu.close()
