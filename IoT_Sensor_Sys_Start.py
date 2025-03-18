from tkinter import *
import math
import RPi.GPIO as GPIO
import Adafruit_DHT as dht
import serial
import spidev
import time
import smbus


# 시리얼 포트 설정
ser = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
uart1 = serial.Serial('/dev/ttyAMA5', baudrate=9600, timeout=1)

# mcp3008 AOC 초기 설정
spi = spidev.SpiDev()
spi.open(0, 0)  # (버스, 디바이스)

# GPIO 핀 설정 (센서에 맞게 변경 필요)
air_quality_pin = 8
human_detection_pin = 18
temperature_humidity_pin_output = 16
temperature_humidity_pin = 21
current_sensor_pin = 12
dust_sensor_pin = 25
pressure_sensor_pin = 26
rbuf = [0, 0] 

bus = smbus.SMBus(0)  # I2C-0 버스
buss = smbus.SMBus(1) # I2C-1 버스

sensor_address_CM1107 = 0x31  # CM1107 센서의 I2C 주소 (0x31)
sensor_address_CM1107_Second = 0x31  # CM1107 센서의 I2C 주소 (0x31) 양자역학적인 놈
sensor_address_D6F = 0x6C  # 유압센서 D6F-0025AD1 센서의 I2C 주소

# CM1107 Command Definitions
CM1107_CMD_MEASURE = 0x01


# GPIO 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(air_quality_pin, GPIO.IN)
GPIO.setup(human_detection_pin, GPIO.IN)
GPIO.setup(temperature_humidity_pin, GPIO.IN)
GPIO.setup(current_sensor_pin, GPIO.IN)
GPIO.setup(dust_sensor_pin, GPIO.IN)
GPIO.setup(pressure_sensor_pin, GPIO.IN)

# GPIO 데이터 읽기
# MCP3008을 통한 아날로그 입력 읽기 함수
def read_adc(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

def read_air_quality():
    try:
        value_0 = read_adc(0)  # 0번 채널 읽기
        
        result = value_0 / 64
        return result

    except KeyboardInterrupt:
        spi.close()
        
def read_air_quality_output():
    try:
        value_1 = read_adc(0)  # 0번 채널 읽기
        
        result = value_1 / 64
        
        return result

    except KeyboardInterrupt:
        spi.close()
    

#def read_human_detection():
    #return "Human Detection: " + str(GPIO.input(human_detection_pin))

# 온습도 측정기 함수
def read_temperature_humidity():
    humidity, temperature = dht.read_retry(dht.DHT22, temperature_humidity_pin)
    if humidity is not None and temperature is not None:
        return "{0:0.1f},{1:0.1f}".format(temperature, humidity)
    else:
        return "Failed to read temperature and humidity data."
        
def read_temperature_humidity_output():
    humidity1, temperature1 = dht.read_retry(dht.DHT22, temperature_humidity_pin_output)
    if humidity1 is not None and temperature1 is not None:
        return "{0:0.1f},{1:0.1f}".format(temperature1, humidity1)
    else:
        return "Failed to read temperature and humidity data."

# CO2 센서 사용시 센서 초기화를 위한 함수
def init_sensor_Second():
    try:
        # Send a command to start measuring
        buss.write_byte(sensor_address_CM1107_Second, 0x01)
        time.sleep(0.1)  # Give some time for the sensor to perform the measurement
    except Exception as e:
        print("Error starting measurement on the CO2 sensor:", str(e))

# Read CO2 concentration and status from the sensor
def read_co2_sensor_Second():
    try:
        # Read the response data
        response = buss.read_i2c_block_data(sensor_address_CM1107_Second, 0x00, 7)

        # Extract DF0, DF1 from the response
        df0 = response[1]
        df1 = response[2]

        co2_ppm = (df0 << 8 | df1)
        return co2_ppm
    except Exception as e:
        print("Error reading CO2 sensor:", str(e))
        return "Error"


# CO2 센서 사용시 센서 초기화를 위한 함수
def init_sensor():
    try:
        # Send a command to start measuring
        buss.write_byte(sensor_address_CM1107, CM1107_CMD_MEASURE)
        time.sleep(0.1)  # Give some time for the sensor to perform the measurement
    except Exception as e:
        print("Error starting measurement on the CO2 sensor:", str(e))

# Read CO2 concentration and status from the sensor
def read_co2_sensor():
    try:
        # Read the response data
        response = buss.read_i2c_block_data(sensor_address_CM1107, 0x00, 7)

        # Extract DF0, DF1 from the response
        df0 = response[1]
        df1 = response[2]

        co2_ppm = (df0 << 8 | df1)
        return co2_ppm
    except Exception as e:
        print("Error reading CO2 sensor:", str(e))
        return "Error"

def read_dust_sensor():
    pm2_5_cf1 = 0
    time.sleep(1)
    try:
        # 센서로부터 데이터 읽기
        data = ser.read(32)  # PMS7003M은 32바이트 데이터 패킷을 보냅니다.
        # 데이터 패킷 길이 확인
        if len(data) == 32:
            time.sleep(0.3)
            if data[0] == 0x42 and data[1] == 0x4D:
                # 데이터 검증
                
                pm1_cf12 = data[10] << 8 | data[11]
                pm2_5_cf1 = data[12] << 8 | data[13]
                pm10_cf12= data[14] << 8 | data[15]
            else:
                print("데이터 패킷의 시작 바이트가 올바르지 않습니다.")
        else:
            print("데이터 패킷 길이가 올바르지 않습니다.")

    except KeyboardInterrupt:
        ser.close()
    return pm2_5_cf1

def read_dust_sensor_output():
    pm2_5_cf12 = 0
    time.sleep(0.3)
    try:
        # 센서로부터 데이터 읽기
        data = uart1.read(32)  # PMS7003M은 32바이트 데이터 패킷을 보냅니다.
        # 데이터 패킷 길이 확인
        if len(data) == 32:
            # time.sleep(0.1)
            if data[0] == 0x42 and data[1] == 0x4D:
                # 데이터 검증
                
                pm1_cf12 = data[10] << 8 | data[11]
                pm2_5_cf12 = data[12] << 8 | data[13]
                pm10_cf12= data[14] << 8 | data[15]
            else:
                print("데이터 패킷의 시작 바이트가 올바르지 않습니다.")
        else:
            print("데이터 패킷 길이가 올바르지 않습니다.")

    except KeyboardInterrupt:
        uart1.close()
    return pm2_5_cf12
    
def read_pressure_sensor():
    try:
        # 센서 초기화 및 MCU 모드 활성화 (0Bh, 00h)
        i2c_write_reg16(sensor_address_D6F, 0x0B00, None, 0)
        time.sleep(0.9)  # 900ms 대기

        # 데이터 요청 (00h, D0h, 40h, 18h, 06h)
        send0 = [0x40, 0x18, 0x06]
        i2c_write_reg16(sensor_address_D6F, 0x00D0, send0, 3)

        time.sleep(0.05)  # 50ms 대기

        # 데이터 읽기 (00h, D0h, 51h, 2Ch) (07h)
        send1 = [0x51, 0x2C]
        i2c_write_reg16(sensor_address_D6F, 0x00D0, send1, 2)

        # 데이터 읽기 (07h)
        # rbuf = i2c_read_reg8(sensor_address_D6F, 0x07, 2)
        
        rd_flow = (rbuf[0] << 8) | rbuf[1]
        pressure = (((rd_flow - 1024) / 60000 * 250 ) * -1) / 6894.76 # 0-250[Pa] 범위 계산
        
        return pressure
        
    except Exception as e:
        print(f"Error reading sensor data: {str(e)}")
        return None
    
def i2c_write_reg16(devAddr, regAddr, data, length):
    try:
        # I2C 디바이스 선택
        bus.write_i2c_block_data(devAddr, regAddr >> 8, [regAddr & 0xFF] + data)
        
        return None

    except Exception as e:
        return str(e)
        
def i2c_read_reg8(devAddr, regAddr, length):
    try:
        # I2C 디바이스 선택
        bus.write_i2c_block_data(devAddr, regAddr >> 8, [regAddr & 0xFF])
        
        # 데이터 읽기
        data = bus.read_i2c_block_data(devAddr, regAddr, length)
        
        # 각 데이터 항목을 정수로 변환
        data_int = [int(x) for x in data]
        
        return data_int  # 데이터를 정수로 변환하여 반환

    except Exception as e:
        return str(e)

def calculate_velocity(delta_p, density):
    velocity = math.sqrt((2 * delta_p) / density)
    return velocity

def calculate_flow_rate(velocity, area):
    flow_rate = velocity * area
    return flow_rate

def flowRate():
    # 주어진 값
    delta_pressure = read_pressure_sensor()  # 압력 차이 (예: P1 - P2)
    fluid_density = 1000  # 물의 밀도 (kg/m³)
    pipe_area = 0.12  # 파이프 단면적

    # 속력 계산
    fluid_velocity = calculate_velocity(delta_pressure, fluid_density)

    # 유량 계산
    flow = calculate_flow_rate(fluid_velocity, pipe_area) * 1000
    return flow

# 주기적 업데이트 시작

########################################################################################################
root = Tk()
root.config(bg = "gray90")
root.wm_attributes("-topmost", 1) # 창을 항상 상단에 배치 / 0 외 모든 인자 True

root.attributes('-fullscreen', True)
def exit_fullscreen(event):
    root.attributes('-fullscreen', False)

def update_number():
    # CO2 센서 초기화 및 측정
    init_sensor()
    co2_val = read_co2_sensor()
    
    # 온습도 측정
    temp_humid = read_temperature_humidity().split(",")
    temp_label.config(text = f"{temp_humid[0]}℃\n{temp_humid[1]}%")
    
    # 공기질 측정
    point = read_air_quality()
    
    # CO2 값 설정
    co2_label.config(text = f"{co2_val}\nppm")
    
    # TVOC 값 설정 (미세먼지 센서 사용)
    dust_val = read_dust_sensor() / 1000
    dust_label.config(text = f"{dust_val:.3f}\nppm")
    
    # 차압 값 설정
    pressure_label.config(text = f"{read_pressure_sensor():.5f}\npsi")
    
    # 미세먼지 값 설정
    air_quality_label.config(text = f"{read_air_quality():.3f}\n㎍/㎥")
    
    # 유량 값 설정 
    flowRate_label.config(text = f"{flowRate():.5f}\nL/s")
    
    root.after(2000, update_number)
    
def update_out():
    # CO2 센서 초기화 및 측정 (두 번째 센서)
    init_sensor_Second()
    co2_val_out = read_co2_sensor_Second()
    
    # 온습도 측정 (외부)
    temp_humid_out = read_temperature_humidity_output().split(",")
    temp_label_out.config(text = f"{temp_humid_out[0]}℃\n{temp_humid_out[1]}%")
    
    # CO2 값 설정 (외부)
    co2_label_out.config(text = f"{co2_val_out}\nppm")
    
    # TVOC 값 설정 (외부 미세먼지 센서 사용)
    dust_val_out = read_dust_sensor_output() / 1000
    dust_label_out.config(text = f"{dust_val_out:.3f}\nppm")
    
    # 미세먼지 값 설정 (외부)
    air_quality_label_out.config(text = f"{read_air_quality_output():.3f}\n㎍/㎥")
    
    root.after(2000, update_out)


# 전체 화면 모드에서 빠져나가기 위한 이벤트 바인딩
root.bind("<Escape>", exit_fullscreen)

ww = 10
hh = 6

title_title = Label(root, pady = 10, height = 2, text = "IoT Monitoring System", font =("Helvetica", 30), bg = "gray90")
title_title.grid(row = 1, column = 3, columnspan = 5)

current_title = Label(root, width = 5, height = 2, text = "전원 : ON", font =("Helvetica", 20), bg = "gray90").grid(row = 2, column = 7, columnspan = 3, sticky = E+W)

in_text = Label(root, width = 4, height = 4, text = "IN", font =("Helvetica", 20), bg = "gray90").grid(row = 4, column = 1)
out_text = Label(root, width = 4, height = 4, text = "OUT", font =("Helvetica", 20), bg = "gray90").grid(row = 5, column = 1)

temp_title = Label(root, width = ww, pady = 20, text = "온/습도", font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink").grid(row = 3, column = 2, sticky = E+W)
temp_label = Label(root, width = ww, height = hh, pady = 30, font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink")
temp_label.grid(row = 4, column = 2, sticky = E+W+S+N)
temp_label_out = Label(root, width = ww, height = hh, pady = 30, font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink")
temp_label_out.grid(row = 5, column = 2, sticky = E+W+S+N)

air_quality_title = Label(root, width = ww, pady = 20, text = "미세먼지", font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink").grid(row = 3, column = 3, sticky = E+W)
air_quality_label = Label(root, width = ww, height = hh, pady = 30, font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink")
air_quality_label.grid(row = 4, column = 3, sticky = E+W+S+N)
air_quality_label_out = Label(root, width = ww, height = hh, pady = 30, font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink")
air_quality_label_out.grid(row = 5, column = 3, sticky = E+W+S+N)

dust_title = Label(root, width = ww, pady = 20, text = "TVOC", font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink").grid(row = 3, column = 4, sticky = E+W)
dust_label = Label(root, width = ww, height = hh, pady = 30, font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink")
dust_label.grid(row = 4, column = 4, sticky = E+W+S+N)
dust_label_out = Label(root, width = ww, height = hh, pady = 30, font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink")
dust_label_out.grid(row = 5, column = 4, sticky = E+W+S+N)

co2_title = Label(root, width = ww, pady = 20, text = "CO2", font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink").grid(row = 3, column = 5, sticky = E+W)
co2_label = Label(root, width = ww, height = hh, pady = 30, font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink")
co2_label.grid(row = 4, column = 5, sticky = E+W+S+N)
co2_label_out = Label(root, width = ww, height = hh, pady = 30, font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink")
co2_label_out.grid(row = 5, column = 5, sticky = E+W+S+N)

empty_title = Label(root, width = 1, pady = 20).grid(row = 3, column = 6, sticky = E+W+S+N)
empty_label = Label(root, width = 1, height = hh, pady = 30).grid(row = 4, column = 6, sticky = E+W+S+N)
empty_label_out = Label(root, width = 1, height = hh, pady = 30).grid(row = 5, column = 6, sticky = E+W+S+N)

flowRate_title = Label(root, width = ww, pady = 20, text = "유량", font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink").grid(row = 3, column = 7, sticky = E+W)
flowRate_label = Label(root, width = ww, height = hh, pady = 30, font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink")
flowRate_label.grid(row = 4, column = 7, rowspan = 2, sticky = E+W+S+N)

pressure_title = Label(root, width = ww, pady = 20, text = "차압", font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink").grid(row = 3, column = 8, sticky = E+W)
pressure_label = Label(root, width = ww, height = hh, pady = 30, font = ("Helvetica", 20), relief = "solid", bd = 1, bg = "pink")
pressure_label.grid(row = 4, column = 8, rowspan = 2, sticky = E+W+S+N)

update_number()
update_out()
root.mainloop()
