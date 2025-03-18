# Raspberry Pi 기반 환경 센서 시스템 개발

## 프로젝트 개요

본 프로젝트는 Raspberry Pi를 활용하여 다양한 환경 센서를 통합하고, 실시간 데이터를 수집 및 분석하는 시스템을 개발하는 것을 목표로 합니다. 이를 통해 공기질, 온습도, CO2 농도, 미세먼지, 압력 및 유량 등을 측정하고 시각화할 수 있습니다.

---

## 개발 내용

### 1. 하드웨어 구성

- **사용한 센서 및 모듈**
    - CO2 센서 (CM1107)
    - 온습도 센서 (DHT22)
    - 미세먼지 센서 (PMS7003M)
    - 압력 센서 (D6F-0025AD1)
    - 아날로그 입력 변환기 (MCP3008)
    - GPIO 기반 센서 (인체 감지, 공기질 측정 등)
    - I2C 통신을 통한 데이터 수집
- **통신 방식**
    - SPI (MCP3008 아날로그-디지털 변환)
    - I2C (압력 및 CO2 센서)
    - UART (미세먼지 센서 PMS7003M)
    - GPIO (공기질 및 인체 감지)

---

### 2. 소프트웨어 구성

- **프로그래밍 언어 및 라이브러리**
    - Python (데이터 처리 및 GUI 개발)
    - Tkinter (GUI 개발 및 데이터 시각화)
    - RPi.GPIO, smbus, spidev, serial 등의 라이브러리를 활용한 센서 데이터 수집
    - Adafruit_DHT 라이브러리를 활용한 온습도 측정
- **기능 구현**
    - 센서 데이터 실시간 수집 및 분석
    - 환경 지표에 따른 알림 기능 추가
    - Tkinter 기반 GUI를 활용한 실시간 모니터링 시스템 구현
    - 데이터 변환 및 보정 알고리즘 적용 (예: 압력 데이터의 PSI 변환, CO2 데이터 필터링 등)

---

### 3. 주요 기능

- **센서 데이터 수집**
    - CO2, 온도, 습도, 미세먼지, 압력, 공기질, 유량 측정 및 분석
    - MCP3008 ADC를 활용한 아날로그 데이터 변환
    - I2C, SPI, UART를 통한 데이터 통신
- **데이터 시각화**
    - Tkinter를 활용한 실시간 모니터링 인터페이스
    - 환경 데이터의 수치를 직관적으로 표시 (ppm, psi, ℃ 등)
- **자동화 및 업데이트 기능**
    - 일정 주기마다 데이터 갱신 (2초마다 업데이트)
    - 데이터 필터링 및 처리 알고리즘 적용

---

## 프로젝트 결과

- 실시간으로 환경 데이터를 수집하고 표시하는 시스템 완성
- 다양한 센서를 통합하여 환경 모니터링이 가능하도록 구현
- Raspberry Pi를 활용한 IoT 환경 감시 시스템 구축

---

## 향후 발전 방향

- 데이터 로깅 기능 추가 및 클라우드 서버 연동
- AI 기반 이상 탐지 및 분석 기능 추가
- 모바일 앱과 연동하여 원격 모니터링 가능하도록 개선
- 보다 정밀한 센서 보정 및 데이터 필터링 기법 적용

본 프로젝트를 통해 IoT 환경 모니터링 시스템의 구축 경험을 쌓았으며, Raspberry Pi를 활용한 센서 데이터 수집 및 분석 기술을 향상시킬 수 있었습니다.
