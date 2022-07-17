package vl53l4cd

import (
	//"errors"
	"encoding/binary"
	"errors"
	"fmt"
	"time"

	"periph.io/x/conn/v3"
	"periph.io/x/conn/v3/i2c"
	"periph.io/x/conn/v3/physic"
)

const (
	VL53L4CD_IDENTIFICATION_MODEL_ID              = 0x010F
	VL53L4CD_FIRMWARE_SYSTEM_STATUS               = 0x00E5
	VL53L4CD_I2C_FAST_MODE_PLUS                   = 0x002D
	VL53L4CD_SYSTEM_START                         = 0x0087
	VL53L4CD_GPIO_TIO_HV_STATUS                   = 0x0031
	VL53L4CD_GPIO_HV_MUX_CTRL                     = 0x0030
	VL53L4CD_SYSTEM_INTERRUPT_CLEAR               = 0x0086
	VL53L4CD_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008
	VL53L4CD_RESULT_OSC_CALIBRATE_VAL             = 0x00DE
	VL53L4CD_INTERMEASUREMENT_MS                  = 0x006C
	VL53L4CD_RANGE_CONFIG_A                       = 0x005E
	VL53L4CD_RANGE_CONFIG_B                       = 0x0061
	VL53L4CD_RESULT_DISTANCE                      = 0x0096
)

type Opts struct {
	Addr uint16
}

var DefaultOpts = Opts{
	Addr: 0x29,
}

// Dev represents an sensor
type Dev struct {
	c    conn.Conn
	opts Opts
}

type SensorValues struct {
	Distance physic.Distance
}

// NewI2C returns a new driver.
func NewI2C(b i2c.Bus, opts *Opts) (*Dev, error) {
	dev := &Dev{
		c:    &i2c.Dev{Bus: b, Addr: opts.Addr},
		opts: *opts,
	}

	return dev, nil
}

func (d *Dev) Reset() error {
	if err := d.writeRegister(0x0000, []byte{0x00}); err != nil {
		return fmt.Errorf("set reset")
	}

	time.Sleep(100 * time.Microsecond)

	if err := d.writeRegister(0x0000, []byte{0x01}); err != nil {
		return fmt.Errorf("release reset")
	}

	time.Sleep(1 * time.Millisecond)

	return nil
}

func (d *Dev) Init() error {
	if data, err := d.readRegister(VL53L4CD_IDENTIFICATION_MODEL_ID, 2); err != nil {
		return fmt.Errorf("read model id: %w", err)
	} else {
		if data[0] != 0xeb || data[1] != 0xaa {
			return fmt.Errorf("unsupported sensor: %x %x", data[0], data[1])
		}
	}

	if err := d.waitForBoot(); err != nil {
		return fmt.Errorf("waiting for boot: %w", err)
	}

	initSeq := []byte{
		// value addr : description
		0x12, // 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch
		0x00, // 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
		0x00, // 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
		0x11, // 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1)
		0x02, // 0x31 : bit 1 = interrupt depending on the polarity
		0x00, // 0x32 : not user-modifiable
		0x02, // 0x33 : not user-modifiable
		0x08, // 0x34 : not user-modifiable
		0x00, // 0x35 : not user-modifiable
		0x08, // 0x36 : not user-modifiable
		0x10, // 0x37 : not user-modifiable
		0x01, // 0x38 : not user-modifiable
		0x01, // 0x39 : not user-modifiable
		0x00, // 0x3a : not user-modifiable
		0x00, // 0x3b : not user-modifiable
		0x00, // 0x3c : not user-modifiable
		0x00, // 0x3d : not user-modifiable
		0xFF, // 0x3e : not user-modifiable
		0x00, // 0x3f : not user-modifiable
		0x0F, // 0x40 : not user-modifiable
		0x00, // 0x41 : not user-modifiable
		0x00, // 0x42 : not user-modifiable
		0x00, // 0x43 : not user-modifiable
		0x00, // 0x44 : not user-modifiable
		0x00, // 0x45 : not user-modifiable
		0x20, // 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC
		0x0B, // 0x47 : not user-modifiable
		0x00, // 0x48 : not user-modifiable
		0x00, // 0x49 : not user-modifiable
		0x02, // 0x4a : not user-modifiable
		0x14, // 0x4b : not user-modifiable
		0x21, // 0x4c : not user-modifiable
		0x00, // 0x4d : not user-modifiable
		0x00, // 0x4e : not user-modifiable
		0x05, // 0x4f : not user-modifiable
		0x00, // 0x50 : not user-modifiable
		0x00, // 0x51 : not user-modifiable
		0x00, // 0x52 : not user-modifiable
		0x00, // 0x53 : not user-modifiable
		0xC8, // 0x54 : not user-modifiable
		0x00, // 0x55 : not user-modifiable
		0x00, // 0x56 : not user-modifiable
		0x38, // 0x57 : not user-modifiable
		0xFF, // 0x58 : not user-modifiable
		0x01, // 0x59 : not user-modifiable
		0x00, // 0x5a : not user-modifiable
		0x08, // 0x5b : not user-modifiable
		0x00, // 0x5c : not user-modifiable
		0x00, // 0x5d : not user-modifiable
		0x01, // 0x5e : not user-modifiable
		0xCC, // 0x5f : not user-modifiable
		0x07, // 0x60 : not user-modifiable
		0x01, // 0x61 : not user-modifiable
		0xF1, // 0x62 : not user-modifiable
		0x05, // 0x63 : not user-modifiable
		0x00, // 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), default value 90 mm
		0xA0, // 0x65 : Sigma threshold LSB
		0x00, // 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB)
		0x80, // 0x67 : Min count Rate LSB
		0x08, // 0x68 : not user-modifiable
		0x38, // 0x69 : not user-modifiable
		0x00, // 0x6a : not user-modifiable
		0x00, // 0x6b : not user-modifiable
		0x00, // 0x6c : Intermeasurement period MSB, 32 bits register
		0x00, // 0x6d : Intermeasurement period
		0x0F, // 0x6e : Intermeasurement period
		0x89, // 0x6f : Intermeasurement period LSB
		0x00, // 0x70 : not user-modifiable
		0x00, // 0x71 : not user-modifiable
		0x00, // 0x72 : distance threshold high MSB (in mm, MSB+LSB)
		0x00, // 0x73 : distance threshold high LSB
		0x00, // 0x74 : distance threshold low MSB ( in mm, MSB+LSB)
		0x00, // 0x75 : distance threshold low LSB
		0x00, // 0x76 : not user-modifiable
		0x01, // 0x77 : not user-modifiable
		0x07, // 0x78 : not user-modifiable
		0x05, // 0x79 : not user-modifiable
		0x06, // 0x7a : not user-modifiable
		0x06, // 0x7b : not user-modifiable
		0x00, // 0x7c : not user-modifiable
		0x00, // 0x7d : not user-modifiable
		0x02, // 0x7e : not user-modifiable
		0xC7, // 0x7f : not user-modifiable
		0xFF, // 0x80 : not user-modifiable
		0x9B, // 0x81 : not user-modifiable
		0x00, // 0x82 : not user-modifiable
		0x00, // 0x83 : not user-modifiable
		0x00, // 0x84 : not user-modifiable
		0x01, // 0x85 : not user-modifiable
		0x00, // 0x86 : clear interrupt, 0x01=clear
		0x00, // 0x87 : ranging, 0x00=stop, 0x40=start
	}

	if err := d.writeRegister(0x2d, initSeq); err != nil {
		return fmt.Errorf("writing init sequence: %w", err)
	}

	if err := d.startVhv(); err != nil {
		return fmt.Errorf("starting vhv: %w", err)
	}

	if err := d.clearInterrupt(); err != nil {
		return fmt.Errorf("clear interrupt: %w", err)
	}

	if err := d.stopRanging(); err != nil {
		return fmt.Errorf("stop ranging: %w", err)
	}

	if err := d.writeRegister(VL53L4CD_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, []byte{0x09}); err != nil {
		return fmt.Errorf("write vhv config: %w", err)
	}

	if err := d.writeRegister(0x0b, []byte{0x00}); err != nil {
		return fmt.Errorf("write register 0x0b: %w", err)
	}

	if err := d.writeRegister(0x24, []byte{0x05, 0x00}); err != nil {
		return fmt.Errorf("write register 0x05: %w", err)
	}

	if err := d.setInterMeasurement(0); err != nil {
		return fmt.Errorf("set inter measurement: %w", err)
	}

	if err := d.setTimingBudget(50); err != nil {
		return fmt.Errorf("set timing budget: %w", err)
	}

	return nil
}

func (d *Dev) Sense(values *SensorValues) (rerr error) {
	if err := d.startRanging(); err != nil {
		return fmt.Errorf("start ranging: %w", err)
	}

	defer func() {
		if err := d.stopRanging(); err != nil {
			rerr = fmt.Errorf("stop ranging: %w", err)
		}
	}()

	if err := d.waitForDataReady(); err != nil {
		return fmt.Errorf("wait for data ready: %w", err)
	}

	if err := d.clearInterrupt(); err != nil {
		return fmt.Errorf("clear interrupt: %w", err)
	}

	if dist, err := d.readDistance(); err != nil {
		return fmt.Errorf("read distance: %w", err)
	} else {
		values.Distance = dist
		return nil
	}
}

func (d *Dev) readDistance() (physic.Distance, error) {
	if data, err := d.readRegister(VL53L4CD_RESULT_DISTANCE, 2); err != nil {
		return 0, fmt.Errorf("read distance register: %w", err)
	} else {
		dist := binary.BigEndian.Uint16(data[0:])
		return physic.Distance(dist) * physic.MilliMetre, nil
	}
}

func (d *Dev) waitForBoot() error {
	for i := 0; i < 1000; i++ {
		data, err := d.readRegister(VL53L4CD_FIRMWARE_SYSTEM_STATUS, 1)
		if err != nil {
			return fmt.Errorf("reading system status: %w", err)
		}

		if data[0] == 0x03 {
			return nil
		}

		time.Sleep(1 * time.Millisecond)
	}
	return errors.New("timed out waiting for boot")
}

func (d *Dev) startVhv() error {
	if err := d.startRanging(); err != nil {
		return fmt.Errorf("start ranging: %w", err)
	}

	if err := d.waitForDataReady(); err != nil {
		return fmt.Errorf("wait for data ready: %w", err)
	}

	return nil
}

func (d *Dev) startRanging() error {
	interMeasurement, err := d.getInterMeasurement()
	if err != nil {
		return fmt.Errorf("getting inter-measurement: %w", err)
	}

	if interMeasurement == 0 {
		// continuous mode
		if err := d.writeRegister(VL53L4CD_SYSTEM_START, []byte{0x21}); err != nil {
			return fmt.Errorf("write system start: %w", err)
		}
	} else {
		// autonomous mode
		if err := d.writeRegister(VL53L4CD_SYSTEM_START, []byte{0x40}); err != nil {
			return fmt.Errorf("write system start: %w", err)
		}
	}

	if err := d.waitForDataReady(); err != nil {
		return fmt.Errorf("wait for data ready: %w", err)
	}

	if err := d.clearInterrupt(); err != nil {
		return fmt.Errorf("clear interrupt: %w", err)
	}

	return nil
}

func (d *Dev) stopRanging() error {
	if err := d.writeRegister(VL53L4CD_SYSTEM_START, []byte{0x00}); err != nil {
		return fmt.Errorf("write system start: %w", err)
	}

	return nil
}

func (d *Dev) waitForDataReady() error {
	for i := 0; i < 1000; i++ {
		if ready, err := d.dataReady(); err != nil {
			return fmt.Errorf("reading data ready: %w", err)
		} else {
			if ready {
				return nil
			}
		}
		time.Sleep(1 * time.Millisecond)
	}

	return errors.New("timed out waiting for data ready")
}

func (d *Dev) dataReady() (bool, error) {
	if data, err := d.readRegister(VL53L4CD_GPIO_TIO_HV_STATUS, 1); err != nil {
		return false, fmt.Errorf("reading status: %w", err)
	} else {
		if intPol, err := d.interruptPolarity(); err != nil {
			return false, fmt.Errorf("getting interrupt polarity: %w", err)
		} else {
			if data[0]&0x01 == intPol {
				return true, nil
			}
			return false, nil
		}
	}
}

func (d *Dev) interruptPolarity() (byte, error) {
	if data, err := d.readRegister(VL53L4CD_GPIO_HV_MUX_CTRL, 1); err != nil {
		return 0, fmt.Errorf("reading mux ctrl: %w", err)
	} else {
		if data[0]&0x10 == 0 {
			return 1, nil
		}
		return 0, nil
	}
}

func (d *Dev) clearInterrupt() error {
	if err := d.writeRegister(VL53L4CD_SYSTEM_INTERRUPT_CLEAR, []byte{0x01}); err != nil {
		return fmt.Errorf("writing interrupt clear: %w", err)
	}
	return nil
}

func (d *Dev) getInterMeasurement() (uint16, error) {
	data, err := d.readRegister(VL53L4CD_INTERMEASUREMENT_MS, 4)
	if err != nil {
		return 0, fmt.Errorf("reading inter-measurement register: %w", err)
	}

	val := binary.BigEndian.Uint32(data)

	data, err = d.readRegister(VL53L4CD_RESULT_OSC_CALIBRATE_VAL, 2)
	if err != nil {
		return 0, fmt.Errorf("reading osc calibrate register: %w", err)
	}

	clockPll := binary.BigEndian.Uint16(data)
	clockPll &= 0x3FF
	clockPll = uint16(float64(1.065) * float64(clockPll))
	return uint16(val / uint32(clockPll)), nil
}

func (d *Dev) setInterMeasurement(val uint16) error {
	timingBudget, err := d.getTimingBudget()
	if err != nil {
		return fmt.Errorf("getting timing budget: %w", err)
	}

	if val != 0 && uint32(val) < timingBudget {
		return fmt.Errorf("inter-measurement period can not be less than timing budget: %d < %d", val, timingBudget)
	}

	data, err := d.readRegister(VL53L4CD_RESULT_OSC_CALIBRATE_VAL, 2)
	if err != nil {
		return fmt.Errorf("reading osc calibrate: %w", err)
	}

	clockPll := binary.BigEndian.Uint16(data[0:]) & 0x3ff
	intMeas := uint32(float64(1.055) * float64(val) * float64(clockPll))
	write := make([]byte, 4)
	binary.BigEndian.PutUint32(write[0:], intMeas)
	if err := d.writeRegister(VL53L4CD_INTERMEASUREMENT_MS, write); err != nil {
		return fmt.Errorf("writing inter measurement: %w", err)
	}

	// need to reset timing budget so that it will be based on new inter-measurement period
	err = d.setTimingBudget(timingBudget)
	if err != nil {
		return fmt.Errorf("setting timing budget")
	}

	return nil
}

func (d *Dev) getTimingBudget() (uint32, error) {
	oscFreq, err := d.getOscFreq()
	if err != nil {
		return 0, fmt.Errorf("getting osc freq: %w", err)
	}

	macroPeriodUs := 16 * (uint32(2304*(uint32(0x40000000)/uint32(oscFreq))) >> 6)

	data, err := d.readRegister(VL53L4CD_RANGE_CONFIG_A, 2)
	if err != nil {
		return 0, fmt.Errorf("reading config A: %w", err)
	}

	macropHigh := uint32(binary.BigEndian.Uint16(data))
	lsByte := uint32((macropHigh & 0x00FF) << 4)
	msByte := uint32((macropHigh & 0xFF00) >> 8)
	msByte = uint32(0x04 - (msByte - 1) - 1)

	timingBudgetMs := (((lsByte + 1) * (macroPeriodUs >> 6)) - ((macroPeriodUs >> 6) >> 1)) >> 12

	if msByte < 12 {
		timingBudgetMs >>= msByte
	}

	interMeasurement, err := d.getInterMeasurement()
	if err != nil {
		return 0, fmt.Errorf("getting inter-measurement: %w", err)
	}

	if interMeasurement == 0 {
		// mode continuous
		timingBudgetMs += 2500
	} else {
		// mode autonomous
		timingBudgetMs *= 2
		timingBudgetMs += 4300
	}
	return uint32(timingBudgetMs / 1000), nil
}

func (d *Dev) setTimingBudget(val uint32) error {
	if val < 10 || val > 200 {
		return fmt.Errorf("timing budget range duration must be 10ms to 200ms")
	}

	interMeasurement, err := d.getInterMeasurement()
	if err != nil {
		return fmt.Errorf("getting inter-measurement: %w", err)
	}

	if interMeasurement != 0 && val > uint32(interMeasurement) {
		return fmt.Errorf("timing budget can not be greater than inter-measurement period: %d > %d", val, interMeasurement)
	}

	oscFreq, err := d.getOscFreq()
	if err != nil {
		return fmt.Errorf("getting osc freq: %w", err)
	}
	if oscFreq == 0 {
		return fmt.Errorf("osc frequency is 0")
	}

	timingBudgetUs := val * 1000

	macroPeriodUs := uint32(2304*(0x40000000/uint32(oscFreq))) >> 6

	if interMeasurement == 0 {
		// continuous mode
		timingBudgetUs -= 2500
	} else {
		// autonomous mode
		timingBudgetUs -= 4300
		timingBudgetUs /= 2
	}

	// config A
	msByte := uint32(0)
	timingBudgetUs <<= 12
	tmp := macroPeriodUs * 16
	lsByte := uint32(((timingBudgetUs + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1)
	for lsByte&0xFFFFFF00 > 0 {
		lsByte >>= 1
		msByte += 1
	}
	msByte = (msByte << 8) + (lsByte & 0xFF)
	write := make([]byte, 2)
	binary.BigEndian.PutUint16(write[0:], uint16(msByte))
	if err := d.writeRegister(VL53L4CD_RANGE_CONFIG_A, write); err != nil {
		return fmt.Errorf("writing range config A: %w", err)
	}

	// config B
	msByte = 0
	tmp = macroPeriodUs * 12
	lsByte = uint32(((timingBudgetUs + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1)
	for lsByte&0xFFFFFF00 > 0 {
		lsByte >>= 1
		msByte += 1
	}

	msByte = (msByte << 8) + (lsByte & 0xFF)
	write = make([]byte, 2)
	binary.BigEndian.PutUint16(write[0:], uint16(msByte))
	if err := d.writeRegister(VL53L4CD_RANGE_CONFIG_B, write); err != nil {
		return fmt.Errorf("writing range config B: %w", err)
	}

	return nil
}

func (d *Dev) getOscFreq() (uint16, error) {
	data, err := d.readRegister(0x0006, 2)
	if err != nil {
		return 0, fmt.Errorf("reading register 0x0006: %w", err)
	}

	return binary.BigEndian.Uint16(data), nil
}

func (d *Dev) readRegister(address uint16, length int) ([]byte, error) {
	write := make([]byte, 2)
	binary.BigEndian.PutUint16(write[0:], address)
	read := make([]byte, length)
	if err := d.c.Tx(write, read); err != nil {
		return nil, err
	}
	return read, nil
}

func (d *Dev) writeRegister(address uint16, data []byte) error {
	write := make([]byte, 2)
	binary.BigEndian.PutUint16(write[0:], address)
	write = append(write, data...)
	if err := d.c.Tx(write, []byte{}); err != nil {
		return err
	}
	return nil
}

// Halt all internal devices.
func (d *Dev) Halt() error {
	return nil
}
