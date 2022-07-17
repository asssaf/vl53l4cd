package cmd

import (
	"flag"
	"fmt"
	"log"

	"periph.io/x/conn/v3/i2c/i2creg"
	"periph.io/x/host/v3"

	"github.com/asssaf/vl53l4cd-go/vl53l4cd"
)

type ReadCommand struct {
	fs   *flag.FlagSet
	addr int
}

func NewReadCommand() *ReadCommand {
	c := &ReadCommand{
		fs: flag.NewFlagSet("read", flag.ExitOnError),
	}

	c.fs.IntVar(&c.addr, "address", 0, "Device address (0x29)")

	return c
}

func (c *ReadCommand) Name() string {
	return c.fs.Name()
}

func (c *ReadCommand) Init(args []string) error {
	if err := c.fs.Parse(args); err != nil {
		return err
	}

	flag.Usage = c.fs.Usage

	return nil
}

func (c *ReadCommand) Execute() error {
	// Make sure periph is initialized.
	if _, err := host.Init(); err != nil {
		log.Fatalf("host init: %w", err)
	}

	i2cPort, err := i2creg.Open("/dev/i2c-1")
	if err != nil {
		log.Fatalf("i2c open: %w", err)
	}

	opts := vl53l4cd.DefaultOpts

	dev, err := vl53l4cd.NewI2C(i2cPort, &opts)
	if err != nil {
		log.Fatal("device creation: %w", err)
	}
	defer dev.Halt()

	if err := dev.Init(); err != nil {
		log.Fatalf("device init: %w", err)
	}

	values := vl53l4cd.SensorValues{}
	if err := dev.Sense(&values); err != nil {
		log.Fatalf("sense: %w", err)
	}

	fmt.Printf("Distance: %s\n", values.Distance)

	return nil
}
