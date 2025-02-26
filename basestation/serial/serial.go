package serial

import (
	"basestation/rover-msgs/go/msgspb"
	"encoding/base64"
	"encoding/binary"
	"fmt"
	"strings"
	"sync"
	"time"

	"go.bug.st/serial"
	"google.golang.org/protobuf/proto"
)

type SerialConnection struct {
	port     serial.Port
	portName string
	baudRate int
	mu       sync.Mutex
}

func NewSerialConnection(portName string, baudRate int) (*SerialConnection, error) {
	mode := &serial.Mode{
		BaudRate: baudRate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	port, err := serial.Open(portName, mode)
	if err != nil {
		return nil, fmt.Errorf("failed to open serial port: %v", err)
	}

	return &SerialConnection{
		port:     port,
		portName: portName,
		baudRate: baudRate,
	}, nil
}

func (s *SerialConnection) Write(data []byte) (int, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.port.Write(data)
}

func (s *SerialConnection) Read(buffer []byte) (int, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.port.Read(buffer)
}

func (s *SerialConnection) Close() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.port.Close()
}

// ListPorts returns a list of available serial ports
func ListPorts() ([]string, error) {
	ports, err := serial.GetPortsList()
	if err != nil {
		return nil, fmt.Errorf("failed to get serial ports list: %v", err)
	}
	return ports, nil
}

// WriteProto writes a protobuf message to the serial port with length prefix
func (s *SerialConnection) WriteProto(message *msgspb.Message) error {
	data, err := proto.Marshal(message)
	if err != nil {
		return fmt.Errorf("failed to marshal proto message: %v", err)
	}

	// Create length prefix (4 bytes, big-endian)
	lengthPrefix := make([]byte, 4)
	binary.BigEndian.PutUint32(lengthPrefix, uint32(len(data)))

	s.mu.Lock()
	defer s.mu.Unlock()

	// Write length prefix
	if _, err := s.port.Write(lengthPrefix); err != nil {
		return fmt.Errorf("failed to write length prefix: %v", err)
	}

	// Write message data
	if _, err := s.port.Write(data); err != nil {
		return fmt.Errorf("failed to write message data: %v", err)
	}

	return nil
}

// Add this method to allow setting read timeout
func (s *SerialConnection) SetReadTimeout(d time.Duration) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.port.SetReadTimeout(d)
}

// ReadProto reads a protobuf message from the serial port
func (s *SerialConnection) ReadProto() (*msgspb.Message, error) {
	s.mu.Lock()
	defer s.mu.Unlock()


	buf, err := s.ReadLine()
	if err != nil {
		return nil, err
	}
	fmt.Println("Recieved: ", buf)

	index:= -1;
	if index = strings.Index(buf,"[PACKET RX]"); index == -1 {
		return nil, fmt.Errorf("invalid packet received: %s", buf)
	}
	buf = strings.TrimSpace(buf[index+11:])
	// Decode base64
	fmt.Println("Buffer: ", buf)	
	decoded, err := base64.RawStdEncoding.DecodeString(buf)
	if err != nil {
		return nil, fmt.Errorf("failed to decode base64: %v", err)
	}
	fmt.Println("Decoded: ", decoded)
	

	
	// Unmarshal protobuf message
	message := &msgspb.Message{}
	if err := proto.Unmarshal(decoded, message); err != nil {
		return nil, fmt.Errorf("failed to unmarshal proto message: %v", err)
	}

	return message, nil
}

// ReadLine reads until a newline character
func (s *SerialConnection) ReadLine() (string, error) {

	var line []byte
	buf := make([]byte, 1)

	for {
		n, err := s.port.Read(buf)
		if err != nil {
			return "", err
		}
		if n == 0 {
			continue
		}

		if buf[0] == '\n' {
			return string(line), nil
		}
		line = append(line, buf[0])
	}
}
