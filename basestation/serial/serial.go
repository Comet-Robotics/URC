package serial

import (
	"basestation/rover-msgs/go/msgspb"
	"bufio"
	"encoding/base64"
	"encoding/binary"
	"fmt"
	"log/slog"
	"strings"
	"go.bug.st/serial"
	"google.golang.org/protobuf/proto"
)

type SerialConnection struct {
	inner serial.Port
	port     *bufio.ReadWriter
	portName string
	baudRate int
	
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
	buf_port := bufio.NewReadWriter(bufio.NewReader(port), bufio.NewWriter(port))
	return &SerialConnection{
		inner: port,
		port:     buf_port,
		portName: portName,
		baudRate: baudRate,
	}, nil
}


func (s *SerialConnection) Close() error {

	return s.inner.Close()
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


// ReadProto reads a protobuf message from the serial port
func (s *SerialConnection) ReadProto() (*msgspb.Message, error) {

	buf,err := s.port.ReadString('\n')

	buf = strings.TrimSuffix(buf,"\n")
	if err != nil {
		return nil, err
	}
	slog.Debug("Serial received", "data", buf)

	index := -1
	if index = strings.Index(buf, "[PACKET RX]"); index == -1 {
		
		return nil, fmt.Errorf("invalid packet received: %s", buf)
	}
	buf = buf[index+11:]
	// Decode base64
	slog.Debug("Processing buffer", "data", buf)
	decoded, err := base64.RawStdEncoding.DecodeString(buf)
	if err != nil {
		return nil, fmt.Errorf("failed to decode base64: %v", err)
	}
	slog.Debug("Decoded message", "bytes", decoded)

	// Unmarshal protobuf message
	message := &msgspb.Message{}
	if err := proto.Unmarshal(decoded, message); err != nil {
		return nil, fmt.Errorf("failed to unmarshal proto message: %v", err)
	}

	return message, nil
}
