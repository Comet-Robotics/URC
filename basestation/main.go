package main

import (
	"basestation/rover-msgs/go/msgspb"
	"basestation/serial"
	"basestation/video_stream"
	"encoding/binary"
	"encoding/json"
	"log/slog"
	"net"
	"net/http"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/gorilla/websocket" // Import the gorilla/websocket library
	"github.com/pion/webrtc/v4"
	spa "github.com/roberthodgen/spa-server"
	"google.golang.org/protobuf/proto"
)

// Add this near the top of main.go with other type definitions
type RoverConnectionStatus struct {
	RoverAddress interface{} `json:"roverAddress"` // Can be string or bool
}

// Define a global upgrader for WebSocket connections
var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin: func(r *http.Request) bool {
		return true // Allow all origins (for development).  In production, _restrict_ this!
	},
}

// Global variable to hold the TCP connection
var tcpConn net.Conn
var tcpConnMutex sync.Mutex

var videoStreams = make(map[string]*webrtc.TrackLocalStaticRTP)

// Map to store WebSocket connections
var websocketConnections = make(map[*websocket.Conn]bool)
var websocketConnectionsMutex sync.Mutex

// Add these at package level
var (
	// Add channel to coordinate shutdown
	shutdownChan = make(chan struct{})
	wg           sync.WaitGroup
	tcpListener  net.Listener
)

// Add to the top-level variables
var serialConn *serial.SerialConnection

func main() {
	handler := slog.NewTextHandler(os.Stdout, &slog.HandlerOptions{Level: slog.LevelDebug})
	slog.SetDefault(slog.New(handler))
	// Create a channel to listen for OS signals
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	// Start the TCP server in a goroutine
	wg.Add(1)
	go startTCPServer()

	rgb_video := video_stream.StartRTPToWebRTC(5000, "rgb_video", shutdownChan)
	depth_video := video_stream.StartRTPToWebRTC(5001, "depth_video", shutdownChan)
	videoStreams[rgb_video.ID()] = rgb_video
	videoStreams[depth_video.ID()] = depth_video

	// Initialize serial connection (add this before starting the HTTP server)
	// serial, err := serial.NewSerialConnection("/dev/tty.usbmodem3301", 115200)
	// serialConn = serial
	// if err != nil {
	// 	slog.Error("Error opening serial port", "error", err)
	// } else {
	// 	defer serialConn.Close()
	// }
	
	// // After initializing serial connection, start the serial handler
	// if serialConn != nil {
	// 	wg.Add(1)
	// 	go handleSerialMessages()
	// }

	// HTTP Server setup
	http.Handle("/", spa.SpaHandler("ui/dist", "index.html"))
	// Add the WebSocket handler
	http.HandleFunc("/ws", handleWebSocket)

	go func() {
		slog.Info("HTTP server listening on :8080")
		if err := http.ListenAndServe(":8080", nil); err != nil {
			slog.Error("HTTP server error", "error", err)
			sigChan <- syscall.SIGTERM // Signal other goroutines to shut down
		}
	}()

	// Block until a signal is received
	<-sigChan
	slog.Info("\nReceived shutdown signal. Cleaning up...")

	// Close TCP listener first to stop accepting new connections
	if tcpListener != nil {
		tcpListener.Close()
	}

	// Signal all goroutines to stop
	close(shutdownChan)

	// Wait for goroutines to finish
	slog.Info("Waiting for goroutines to finish...")
	wg.Wait()
	slog.Info("All goroutines finished")

	// Perform cleanup
	tcpConnMutex.Lock()
	if tcpConn != nil {
		tcpConn.Close()
	}
	tcpConnMutex.Unlock()

	// Close all WebSocket connections
	websocketConnectionsMutex.Lock()
	for conn := range websocketConnections {
		conn.Close()
	}
	websocketConnectionsMutex.Unlock()

	// Add to the cleanup section
	if serialConn != nil {
		serialConn.Close()
	}

	slog.Info("Shutdown complete.")

}

func startTCPServer() {
	defer wg.Done()
	var err error
	tcpListener, err = net.Listen("tcp", ":8000") // Choose a different port
	if err != nil {
		slog.Error("TCP server error", "error", err)
		return
	}
	defer tcpListener.Close()
	slog.Info("TCP server listening on :8000")

	for {
		select {
		case <-shutdownChan:
			return
		default:
			tcpListener.(*net.TCPListener).SetDeadline(time.Now().Add(time.Second))
			conn, err := tcpListener.Accept()
			if err != nil {
				if opErr, ok := err.(*net.OpError); ok && opErr.Timeout() {
					continue
				}
				return
			}

			tcpConnMutex.Lock()
			if tcpConn != nil {
				tcpConn.Close() // Close any previous connection
			}
			tcpConn = conn
			tcpConnMutex.Unlock()

			go handleTCPConnection(conn)
		}
	}
}

func handleTCPConnection(conn net.Conn) {
	// Get remote address when connection is established
	remoteAddr := conn.RemoteAddr().(*net.TCPAddr).IP.String()
	slog.Debug("TCP connection established", "addr", remoteAddr)
	// Notify all WebSocket clients about new connection
	status := RoverConnectionStatus{
		RoverAddress: remoteAddr,
	}
	broadcastConnectionStatus(status)

	defer func() {
		conn.Close()
		tcpConnMutex.Lock()
		if tcpConn == conn {
			tcpConn = nil
			// Notify all WebSocket clients about disconnection
			status := RoverConnectionStatus{
				RoverAddress: false,
			}
			broadcastConnectionStatus(status)
		}
		tcpConnMutex.Unlock()
		slog.Info("TCP connection closed")
	}()

	for {
		// Read the length prefix (4 bytes)
		lengthBytes := make([]byte, 4)
		n, err := conn.Read(lengthBytes)
		if err != nil {
			slog.Error("TCP read error (length)", "error", err)
			return
		}
		if n != 4 {
			slog.Error("TCP read error (incomplete length)", "bytes", n, "expected", 4)
			return
		}

		messageLength := binary.BigEndian.Uint32(lengthBytes)

		// Read the message body

		messageBuffer := make([]byte, messageLength)
		n, err = conn.Read(messageBuffer)
		if err != nil {
			slog.Error("TCP read error (message)", "error", err)
			return
		}
		if uint32(n) != messageLength {
			slog.Error("TCP read error (incomplete message)", "bytes", n, "expected", messageLength)
			return
		}

		// Unmarshal the protobuf message
		message := &msgspb.Message{}
		if err := proto.Unmarshal(messageBuffer, message); err != nil {
			slog.Error("Failed to parse message", "error", err)
			continue // Or handle the error more robustly
		}

		slog.Debug("Received from TCP client", "type", message.ProtoReflect().WhichOneof(message.ProtoReflect().Descriptor().Oneofs().ByName("data_type")).FullName())

		// Forward to all WebSocket clients
		forwardToAllWebSockets(message)
	}
}

// forwardToAllWebSockets sends a protobuf message to all connected WebSocket clients.
func forwardToAllWebSockets(message *msgspb.Message) {
	websocketConnectionsMutex.Lock()
	defer websocketConnectionsMutex.Unlock()

	msg, err := proto.Marshal(message)
	if err != nil {
		slog.Error("Error marshaling proto message", "error", err)
		return
	}

	for conn := range websocketConnections {
		err := conn.WriteMessage(websocket.BinaryMessage, msg)
		if err != nil {
			slog.Error("Error writing message to websocket", "error", err)
			// Remove the connection if there's an error
			delete(websocketConnections, conn)
			conn.Close()
		}
	}
}

// handleWebSocket handles WebSocket connections
func handleWebSocket(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		slog.Error("WebSocket upgrade failed", "error", err)
		return
	}

	websocketConnectionsMutex.Lock()
	websocketConnections[conn] = true // Add the new connection to the map
	websocketConnectionsMutex.Unlock()
	tcpConnMutex.Lock()

	status := RoverConnectionStatus{
		RoverAddress: false,
	}
	if tcpConn != nil {
		status.RoverAddress = tcpConn.RemoteAddr().String()
	}
	tcpConnMutex.Unlock()

	if err = conn.WriteJSON(status); err != nil {
		slog.Error("Error writing connection status to websocket", "error", err)
		delete(websocketConnections, conn)
		conn.Close()
		return
	}

	peerConnection, err := webrtc.NewPeerConnection(webrtc.Configuration{})
	if err != nil {
		panic(err)
	}

	peerConnection.OnICECandidate(func(candidate *webrtc.ICECandidate) {
		if candidate == nil {
			return
		}

		if err = conn.WriteJSON(candidate.ToJSON()); err != nil {
			slog.Error("Error writing to JSON", "error", err)
			return
		}
	})

	peerConnection.OnICEConnectionStateChange(func(connectionState webrtc.ICEConnectionState) {
		slog.Info("ICE Connection State changed", "state", connectionState.String())
	})

	defer func() {
		websocketConnectionsMutex.Lock()
		delete(websocketConnections, conn) // Remove the connection when it's closed
		websocketConnectionsMutex.Unlock()
		conn.Close()
		slog.Info("WebSocket connection closed")
	}()

	for track := range videoStreams {
		slog.Debug("Adding Video Stream")
		rtpSender, err := peerConnection.AddTrack(videoStreams[track])
		if err != nil {
			panic(err)
		}
		go func() {
			rtcpBuf := make([]byte, 1500)
			for {
				if _, _, rtcpErr := rtpSender.Read(rtcpBuf); rtcpErr != nil {
					return
				}
			}
		}()
	}

	slog.Info("WebSocket connection established")

	for {
		messageType, p, err := conn.ReadMessage()
		if err != nil {
			slog.Error("Error reading WebSocket message", "error", err)
			// Remove the connection if there's an error
			websocketConnectionsMutex.Lock()
			delete(websocketConnections, conn)
			websocketConnectionsMutex.Unlock()
			return
		}

		// Only process binary messages
		if messageType == websocket.BinaryMessage {
			message := &msgspb.Message{}
			if err := proto.Unmarshal(p, message); err != nil {
				slog.Error("Failed to parse WebSocket message", "error", err)
				continue // Or handle the error more robustly
			}

			slog.Debug("Received from WebSocket client", "type", message.ProtoReflect().WhichOneof(message.ProtoReflect().Descriptor().Oneofs().ByName("data_type")).FullName())

			// Forward to TCP server
			forwardToRover(message)
		}
		if messageType == websocket.TextMessage {
			var (
				candidate webrtc.ICECandidateInit
				offer     webrtc.SessionDescription
			)

			switch {
			// Attempt to unmarshal as a SessionDescription. If the SDP field is empty
			// assume it is not one.
			case json.Unmarshal(p, &offer) == nil && offer.SDP != "":
				slog.Info("Received Offer")
				if err = peerConnection.SetRemoteDescription(offer); err != nil {
					panic(err)
				}

				answer, answerErr := peerConnection.CreateAnswer(nil)
				if answerErr != nil {
					panic(answerErr)
				}

				if err = peerConnection.SetLocalDescription(answer); err != nil {
					panic(err)
				}

				if err = conn.WriteJSON(answer); err != nil {
					panic(err)
				}
			// Attempt to unmarshal as a ICECandidateInit. If the candidate field is empty
			// assume it is not one.
			case json.Unmarshal(p, &candidate) == nil && candidate.Candidate != "":
				slog.Info("Adding Candidate")
				if err = peerConnection.AddICECandidate(candidate); err != nil {
					panic(err)
				}
			default:
				var msg map[string]interface{}
				if err := json.Unmarshal(p, &msg); err != nil {
					slog.Error("Error unmarshalling JSON", "error", err)
					return
				}
				slog.Warn("Unknown Message", "msg", msg)
			}
		}
	}
}

// forwardToRover sends a protobuf message to the TCP server.
func forwardToRover(message *msgspb.Message) {
	// Forward to TCP
	tcpConnMutex.Lock()
	if tcpConn != nil {
		// Marshal the protobuf message
		data, err := proto.Marshal(message)
		if err != nil {
			slog.Error("Error marshaling proto message", "error", err)
			return
		}

		// Create length prefix (4 bytes, big-endian)
		lengthPrefix := make([]byte, 4)
		binary.BigEndian.PutUint32(lengthPrefix, uint32(len(data)))

		// Write length prefix first
		_, err = tcpConn.Write(lengthPrefix)
		if err != nil {
			slog.Error("Error writing length prefix to TCP server", "error", err)
			return
		}

		// Write message data
		_, err = tcpConn.Write(data)
		if err != nil {
			slog.Error("Error writing message to TCP server", "error", err)
			return
		}
	}
	tcpConnMutex.Unlock()

	// Forward to Serial
	if serialConn != nil {
		if err := serialConn.WriteProto(message); err != nil {
			slog.Error("Error writing to serial", "error", err)
		}
	}
}

// Add this new function to handle serial messages
func handleSerialMessages() {
	defer wg.Done()

	for {
		slog.Debug("Serial handler running")
		select {
		case <-shutdownChan:
			slog.Info("Serial handler shutting down")
			return
		default:
			message, err := serialConn.ReadProto()
			if err != nil {
				slog.Error("Error reading from serial", "error", err)
			
				continue
			}

			slog.Debug("Received from Serial", "message", message)
		}
	}
}

// Add this new function to handle broadcasting connection status
func broadcastConnectionStatus(status RoverConnectionStatus) {
	websocketConnectionsMutex.Lock()
	defer websocketConnectionsMutex.Unlock()

	for conn := range websocketConnections {
		err := conn.WriteJSON(status)
		if err != nil {
			slog.Error("Error writing connection status to websocket", "error", err)
			delete(websocketConnections, conn)
			conn.Close()
		}
	}
}
