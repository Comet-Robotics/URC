package main

import (
	"basestation/rover-msgs/go/msgspb"
	"basestation/video_stream"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"log"
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
var videoStreamsMutex sync.Mutex

// Map to store WebSocket connections
var websocketConnections = make(map[*websocket.Conn]bool)
var websocketConnectionsMutex sync.Mutex

// Add these at package level
var (
	// Add channel to coordinate shutdown
	shutdownChan = make(chan struct{})
)

func main() {
	// Create a channel to listen for OS signals
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	// Start the TCP server in a goroutine
	go startTCPServer()

	rgb_video := video_stream.StartRTPToWebRTC(5000, "rgb_video", shutdownChan)
	depth_video := video_stream.StartRTPToWebRTC(5001, "depth_video", shutdownChan)
	videoStreams[rgb_video.ID()] = rgb_video
	videoStreams[depth_video.ID()] = depth_video

	// HTTP Server setup
	http.Handle("/", spa.SpaHandler("ui/dist", "index.html"))

	// Add the WebSocket handler
	http.HandleFunc("/ws", handleWebSocket)

	go func() {
		fmt.Println("HTTP server listening on :8080")
		if err := http.ListenAndServe(":8080", nil); err != nil {
			fmt.Printf("HTTP server error: %v\n", err)
			sigChan <- syscall.SIGTERM // Signal other goroutines to shut down
		}
	}()

	// Block until a signal is received
	<-sigChan
	fmt.Println("\nReceived shutdown signal. Cleaning up...")

	// Signal all goroutines to stop
	close(shutdownChan)

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

	// Give goroutines time to clean up
	time.Sleep(time.Second)
	fmt.Println("Shutdown complete.")
	os.Exit(0)
}

func startTCPServer() {
	listener, err := net.Listen("tcp", ":8000") // Choose a different port
	if err != nil {
		fmt.Printf("TCP server error: %v\n", err)
		return
	}
	defer listener.Close()
	fmt.Println("TCP server listening on :8000")

	for {
		conn, err := listener.Accept()
		if err != nil {
			fmt.Printf("TCP accept error: %v\n", err)
			continue
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

func handleTCPConnection(conn net.Conn) {
	// Get remote address when connection is established
	remoteAddr := conn.RemoteAddr().(*net.TCPAddr).IP.String()

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
		fmt.Println("TCP connection closed")
	}()

	for {
		// Read the length prefix (4 bytes)
		lengthBytes := make([]byte, 4)
		n, err := conn.Read(lengthBytes)
		if err != nil {
			fmt.Printf("TCP read error (length): %v\n", err)
			return
		}
		if n != 4 {
			fmt.Printf("TCP read error (incomplete length): read %d bytes, expected 4\n", n)
			return
		}

		messageLength := binary.BigEndian.Uint32(lengthBytes)

		// Read the message body
		messageBuffer := make([]byte, messageLength)
		n, err = conn.Read(messageBuffer)
		if err != nil {
			fmt.Printf("TCP read error (message): %v\n", err)
			return
		}
		if uint32(n) != messageLength {
			fmt.Printf("TCP read error (incomplete message): read %d bytes, expected %d\n", n, messageLength)
			return
		}

		// Unmarshal the protobuf message
		message := &msgspb.Message{}
		if err := proto.Unmarshal(messageBuffer, message); err != nil {
			log.Println("Failed to parse message:", err)
			continue // Or handle the error more robustly
		}

		fmt.Printf("Received from TCP client: %s\n",message.ProtoReflect().WhichOneof(message.ProtoReflect().Descriptor().Oneofs().ByName("data_type")).FullName())

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
		fmt.Printf("error marshaling proto message: %v", err)
		return
	}

	for conn := range websocketConnections {
		err := conn.WriteMessage(websocket.BinaryMessage, msg)
		if err != nil {
			fmt.Printf("error writing message to websocket: %v", err)
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
		log.Println(err)
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
		fmt.Printf("error writing connection status to websocket: %v", err)
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

			fmt.Println("Error Writing to JSON", err)
			return
		}
	})

	peerConnection.OnICEConnectionStateChange(func(connectionState webrtc.ICEConnectionState) {
		fmt.Printf("ICE Connection State has changed: %s\n", connectionState.String())
	})

	defer func() {
		websocketConnectionsMutex.Lock()
		delete(websocketConnections, conn) // Remove the connection when it's closed
		websocketConnectionsMutex.Unlock()
		conn.Close()
		fmt.Println("WebSocket connection closed")
	}()

	for track := range videoStreams {
		fmt.Println("Adding Video Stream")
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

	fmt.Println("WebSocket connection established")

	for {
		messageType, p, err := conn.ReadMessage()
		if err != nil {
			log.Println(err)
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
				log.Println("Failed to parse WebSocket message:", err)
				continue // Or handle the error more robustly
			}

			fmt.Printf("Received from WebSocket client: %s\n",message.ProtoReflect().WhichOneof(message.ProtoReflect().Descriptor().Oneofs().ByName("data_type")).FullName())


			// Forward to TCP server
			forwardToTCPServer(message)
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
				fmt.Println("Received Offer")
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
				fmt.Println("Adding Candidate")
				if err = peerConnection.AddICECandidate(candidate); err != nil {
					panic(err)
				}
			default:
				var msg map[string]interface{}
				if err := json.Unmarshal(p, &msg); err != nil {
					fmt.Println("Error unmarshalling JSON", err)
					return
				}
				fmt.Println("Unknown Message", msg)

			}
		}
	}
}

// forwardToTCPServer sends a protobuf message to the TCP server.
func forwardToTCPServer(message *msgspb.Message) {
	tcpConnMutex.Lock()
	defer tcpConnMutex.Unlock()

	if tcpConn != nil {
		// Marshal the protobuf message
		data, err := proto.Marshal(message)
		if err != nil {
			fmt.Println("Error marshaling proto message:", err)
			return
		}

		// Create length prefix (4 bytes, big-endian)
		lengthPrefix := make([]byte, 4)
		binary.BigEndian.PutUint32(lengthPrefix, uint32(len(data)))

		// Write length prefix first
		_, err = tcpConn.Write(lengthPrefix)
		if err != nil {
			fmt.Println("Error writing length prefix to TCP server:", err)
			return
		}

		// Write message data
		_, err = tcpConn.Write(data)
		if err != nil {
			fmt.Println("Error writing message to TCP server:", err)
			return
		}
	} else {
		fmt.Println("No TCP connection available.")
	}
}

// Add this new function to handle broadcasting connection status
func broadcastConnectionStatus(status RoverConnectionStatus) {
	websocketConnectionsMutex.Lock()
	defer websocketConnectionsMutex.Unlock()

	for conn := range websocketConnections {
		err := conn.WriteJSON(status)
		if err != nil {
			fmt.Printf("error writing connection status to websocket: %v", err)
			delete(websocketConnections, conn)
			conn.Close()
		}
	}
}
