package main

import (
    "fmt"
    "log"
    "net"
    "net/http"
    "os"
    "os/signal"
    "sync"
    "syscall"

    "google.golang.org/protobuf/proto"

    spa "github.com/roberthodgen/spa-server"
    "github.com/gorilla/websocket" // Import the gorilla/websocket library

    "basestation/go/msgspb"
)

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

// Map to store WebSocket connections
var websocketConnections = make(map[*websocket.Conn]bool)
var websocketConnectionsMutex sync.Mutex

func main() {
    // Create a channel to listen for OS signals
    sigChan := make(chan os.Signal, 1)
    signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

    // Start the TCP server in a goroutine
    go startTCPServer()

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
    fmt.Println("Received shutdown signal.  Exiting...")

    // Perform any cleanup here
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

    fmt.Println("Shutdown complete.")
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
    defer func() {
        conn.Close()
        tcpConnMutex.Lock()
        if tcpConn == conn {
            tcpConn = nil // Clear the global connection if it's the one closing
        }
        tcpConnMutex.Unlock()
        fmt.Println("TCP connection closed")
    }()

    buffer := make([]byte, 1024) // Adjust buffer size as needed

    for {
        n, err := conn.Read(buffer)
        if err != nil {
            fmt.Printf("TCP read error: %v\n", err)
            return
        }

        message := &msgspb.Message{}
        if err := proto.Unmarshal(buffer[:n], message); err != nil {
            log.Println("Failed to parse message:", err)
            continue // Or handle the error more robustly
        }

        fmt.Printf("Received from TCP client: %s\n", message)

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

    defer func() {
        websocketConnectionsMutex.Lock()
        delete(websocketConnections, conn) // Remove the connection when it's closed
        websocketConnectionsMutex.Unlock()
        conn.Close()
        fmt.Println("WebSocket connection closed")
    }()

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

            fmt.Printf("Received from WebSocket client: %s\n", message)

            // Forward to TCP server
            forwardToTCPServer(message)
        } else {
            fmt.Println("Received non-binary message. Ignoring.")
        }
    }
}

// forwardToTCPServer sends a protobuf message to the TCP server.
func forwardToTCPServer(message *msgspb.Message) {
    tcpConnMutex.Lock()
    defer tcpConnMutex.Unlock()

    if tcpConn != nil {
        data, err := proto.Marshal(message)
        if err != nil {
            fmt.Println("Error marshaling proto message:", err)
            return
        }

        _, err = tcpConn.Write(data)
        if err != nil {
            fmt.Println("Error writing to TCP server:", err)
        }
    } else {
        fmt.Println("No TCP connection available.")
    }
}
