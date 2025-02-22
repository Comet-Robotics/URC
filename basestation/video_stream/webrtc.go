// SPDX-FileCopyrightText: 2023 The Pion community <https://pion.ly>
// SPDX-License-Identifier: MIT

//go:build !js
// +build !js

// rtp-to-webrtc demonstrates how to consume a RTP stream video UDP, and then send to a WebRTC client.
package video_stream

import (
	"errors"
	"fmt"
	"io"
	"net"
	"os"
	"time"

	"github.com/pion/webrtc/v4"
)

// nolint:cyclop
func StartRTPToWebRTC(port int, track_id string, shutdown chan struct{}) *webrtc.TrackLocalStaticRTP {
	// Open a UDP Listener for RTP Packets on port 5004
	// Create a video track
	videoTrack, err := webrtc.NewTrackLocalStaticRTP(

		webrtc.RTPCodecCapability{MimeType: webrtc.MimeTypeH264}, track_id, track_id,
	)

	if err != nil {
		panic(err)
	}
	// Output the answer in base64 so we can paste it in browser
	// Read RTP packets forever and send them to the WebRTC Client
	inboundRTPPacket := make([]byte, 1600) // UDP MTU
	go func() {
		listener, err := net.ListenUDP("udp", &net.UDPAddr{IP: net.ParseIP("0.0.0.0"), Port: port})
		if err != nil {
			panic(err)
		}

		// Add timeout checking
		lastPacketTime := time.Now()
		timeoutCheck := time.NewTicker(1 * time.Second)
		defer timeoutCheck.Stop()

		// Clean shutdown of listener
		defer func() {
			if err = listener.Close(); err != nil {
				fmt.Printf("Error closing listener on port %d: %v\n", port, err)
			}
		}()

		// Timeout checker goroutine
		go func() {
			isConnected := true // Track connection state
			for {
				select {
				case <-timeoutCheck.C:
					if time.Since(lastPacketTime) > 5*time.Second {
						if isConnected { // Only log when state changes
							fmt.Printf("Warning: No video received on port %d\n", port)
							isConnected = false
						}
					} else if !isConnected { // Video is back
						fmt.Printf("Video connection restored on port %d\n", port)
						isConnected = true
					}
				case <-shutdown:
					fmt.Printf("Shutting down timeout checker %d\n", port)
					return
				}
			}
		}()

		// Main packet reading loop
		for {
			select {
			case <-shutdown:
				fmt.Printf("Shutting down video stream on port %d\n", port)
				return
			default:
				// Set a read deadline to prevent blocking forever
				listener.SetReadDeadline(time.Now().Add(time.Second))
				n, _, err := listener.ReadFrom(inboundRTPPacket)
				if err != nil {
					if !errors.Is(err, os.ErrDeadlineExceeded) {
						fmt.Printf("Error reading from port %d: %v\n", port, err)
					}
					continue
				}

				lastPacketTime = time.Now()

				if _, err = videoTrack.Write(inboundRTPPacket[:n]); err != nil {
					if errors.Is(err, io.ErrClosedPipe) {
						return
					}
					fmt.Printf("Error writing to video track: %v\n", err)
				}
			}
		}
	}()
	return videoTrack
}
