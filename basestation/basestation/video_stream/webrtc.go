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
	"github.com/pion/webrtc/v4"
)

// nolint:cyclop
func StartRTPToWebRTC(port int , track_id string) *webrtc.TrackLocalStaticRTP {
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

    // Increase the UDP receive buffer size
    // Default UDP buffer sizes vary on different operating systems
    bufferSize := 300000 // 300KB
    err = listener.SetReadBuffer(bufferSize)
    if err != nil {
      panic(err)
    }

    defer func() {
      if err = listener.Close(); err != nil {
        panic(err)
      }
    }()

    fmt.Println("Listener started waiting for packets on port ", port)


    for {
		  n, _, err := listener.ReadFrom(inboundRTPPacket)
		  if err != nil {
			  panic(fmt.Sprintf("error during read: %s", err))
		  }

      if _, err = videoTrack.Write(inboundRTPPacket[:n]); err != nil {
        if errors.Is(err, io.ErrClosedPipe) {
          // The peerConnection has been closed.
          return
        }

        panic(err)
      }
    }
  }()
  return videoTrack;
}


