import React, { useState } from 'react';

const StartStream = () => {
  const [logMessages, setLogMessages] = useState<string[]>([]);
  const [remoteVideos, setRemoteVideos] = useState<HTMLVideoElement[]>([]);

  const log = (message:string) => {
    setLogMessages((prevMessages) => [...prevMessages, message]);
  };

  const startSession = async () => {
    console.log("Starting Session")
    const pc = new RTCPeerConnection({
      iceServers: [
        {
          urls: 'stun:stun.l.google.com:19302'
        }
      ]
    });



    // Handle incoming tracks
    pc.ontrack = (event) => {
      const el = document.createElement(event.track.kind) as HTMLVideoElement;
      el.srcObject = event.streams[0];
      el.autoplay = true;
      el.controls = true;
      setRemoteVideos((prevVideos) => [...prevVideos, el]);
    };

    // Log the ICE connection state
    pc.oniceconnectionstatechange = () => log(pc.iceConnectionState);

    // Handle ICE candidate event
    pc.onicecandidate = async (event) => {
        console.log(event);
      if (event.candidate === null) {
        console.log("Sending local descriptor")
        // When ICE gathering is complete, encode local session description
        const localDescriptor = btoa(JSON.stringify(pc.localDescription));

        try {
          // Send the local descriptor to the server and receive the remote descriptor
          const response = await fetch('/start_stream', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ localSessionDescription: localDescriptor })
          });

          if (response.ok) {
            const  remoteSessionDescription = await response.text();
            pc.setRemoteDescription(new RTCSessionDescription(JSON.parse(atob(remoteSessionDescription))));
            log("Session established.");
          } else {
            log("Failed to start stream.");
          }
        } catch (error) {
          console.error('Error exchanging session descriptors:', error);
          log('Error exchanging session descriptors.');
        }
      }
    };

    // Set up transceivers and create an offer
    pc.addTransceiver('video', { direction: 'recvonly' });

    try {
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);
    } catch (error) {
      log('Failed to create offer.');
    }
  };

  return (
    <div>
      <button onClick={startSession}>Start Stream</button>
      <div id="log">
        {logMessages.map((msg, index) => (
          <p key={index}>{msg}</p>
        ))}
      </div>
      <div id="remoteVideos">
        {remoteVideos.map((videoElement, index) => (
          <div key={index} ref={(ref) => ref && ref.appendChild(videoElement)} />
        ))}
      </div>
    </div>
  );
};

export default StartStream;
